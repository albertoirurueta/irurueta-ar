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
package com.irurueta.ar.epipolar.refiners;

import com.irurueta.algebra.Matrix;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidFundamentalMatrixException;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.refiners.PairMatchesAndInliersDataRefiner;
import com.irurueta.geometry.refiners.RefinerException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.MultiDimensionFunctionEvaluatorListener;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.numerical.robust.InliersData;

import java.util.BitSet;
import java.util.List;

/**
 * Refines a fundamental matrix by taking into account an initial estimation,
 * inlier matches and their residuals.
 * This class can be used to find a solution that minimizes error of inliers
 * in LMSE terms.
 * Typically a refiner is used by a robust estimator, however it can also be
 * useful in some other situations.
 */
public class FundamentalMatrixRefiner extends 
        PairMatchesAndInliersDataRefiner<FundamentalMatrix, Point2D, Point2D> {    
    
    /**
     * Test line to compute epipolar residuals.
     */
    private Line2D mTestLine = new Line2D();   
    
    /**
     * Standard deviation used for Levenberg-Marquardt fitting during 
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust 
     * estimation, since residuals of found inliers are within the range of 
     * such threshold.
     */
    private double mRefinementStandardDeviation;
    
    /**
     * Constructor.
     */
    public FundamentalMatrixRefiner() { }
    
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliers set indicating which of the provided matches are inliers.
     * @param residuals residuals for matched samples.
     * @param numInliers number of inliers on initial estimation.
     * @param samples1 1st set of paired samples.
     * @param samples2 2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for 
     * Levenberg-Marquardt fitting.
     */
    public FundamentalMatrixRefiner(FundamentalMatrix initialEstimation,
            boolean keepCovariance, BitSet inliers, double[] residuals,
            int numInliers, List<Point2D> samples1, List<Point2D> samples2,
            double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliers, residuals, numInliers,
                samples1, samples2);
        mRefinementStandardDeviation = refinementStandardDeviation;
    }    
    
    /**
     * Constructor.
     * @param initialEstimation initial estimation to be set.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliersData inlier data, typically obtained from a robust 
     * estimator.
     * @param samples1 1st set of paired samples.
     * @param samples2 2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for 
     * Levenberg-Marquardt fitting.
     */
    public FundamentalMatrixRefiner(FundamentalMatrix initialEstimation,
            boolean keepCovariance, InliersData inliersData, 
            List<Point2D> samples1, List<Point2D> samples2, 
            double refinementStandardDeviation) {
        super(initialEstimation, keepCovariance, inliersData, samples1, 
                samples2);
        mRefinementStandardDeviation = refinementStandardDeviation;
    }
    
    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during 
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     * @return standard deviation used for refinement.
     */
    public double getRefinementStandardDeviation() {
        return mRefinementStandardDeviation;
    }
    
    /**
     * Sets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     * @param refinementStandardDeviation standard deviation used for 
     * refinement.
     * @throws LockedException if estimator is locked.
     */
    public void setRefinementStandardDeviation(
            double refinementStandardDeviation) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRefinementStandardDeviation = refinementStandardDeviation;
    }
    
    /**
     * Refines provided initial estimation.
     * @return refined estimation.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException if estimator is locked because refinement is 
     * already in progress.
     * @throws RefinerException if refinement fails for some reason (e.g. unable 
     * to converge to a result).
     */    
    @Override
    public FundamentalMatrix refine() throws NotReadyException, 
            LockedException, RefinerException {
        FundamentalMatrix result = new FundamentalMatrix();
        refine(result);
        return result;
    }
    
    /**
     * Refines provided initial estimation.
     * This method always sets a value into provided result instance regardless
     * of the fact that error has actually improved in LMSE terms or not.
     * @param result instance where refined estimation will be stored.
     * @return true if result improves (decreases) in LMSE terms respect to 
     * initial estimation, false if no improvement has been achieved.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException if estimator is locked because refinement is 
     * already in progress.
     * @throws RefinerException if refinement fails for some reason (e.g. unable 
     * to converge to a result).
     */    
    @Override
    public boolean refine(FundamentalMatrix result) throws NotReadyException,
            LockedException, RefinerException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        mLocked = true;
        
        if (mListener != null) {
            mListener.onRefineStart(this, mInitialEstimation);
        }
        
        mInitialEstimation.normalize();
        
        final double initialTotalResidual = totalResidual(mInitialEstimation);
        
        try {
            Matrix internalMatrix = mInitialEstimation.getInternalMatrix();                
            final double[] initParams = internalMatrix.getBuffer();
            
            //output values to be fitted/optimized will contain residuals
            double[] y = new double[mNumInliers];
            //input values will contain 2 points to compute residuals
            final int nDims = 2*Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH;
            Matrix x = new Matrix(mNumInliers, nDims);
            int nSamples = mInliers.length();
            int pos = 0;
            Point2D leftPoint, rightPoint;
            for (int i = 0; i < nSamples; i++) {
                if (mInliers.get(i)) {
                    //sample is inlier
                    leftPoint = mSamples1.get(i);
                    rightPoint = mSamples2.get(i);
                    leftPoint.normalize();
                    rightPoint.normalize();
                    x.setElementAt(pos, 0, leftPoint.getHomX());
                    x.setElementAt(pos, 1, leftPoint.getHomY());
                    x.setElementAt(pos, 2, leftPoint.getHomW());
                    x.setElementAt(pos, 3, rightPoint.getHomX());
                    x.setElementAt(pos, 4, rightPoint.getHomY());
                    x.setElementAt(pos, 5, rightPoint.getHomW());
                        
                    y[pos] = mResiduals[i];
                    pos++;
                }
            }
            
            LevenbergMarquardtMultiDimensionFunctionEvaluator evaluator = 
                    new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                            
                private Point2D mLeftPoint = Point2D.create(
                        CoordinatesType.HOMOGENEOUS_COORDINATES);
                    
                private Point2D mRightPoint = Point2D.create(
                        CoordinatesType.HOMOGENEOUS_COORDINATES);
                    
                private FundamentalMatrix mFundMatrix = 
                        new FundamentalMatrix();
                    
                private Matrix mInternalMatrix;
                    
                private GradientEstimator mGradientEstimator = 
                        new GradientEstimator(
                                new MultiDimensionFunctionEvaluatorListener() {
                                    
                        @Override
                        public double evaluate(double[] params) 
                                throws Throwable {

                            mInternalMatrix.fromArray(params);
                            try {
                                mFundMatrix.setInternalMatrix(mInternalMatrix);
                            
                                return residual(mFundMatrix, mLeftPoint, 
                                        mRightPoint);
                            } catch (InvalidFundamentalMatrixException e) {
                                return initialTotalResidual;
                            }
                        }
                    });
                    
                    @Override
                    public int getNumberOfDimensions() {
                        return nDims;
                    }

                    @Override
                    public double[] createInitialParametersArray() {
                        return initParams;
                    }

                    @Override
                    public double evaluate(int i, double[] point, 
                            double[] params, double[] derivatives) 
                            throws Throwable {
                        mLeftPoint.setHomogeneousCoordinates(point[0], point[1],
                                point[2]);
                        mRightPoint.setHomogeneousCoordinates(point[3], 
                                point[4], point[5]);
                        
                        if (mInternalMatrix == null) {
                            mInternalMatrix = new Matrix(
                                    FundamentalMatrix.FUNDAMENTAL_MATRIX_ROWS, 
                                    FundamentalMatrix.FUNDAMENTAL_MATRIX_COLS);
                        }
                        mInternalMatrix.fromArray(params);
                        double y;
                        try {
                            mFundMatrix.setInternalMatrix(mInternalMatrix);
                        
                            y = residual(mFundMatrix, mLeftPoint, 
                                    mRightPoint);
                        } catch (InvalidFundamentalMatrixException e) {
                            y = initialTotalResidual;
                        }
                        mGradientEstimator.gradient(params, derivatives);
                        
                        return y;
                    }
                };
                
            LevenbergMarquardtMultiDimensionFitter fitter =
                    new LevenbergMarquardtMultiDimensionFitter(evaluator, x, y, 
                            getRefinementStandardDeviation());
                
            fitter.fit();
                
            //obtain estimated params
            double[] params = fitter.getA();
                
            //update fundamental matrix
            internalMatrix.fromArray(params);
            result.setInternalMatrix(internalMatrix);
                
            if (mKeepCovariance) {
                //keep covariance
                mCovariance = fitter.getCovar();
            }                
                
            double finalTotalResidual = totalResidual(result);
            boolean errorDecreased = finalTotalResidual < initialTotalResidual;
            
            if (mListener != null) {
                mListener.onRefineEnd(this, mInitialEstimation, result, 
                        errorDecreased);
            }
                
            return errorDecreased;
            
        } catch (Exception e) {
            throw new RefinerException(e);
        } finally {
            mLocked = false;
        }      
    }
    
    /**
     * Computes the residual between a fundamental matrix and a pair of matched
     * points.
     * @param fundamentalMatrix a fundamental matrix.
     * @param leftPoint left 2D point.
     * @param rightPoint right 2D point.
     * @return residual (distance of point to epipolar line).
     */
    private double residual(FundamentalMatrix fundamentalMatrix,
            Point2D leftPoint, Point2D rightPoint) {
        try {
            leftPoint.normalize();
            rightPoint.normalize();
            fundamentalMatrix.normalize();
            fundamentalMatrix.leftEpipolarLine(rightPoint, mTestLine);
            double leftDistance = Math.abs(mTestLine.signedDistance(
                    leftPoint));
            fundamentalMatrix.rightEpipolarLine(leftPoint, mTestLine);
            double rightDistance = Math.abs(mTestLine.signedDistance(
                    rightPoint));
            //return average distance as an error residual
            return 0.5 * (leftDistance + rightDistance);
        } catch (NotReadyException e) {
            return Double.MAX_VALUE;
        }
    }    
    
    /**
     * Computes total residual among all provided inlier samples.
     * @param fundamentalMatrix a fundamental matrix.
     * @return total residual.
     */
    private double totalResidual(FundamentalMatrix fundamentalMatrix) {
        double result = 0.0;
        
        int nSamples = mInliers.length();
        Point2D leftPoint, rightPoint;
        for (int i = 0; i < nSamples; i++) {
            if (mInliers.get(i)) {
                //sample is inlier
                leftPoint = mSamples1.get(i);
                rightPoint = mSamples2.get(i);
                leftPoint.normalize();
                rightPoint.normalize();
                result += residual(fundamentalMatrix, leftPoint, rightPoint);
            }
        }        
        
        return result;
    }
}
