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
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.refiners.RefinerException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.MultiDimensionFunctionEvaluatorListener;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.numerical.robust.InliersData;

import java.util.BitSet;
import java.util.List;

/**
 * Refines the epipole of a fundamental matrix formed by an initial epipole 
 * estimation and an estimated homography.
 * Any fundamental matrix can be expressed as F = [e']x*H, where 
 * e' is the epipole on the right view, []x is the skew matrix, and H is a non
 * degenerate homography.
 * This class refines an initial epipole so that residuals from provided point
 * correspondences generating fundamental matrix F are reduced.
 * This class is especially useful in cases where geometry of the scene is 
 * degenerate (e.g. planar scene) and provided point correspondences would
 * generate an inaccurate fundamental matrix.
 * This implementation uses Levenberg-Marquardt algorithm for a fast cost 
 * optimization and uses inhomogeneous points for epipole refinement,
 * which makes it unsuitable for cases when epipoles are are or near infinity
 * (i.e pure lateral translations).
 */
@SuppressWarnings("WeakerAccess")
public class InhomogeneousRightEpipoleRefiner extends RightEpipoleRefiner {
    
    /**
     * Maximum allowed number of refinement iterations.
     */
    public static int MAX_ITERS = 10;   
    
    /**
     * Constructor.
     */
    public InhomogeneousRightEpipoleRefiner() { }
    
    /**
     * Constructor.
     * @param initialEpipoleEstimation initial right epipole estimation to be 
     * set and refined.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliers set indicating which of the provided matches are inliers.
     * @param residuals residuals for matched samples.
     * @param numInliers number of inliers on initial estimation.
     * @param samples1 1st set of paired samples.
     * @param samples2 2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for 
     * Levenberg-Marquardt fitting.
     * @param homography homography relating samples in two views, which is
     * used to generate a fundamental matrix and its corresponding epipolar 
     * geometry.
     */
    public InhomogeneousRightEpipoleRefiner(Point2D initialEpipoleEstimation, 
            boolean keepCovariance, BitSet inliers, double[] residuals, 
            int numInliers, List<Point2D> samples1, List<Point2D> samples2,
            double refinementStandardDeviation, Transformation2D homography) {
        super(initialEpipoleEstimation, keepCovariance, inliers, residuals, 
                numInliers, samples1, samples2, refinementStandardDeviation, 
                homography);
    }
    
    /**
     * Constructor.
     * @param initialEpipoleEstimation initial right epipole estimation to be
     * set and refined.
     * @param keepCovariance true if covariance of estimation must be kept after
     * refinement, false otherwise.
     * @param inliersData inlier data, typically obtained from a robust 
     * estimator.
     * @param samples1 1st set of paired samples.
     * @param samples2 2nd set of paired samples.
     * @param refinementStandardDeviation standard deviation used for 
     * Levenberg-Marquardt fitting.
     * @param homography homography relating samples in two views, which is used
     * to generate a fundamental matrix and its corresponding epipolar geometry.
     */
    public InhomogeneousRightEpipoleRefiner(Point2D initialEpipoleEstimation,
            boolean keepCovariance, InliersData inliersData,
            List<Point2D> samples1, List<Point2D> samples2, 
            double refinementStandardDeviation, Transformation2D homography) {
        super(initialEpipoleEstimation, keepCovariance, inliersData, samples1, 
                samples2, refinementStandardDeviation, homography);
    }
    
    /**
     * Refines provided initial right epipole estimation.
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
    public boolean refine(Point2D result) throws NotReadyException, 
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
        
        try {    
            InhomogeneousPoint2D epipole = new InhomogeneousPoint2D(
                    mInitialEstimation);
            boolean errorDecreased = false, iterErrorDecreased;
            int numIter = 0;
            do {
                FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
                computeFundamentalMatrix(mHomography, epipole, 
                        fundamentalMatrix);
                double initialTotalResidual = totalResidual(fundamentalMatrix);
                final double[] initParams = epipole.asArray();
        
                //output values to be fitted/optimized will contain residuals
                double[] y = new double[mNumInliers];
                //input values will contain 2 points to compute residuals
                final int nDims = 
                        2*Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
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
                        x.setElementAt(pos, 0, leftPoint.getInhomX());
                        x.setElementAt(pos, 1, leftPoint.getInhomY());
                        x.setElementAt(pos, 2, rightPoint.getInhomX());
                        x.setElementAt(pos, 3, rightPoint.getInhomY());
                        
                        y[pos] = mResiduals[i];
                        pos++;                    
                    }
                }
            
                LevenbergMarquardtMultiDimensionFunctionEvaluator evaluator =
                        new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
                
                    private Point2D mLeftPoint = Point2D.create(
                            CoordinatesType.INHOMOGENEOUS_COORDINATES);
                    
                    private Point2D mRightPoint = Point2D.create(
                            CoordinatesType.INHOMOGENEOUS_COORDINATES);
                        
                    private InhomogeneousPoint2D mEpipole = 
                            new InhomogeneousPoint2D();
                
                    private FundamentalMatrix mFundMatrix = 
                            new FundamentalMatrix();
                
                    private GradientEstimator mGradientEstimator =
                            new GradientEstimator(
                                    new MultiDimensionFunctionEvaluatorListener() {
                        @Override
                        public double evaluate(double[] point) 
                                throws Throwable {
                        
                            mEpipole.setCoordinates(point);
                            computeFundamentalMatrix(mHomography, mEpipole, 
                                    mFundMatrix);
                            return residual(mFundMatrix, mLeftPoint, 
                                    mRightPoint);
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
                        mLeftPoint.setInhomogeneousCoordinates(point[0], 
                                point[1]);
                        mRightPoint.setInhomogeneousCoordinates(point[2], 
                                point[3]);

                        mEpipole.setCoordinates(params);
                        computeFundamentalMatrix(mHomography, mEpipole, 
                                mFundMatrix);
                    
                        double y = residual(mFundMatrix, mLeftPoint, 
                                mRightPoint);
                        mGradientEstimator.gradient(params, derivatives);
                    
                        return y;
                    }
                };
        
                LevenbergMarquardtMultiDimensionFitter fitter =
                        new LevenbergMarquardtMultiDimensionFitter(evaluator, x,
                                y, getRefinementStandardDeviation());
                
                fitter.fit();
                
                //obtain estimated params
                double[] params = fitter.getA();

                //update initial epipole for next iteration
                epipole.setInhomogeneousCoordinates(params[0], params[1]);
                  
                computeFundamentalMatrix(mHomography, epipole, 
                        fundamentalMatrix);
            
                double finalTotalResidual = totalResidual(fundamentalMatrix);
                iterErrorDecreased = finalTotalResidual < initialTotalResidual;
                if (iterErrorDecreased || !errorDecreased) {
                    errorDecreased = true;
                    //update final result
                    result.setInhomogeneousCoordinates(params[0], params[1]);
                    
                    if (mKeepCovariance) {
                        //keep covariance
                        mCovariance = fitter.getCovar();
                    }                    
                }
                numIter++;
            } while (iterErrorDecreased && numIter < MAX_ITERS);
            
            if (mListener != null) {
                //noinspection all
                mListener.onRefineEnd(this, mInitialEstimation, result, 
                        errorDecreased);
            }

            //noinspection all
            return errorDecreased;
            
        } catch (Exception e) {
            throw new RefinerException(e);
        } finally {
            mLocked = false;
        }
    }    
}
