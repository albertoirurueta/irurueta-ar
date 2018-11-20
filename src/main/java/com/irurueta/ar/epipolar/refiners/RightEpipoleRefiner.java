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

import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidFundamentalMatrixException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.refiners.PairMatchesAndInliersDataRefiner;
import com.irurueta.geometry.refiners.RefinerException;
import com.irurueta.numerical.robust.InliersData;

import java.util.BitSet;
import java.util.List;

/**
 * Base class to refine the epipole of a fundamental matrix formed by an initial 
 * epipole estimation and an estimated homography.
 * Any fundamental matrix can be expressed as F = [e']x*H, where
 * e' is the epipole on the right view and H is a non degenerate homography.
 * This class refines an initial epipole so that residuals from provided point
 * correspondences generating fundamental matrix F are reduced.
 * This class is especially useful in cases where geometry of the scene is 
 * degenerate (e.g. planar scene) and provided point correspondences would
 * generate an inccurate fundamental matrix.
 */
public abstract class RightEpipoleRefiner extends
        PairMatchesAndInliersDataRefiner<Point2D, Point2D, Point2D> {
    
    /**
     * Test line to compute epipolar residuals.
     */
    protected Line2D mTestLine = new Line2D();       
    
    /**
     * Homography relating two views through a given planar scene.
     */
    protected Transformation2D mHomography;
    
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
    public RightEpipoleRefiner() { }
    
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
     * @param homography homography relating samples in two views, which is used
     * to generate a fundamental matrix and its corresponding epipolar geometry.
     */
    public RightEpipoleRefiner(Point2D initialEpipoleEstimation,
            boolean keepCovariance, BitSet inliers, double[] residuals,
            int numInliers, List<Point2D> samples1, List<Point2D> samples2, 
            double refinementStandardDeviation, Transformation2D homography) {
        super(initialEpipoleEstimation, keepCovariance, inliers, residuals,
                numInliers, samples1, samples2);
        mHomography = homography;
        mRefinementStandardDeviation = refinementStandardDeviation;
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
    public RightEpipoleRefiner(Point2D initialEpipoleEstimation,
            boolean keepCovariance, InliersData inliersData,
            List<Point2D> samples1, List<Point2D> samples2,
            double refinementStandardDeviation, Transformation2D homography) {
        super(initialEpipoleEstimation, keepCovariance, inliersData, samples1, 
                samples2);
        mHomography = homography;
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
     * Gets homography relating samples in two views, which is used to generate 
     * a fundamental matrix and its corresponding epipolar geometry.
     * @return homography relating samples in two views.
     */
    public Transformation2D getHomography() {
        return mHomography;
    }
    
    /**
     * Sets homography relating samples in two views, which is used to generate
     * a fundamental matrix and its corresponding epipolar geometry.
     * @param homography homography relating samples in two views.
     * @throws LockedException if estimator is locked.
     */
    public void setHomography(Transformation2D homography) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mHomography = homography;
    }
    
    /**
     * Indicates whether this refiner is ready to start refinement computation.
     * @return true if refiner is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mHomography != null && super.isReady();
    }
    
    /**
     * Refines provided initial right epipole estimation.
     * @return refined estimation.
     * @throws NotReadyException if not enough input data has been provided.
     * @throws LockedException if estimator is locked because refinement is 
     * already in progress.
     * @throws RefinerException if refinement fails for some reason (e.g. unable
     * to converge to a result).
     */
    @Override
    public Point2D refine() throws NotReadyException, LockedException, 
            RefinerException {
        HomogeneousPoint2D result = new HomogeneousPoint2D();
        refine(result);
        return result;
    }
    
    /**
     * Computes a fundamental matrix from a 2D homography and provided epipole
     * on right view.
     * @param homography a 2D homography. Must be invertible.
     * @param rightEpipole epipole on right view.
     * @param result instance where computed fundamental matrix will be stored.
     * @throws InvalidFundamentalMatrixException if provided homography is not
     * invertible, which would generate a degenerate fundamental matrix.
     */
    public static void computeFundamentalMatrix(Transformation2D homography, 
            Point2D rightEpipole, FundamentalMatrix result) 
            throws InvalidFundamentalMatrixException {
        
        result.setFromHomography(homography, rightEpipole);
    }
    
    /**
     * Computes the residual between a fundamental matrix and a pair of matched
     * points.
     * @param fundamentalMatrix a fundamental matrix.
     * @param leftPoint left 2D point.
     * @param rightPoint right 2D point.
     * @return residual (distance of point to epipolar line).
     */
    protected double residual(FundamentalMatrix fundamentalMatrix,
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
    protected double totalResidual(FundamentalMatrix fundamentalMatrix) {
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
