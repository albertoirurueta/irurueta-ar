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
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.ar.calibration.RadialDistortion;
import com.irurueta.ar.calibration.RadialDistortionException;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.WeightSelection;
import com.irurueta.sorting.SortingException;

import java.util.Iterator;
import java.util.List;

/**
 * This class implements a radial distortion estimator using a weighted 
 * algorithm and correspondences.
 * Weights can be used so that correspondences assumed to have a better quality
 * are considered to be more relevant.
 */
public class WeightedRadialDistortionEstimator extends RadialDistortionEstimator{
    /**
     * Default number of points (i.e. correspondences) to be weighted and taken
     * into account.
     */
    public static final int DEFAULT_MAX_POINTS = 50;
    
    /**
     * Indicates if weights are sorted by default so that largest weighted
     * correspondences are used first.
     */
    public static final boolean DEFAULT_SORT_WEIGHTS = true;
    
    /**
     * Maximum number of points (i.e. correspondences) to be weighted and taken
     * into account.
     */
    private int mMaxPoints;
    
    /**
     * Indicates if weights are sorted by default so that largest weighted
     * correspondences are used first.
     */
    private boolean mSortWeights;
    
    /**
     * Array containing weights for all point correspondences.
     */
    private double[] mWeights;
    
    /**
     * Constructor.
     */
    public WeightedRadialDistortionEstimator() {
        super();
        mMaxPoints = DEFAULT_MAX_POINTS;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public WeightedRadialDistortionEstimator(
            RadialDistortionEstimatorListener listener) {
        super(listener);
        mMaxPoints = DEFAULT_MAX_POINTS;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
    }
    
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param weights array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size or their size is smaller than 
     * MIN_NUMBER_OF_POINT_CORRESPONDENCES.
     */
    public WeightedRadialDistortionEstimator(List<Point2D> distortedPoints, 
            List<Point2D> undistortedPoints, double[] weights) 
            throws IllegalArgumentException {
        super();
        internalSetPointsAndWeights(distortedPoints, undistortedPoints, weights);
        mMaxPoints = DEFAULT_MAX_POINTS;
        mSortWeights = DEFAULT_SORT_WEIGHTS;        
    }
   
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param weights array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size or their size is smaller than
     * MIN_NUMBER_OF_POINT_CORRESPONDENCES.
     */
    
    public WeightedRadialDistortionEstimator(List<Point2D> distortedPoints, 
            List<Point2D> undistortedPoints, double[] weights,
            RadialDistortionEstimatorListener listener) 
            throws IllegalArgumentException {
        super(listener);
        internalSetPointsAndWeights(distortedPoints, undistortedPoints, weights);
        mMaxPoints = DEFAULT_MAX_POINTS;
        mSortWeights = DEFAULT_SORT_WEIGHTS;        
    }

    /**
     * Constructor with distortion center.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     */
    public WeightedRadialDistortionEstimator(Point2D distortionCenter) {
        super(distortionCenter);
        mMaxPoints = DEFAULT_MAX_POINTS;
        mSortWeights = DEFAULT_SORT_WEIGHTS;        
    }
    
    /**
     * Constructor with listener and distortion center.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public WeightedRadialDistortionEstimator(Point2D distortionCenter,
            RadialDistortionEstimatorListener listener) {
        super(distortionCenter, listener);
        mMaxPoints = DEFAULT_MAX_POINTS;
        mSortWeights = DEFAULT_SORT_WEIGHTS;        
    }
    
    /**
     * Constructor with points and distortion center.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param weights array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size or their size is smaller than 
     * MIN_NUMBER_OF_POINT_CORRESPONDENCES.
     */
    public WeightedRadialDistortionEstimator(List<Point2D> distortedPoints, 
            List<Point2D> undistortedPoints, double[] weights, Point2D distortionCenter) 
            throws IllegalArgumentException {
        super(distortionCenter);
        internalSetPointsAndWeights(distortedPoints, undistortedPoints, weights);
        mMaxPoints = DEFAULT_MAX_POINTS;
        mSortWeights = DEFAULT_SORT_WEIGHTS;        
    }
   
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param weights array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size or their size is smaller than
     * MIN_NUMBER_OF_POINT_CORRESPONDENCES.
     */ 
    public WeightedRadialDistortionEstimator(List<Point2D> distortedPoints, 
            List<Point2D> undistortedPoints, double[] weights, 
            Point2D distortionCenter, 
            RadialDistortionEstimatorListener listener) 
            throws IllegalArgumentException {
        super(distortionCenter, listener);
        internalSetPointsAndWeights(distortedPoints, undistortedPoints, weights);
        mMaxPoints = DEFAULT_MAX_POINTS;
        mSortWeights = DEFAULT_SORT_WEIGHTS;        
    }
    
    /**
     * Sets lists of corresponding distorted and undistorted points.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param weights array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if any of the lists or arrays are null 
     * or if provided lists of points don't have the same size and enough points 
     * or if the length of the weights array is not equal to the number of point 
     * correspondences.
     */
    public void setPointsAndWeights(List<Point2D> distortedPoints, 
            List<Point2D> undistortedPoints, double[] weights) 
            throws LockedException, IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        internalSetPointsAndWeights(distortedPoints, undistortedPoints, weights);
    }       
    
    /**
     * Indicates if lists of corresponding distorted and undistorted points are
     * valid.
     * Lists are considered valid if they have the same number of points and
     * both have more than the required minimum of correspondences (which is 2).
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param weights array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @return true if lists of points are valid, false otherwise.
     */
    public boolean areValidPointsAndWeights(List<Point2D> distortedPoints,
            List<Point2D> undistortedPoints, double[] weights){
        if (distortedPoints == null || undistortedPoints == null ||
                weights == null) {
            return false;
        }
        
        return distortedPoints.size() == undistortedPoints.size() &&
                distortedPoints.size() == weights.length &&
                distortedPoints.size() >= getMinNumberOfMatchedPoints();
    }    
    
    /**
     * Returns array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @return array containing weights for each correspondence.
     */
    public double[] getWeights() {
        return mWeights;
    }
    
    /**
     * Returns boolean indicating whether weights have been provided and are
     * available for retrieval.
     * @return true if weights are available, false otherwise.
     */
    public boolean areWeightsAvailable() {
        return mWeights != null;
    }    
    
    /**
     * Returns maximum number of points (i.e. correspondences) to be weighted 
     * and taken into account.
     * @return maximum number of points to be weighted.
     */
    public int getMaxPoints() {
        return mMaxPoints;
    }
    
    /**
     * Sets maximum number of points (i.e. correspondences) to be weighted and
     * taken into account.
     * @param maxPoints maximum number of points to be weighted.
     * @throws IllegalArgumentException if provided value is less than the 
     * minimum allowed number of point correspondences.
     * @throws LockedException if this instance is locked.
     */
    public void setMaxPoints(int maxPoints) throws IllegalArgumentException,
            LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxPoints < getMinNumberOfMatchedPoints()) {
            throw new IllegalArgumentException();
        }
        
        mMaxPoints = maxPoints;
    }
    
    /**
     * Indicates if weights are sorted by so that largest weighted
     * correspondences are used first.
     * @return true if weights are sorted, false otherwise.
     */
    public boolean isSortWeightsEnabled() {
        return mSortWeights;
    }
    
    /**
     * Specifies whether weights are sorted by so that largest weighted
     * correspondences are used first.
     * @param sortWeights true if weights are sorted, false otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setSortWeightsEnabled(boolean sortWeights) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mSortWeights = sortWeights;
    }
    
    /**
     * Indicates if this estimator is ready to start the estimation.
     * Estimator will be ready once both lists and weights are available.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return arePointsAvailable() && areWeightsAvailable();
    }    
    
    /**
     * Estimates a radial distortion.
     * @return estimated radial distortion.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if input has not yet been provided.
     * @throws RadialDistortionEstimatorException if an error occurs during
     * estimation, usually because input data is not valid.
     */    
    @Override
    public RadialDistortion estimate() throws LockedException, 
            NotReadyException, RadialDistortionEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        try {
            mLocked = true;
            if (mListener != null) {
                mListener.onEstimateStart(this);
            }
            
            WeightSelection selection = WeightSelection.selectWeights(mWeights, 
                    mSortWeights, mMaxPoints);
            boolean[] selected = selection.getSelected();
            
            int nPoints = mDistortedPoints.size();
            
            Matrix A = new Matrix(2 * nPoints, mNumKParams);
            double[] b = new double[2 * nPoints];
            
            Iterator<Point2D> iteratorDistorted = mDistortedPoints.iterator();
            Iterator<Point2D> iteratorUndistorted = 
                    mUndistortedPoints.iterator();
            
            Point2D distorted, undistorted;
            int index = 0, counter = 0;
            
            //undistorted normalized homogeneous coordinates
            double uNormHomX, uNormHomY, uNormHomW; 
            //undistorted normalized inhomogeneous coordinates
            double uNormInhomX, uNormInhomY;
            //undistorted denormalized homogeneous coordinates
            double uDenormHomX, uDenormHomY, uDenormHomW; 
            //undistorted denormalized inhomogeneous coordinates
            double uDenormInhomX, uDenormInhomY;
            //distorted inhomogeneous coordinates
            double dInhomX, dInhomY;
            double rowNormX, rowNormY;
            
            //radial distortion center
            double centerX = 0.0, centerY = 0.0;
            if (mDistortionCenter != null) {
                centerX = mDistortionCenter.getInhomX();
                centerY = mDistortionCenter.getInhomY();
            }
            
            //radial distance of undistorted normalized (calibration independent) 
            //coordinates
            double r2; 
            double a, value, weight;
            
            
            while (iteratorDistorted.hasNext() && iteratorUndistorted.hasNext()) {
                distorted = iteratorDistorted.next();
                undistorted = iteratorUndistorted.next();                
                
                if (selected[index]) {
                    undistorted.normalize();                    
                    
                    weight = mWeights[index];
                    
                    uDenormHomX = undistorted.getHomX();
                    uDenormHomY = undistorted.getHomY();
                    uDenormHomW = undistorted.getHomW();
                
                    uDenormInhomX = uDenormHomX / uDenormHomW;
                    uDenormInhomY = uDenormHomY / uDenormHomW;
                
                    //multiply intrinsic parameters by undistorted point
                    uNormHomX = mKinv.getElementAt(0, 0) * uDenormHomX +
                            mKinv.getElementAt(0, 1) * uDenormHomY +
                            mKinv.getElementAt(0, 2) * uDenormHomW;
                    uNormHomY = mKinv.getElementAt(1, 0) * uDenormHomX +
                            mKinv.getElementAt(1, 1) * uDenormHomY +
                            mKinv.getElementAt(1, 2) * uDenormHomW;
                    uNormHomW = mKinv.getElementAt(2, 0) * uDenormHomX +
                            mKinv.getElementAt(2, 1) * uDenormHomY +
                            mKinv.getElementAt(2, 2) * uDenormHomW;
                
                    uNormInhomX = uNormHomX / uNormHomW;
                    uNormInhomY = uNormHomY / uNormHomW;
                
                    r2 = uNormInhomX * uNormInhomX + uNormInhomY * uNormInhomY;
                
                    dInhomX = distorted.getInhomX();
                    dInhomY = distorted.getInhomY();
                    
                    a = 1.0;
                    rowNormX = rowNormY = 0.0;
                    for (int i = 0; i < mNumKParams; i++) {
                        a *= r2;
                        
                        //x and y coordinates generate linear dependent equations, for 
                        //that reason we need more than one point
                    
                        //x coordinates
                        value = (uDenormInhomX - centerX) * a * weight;
                        A.setElementAt(counter * 2, i, value);
                    
                        rowNormX += Math.pow(value, 2.0);
                    
                        //y coordinates
                        value = (uDenormInhomY - centerY) * a * weight;
                        A.setElementAt(counter * 2 + 1, i, value);
                    
                        rowNormY += Math.pow(value, 2.0);                        
                    }
                    
                    //x coordinates
                    value = (dInhomX - uDenormInhomX) * weight;
                    b[counter * 2] = value;
                
                    rowNormX += Math.pow(value, 2.0);
                
                    //y coordinates
                    value = (dInhomY - uDenormInhomY) * weight;
                    b[counter * 2 + 1] = value;
                
                    rowNormY += Math.pow(value, 2.0);
                    
                    //normalize rows to increase accuracy
                    for (int i = 0; i < mNumKParams; i++) {
                        A.setElementAt(counter * 2, i, 
                                A.getElementAt(counter * 2, i) / rowNormX);
                        A.setElementAt(counter * 2 + 1, i, 
                                A.getElementAt(counter * 2 + 1, i) / rowNormY);
                    }
                
                    b[counter * 2] /= rowNormX;
                    b[counter * 2 +1] /= rowNormY;
                                
                    counter++;                                                            
                }

                index++;
            }
            
            double[] params = Utils.solve(A, b);
            
            RadialDistortion distortion = 
                    new RadialDistortion(params, mDistortionCenter,
                    mHorizontalFocalLength, mVerticalFocalLength, mSkew);
            
            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
            
            return distortion;
        } catch (AlgebraException | SortingException | RadialDistortionException e) {
            throw new RadialDistortionEstimatorException(e);
        } finally {
            mLocked = false;
        }
    }    
    
    /**
     * Returns type of radial distortion estimator.
     * @return type of radial distortion estimator.
     */    
    @Override
    public RadialDistortionEstimatorType getType() {
        return RadialDistortionEstimatorType.
                WEIGHTED_RADIAL_DISTORTION_ESTIMATOR;
    }  
            
    /**
     * Internal method to set list of corresponding points (it does not check
     * if estimator is locked).
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param weights array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @throws IllegalArgumentException if any of the lists or arrays are null 
     * or if provided lists of points don't have the same size and enough points 
     * or if the length of the weights array is not equal to the number of point 
     * correspondences.
     */
    private void internalSetPointsAndWeights(List<Point2D> distortedPoints, 
            List<Point2D> undistortedPoints, double[] weights) 
            throws IllegalArgumentException {
        
        if (distortedPoints == null || undistortedPoints == null ||
                weights == null) {
            throw new IllegalArgumentException();
        }
        
        if (!areValidPointsAndWeights(distortedPoints, undistortedPoints, weights)) {
            throw new IllegalArgumentException();
        }
        
        mDistortedPoints = distortedPoints;
        mUndistortedPoints = undistortedPoints;        
        mWeights = weights;
    }        
}
