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

package com.irurueta.ar.calibration.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.ar.calibration.DualAbsoluteQuadric;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.robust.WeightSelection;
import com.irurueta.sorting.SortingException;

import java.util.List;

/**
 * Implementation of a Dual Absolute Quadric estimator using a weighted solution
 * for provided pinhole cameras.
 * This implementation assumes that:
 * - cameras are arbitrary (usually the initial camera is the identity and must
 * be discarded) as it creates a numerical degeneracy.
 * - all provided cameras have the same intrinsic parameters
 * - it is assumed that skewness is zero, the principal point is at the center
 * of the image plane (zero), and both horizontal and vertical focal planes are
 * equal.
 */
public class WeightedDualAbsoluteQuadricEstimator extends 
        DualAbsoluteQuadricEstimator {
    
    /**
     * Default number of cameras (i.e. correspondences) to be weighted and taken
     * into account.
     */
    public static final int DEFAULT_MAX_CAMERAS = 50;
    
    /**
     * Indicates if weights are sorted by default so that largest weighted 
     * cameras are used first.
     */
    public static final boolean DEFAULT_SORT_WEIGHTS = true;
    
    /**
     * Maximum number of cameras (i.e. correspondences) to be weighted and taken
     * into account.
     */
    private int mMaxCameras;
    
    /**
     * Indicates if weights are sorted by default so that largest weighted
     * cameras are used first.
     */
    private boolean mSortWeights;
    
    /**
     * Array containing weights for all cameras.
     */
    private double[] mWeights;
    
    /**
     * Constructor.
     */
    public WeightedDualAbsoluteQuadricEstimator() {
        super();
        mMaxCameras = DEFAULT_MAX_CAMERAS;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
        mWeights = null;
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public WeightedDualAbsoluteQuadricEstimator(
            DualAbsoluteQuadricEstimatorListener listener) {
        super(listener);
        mMaxCameras = DEFAULT_MAX_CAMERAS;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
        mWeights = null;        
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric
     * (DAQ).
     * @throws IllegalArgumentException if list of cameras is null.
     */
    public WeightedDualAbsoluteQuadricEstimator(List<PinholeCamera> cameras)
            throws IllegalArgumentException {
        super(cameras);
        mMaxCameras = DEFAULT_MAX_CAMERAS;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
        mWeights = null;        
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric
     * (DAQ).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if list of cameras is null.
     */
    public WeightedDualAbsoluteQuadricEstimator(List<PinholeCamera> cameras,
            DualAbsoluteQuadricEstimatorListener listener)
            throws IllegalArgumentException {
        super(cameras, listener);
        mMaxCameras = DEFAULT_MAX_CAMERAS;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
        mWeights = null;        
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric
     * (DAQ).
     * @param weights array containing a weight amount for each corresponding 
     * camera. The larget the value of a weight, the most significant the 
     * corresponding camera data will be.
     * @throws IllegalArgumentException if provided lists of cameras and weights
     * don't have the same size or enough cameras.
     */
    public WeightedDualAbsoluteQuadricEstimator(List<PinholeCamera> cameras,
            double[] weights) throws IllegalArgumentException {
        super(cameras);    
        try {
            setWeights(weights);
        } catch(LockedException ignore) { /* never thrown */ }
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric
     * (DAQ).
     * @param weights array containing a weight amount for each corresponding 
     * camera. The largest the value of a weight, the most significant the 
     * corresponding camera data will be.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if provided lists of cameras and weights
     * don't have the same size or enough cameras.
     */
    public WeightedDualAbsoluteQuadricEstimator(List<PinholeCamera> cameras,
            double[] weights, DualAbsoluteQuadricEstimatorListener listener)
            throws IllegalArgumentException {
        super(cameras, listener);
        try {
            setWeights(weights);
        } catch(LockedException ignore) { /* never thrown */ }
    }
    
    /**
     * Indicates whether provided cameras and weights are valid or not.
     * Cameras and weights must have the same length to be valid and their 
     * length must be greater than 1.
     * @param cameras list of cameras to check.
     * @param weights array of weights to check.
     * @return true if cameras and weights are valid, false otherwise.
     */
    public static boolean areValidCamerasAndWeights(List<PinholeCamera> cameras, 
            double[] weights) {
        return cameras != null && weights != null && 
                cameras.size() == weights.length;
    }
    
    /**
     * Returns array containing a weight amount for each corresponding camera.
     * The largest the value of a weight, the more significant the corresponding
     * camera data will be.
     * @return weights for each corresponding camera.
     */
    public double[] getWeights() {
        return mWeights;
    }
    
    /**
     * Sets array of camera weight for each corresponding camera.
     * The largest the value of a weight, the more significant the corresponding
     * camera data will be.
     * @param weights weights for each corresponding camera.
     * @throws IllegalArgumentException if provided lists of cameras and weights
     * don't have the same size or enough cameras.
     * @throws LockedException if estimator is locked.
     */
    public final void setWeights(double[] weights) 
            throws IllegalArgumentException, LockedException {
        if (!areValidCamerasAndWeights(mCameras, weights)) {
            throw new IllegalArgumentException(
                    "cameras and weights must have the same length");
        }
        if (isLocked()) {
            throw new LockedException();
        }
        
        mWeights = weights;
    }
    
    /**
     * Sets list of cameras and corresponding weights.
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric
     * (DAQ).
     * @param weights array containing a weight amount for each corresponding 
     * camera. The largest the value of a weight, the most significant the 
     * corresponding camera data will be.
     * @throws IllegalArgumentException if provided lists of cameras and weights
     * don't have the same size or enough cameras.
     * @throws LockedException if estimator is locked.
     */
    public void setCamerasAndWeights(List<PinholeCamera> cameras, 
            double[] weights) throws IllegalArgumentException, LockedException {
        if (!areValidCamerasAndWeights(cameras, weights)) {
            throw new IllegalArgumentException(
                    "cameras and weights must have the same length");
        }
        if (isLocked()) {
            throw new LockedException();
        }
        
        mCameras = cameras;
        mWeights = weights;
    }
    
    /**
     * Indicates whether weights have already been provided or not.
     * @return true if weights have been provided, false otherwise.
     */
    public boolean areWeightsAvailable() {
        return mWeights != null;
    }
        
    /**
     * Gets the maximum number of cameras (i.e. correspondences) to be weighted
     * and taken into account.
     * @return maximum number of cameras.
     */
    public int getMaxCameras() {
        return mMaxCameras;
    }
    
    /**
     * Sets the maximum number of cameras (i.e. correspondences) to be weighted
     * and taken into account.
     * @param maxCameras maximum number of cameras.
     * @throws IllegalArgumentException if provided value is less than 2.
     * @throws LockedException if estimator is locked.
     */
    public void setMaxCameras(int maxCameras) throws IllegalArgumentException,
            LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mMaxCameras = maxCameras;
    }
    
    /**
     * Indicates if weights are sorted by default so that largest weighted
     * cameras are used first.
     * @return true if weights are sorted by default, false otherwise.
     */
    public boolean isSortWeightsEnabled() {
        return mSortWeights;
    }
    
    /**
     * Specifies whether weights are sorted by default so that largest weighted
     * cameras are used first.
     * @param sortWeights true if weights are sorted by default, false 
     * otherwise.
     * @throws LockedException if estimator is locked.
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
     * @return true if estimator is ready, false otherwise.
     */    
    @Override
    public boolean isReady() {
        return super.isReady() && areWeightsAvailable() && 
                mMaxCameras >= getMinNumberOfRequiredCameras();
    }

    /**
     * Estimates the Dual Absolute Quadric using provided cameras.
     * @param result instance where estimated Dual Absolute Quadric (DAQ) will
     * be stored.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if no valid input data has already been 
     * provided.
     * @throws DualAbsoluteQuadricEstimatorException if an error occurs during
     * estimation, usually because input data is not valid or numerically
     * unstable.
     */
    @Override
    public void estimate(DualAbsoluteQuadric result) 
            throws LockedException, NotReadyException, 
            DualAbsoluteQuadricEstimatorException {
        
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
                        
            if(mPrincipalPointAtOrigin) {
                if(mZeroSkewness) {
                    if(mFocalDistanceAspectRatioKnown) {
                        estimateZeroSkewnessPrincipalPointAtOriginAndKnownFocalDistanceAspectRatio(
                                result);
                    } else {
                        estimateZeroSkewnessAndPrincipalPointAtOrigin(result);
                    }
                } else {
                    estimatePrincipalPointAtOrigin(result);
                }
            }
            
            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }                                   
        } finally {
            mLocked = false;
        }
    }

    /**
     * Returns type of Dual Absolute Quadric estimator.
     * @return type of DAQ estimator.
     */
    @Override
    public DualAbsoluteQuadricEstimatorType getType() {
        return DualAbsoluteQuadricEstimatorType.
                WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR;
    }  
    
    /**
     * Indicates whether current constraints are enough to start the estimation.
     * In order to obtain a linear solution for the DAQ estimation, we need at
     * least the principal point at origin constraint.
     * @return true if constraints are valid, false otherwise.
     */
    @Override
    public boolean areValidConstraints() {
        return super.areValidConstraints() && isZeroSkewness() && 
                isFocalDistanceAspectRatioKnown();
    }    
    
    /**
     * Estimates Dual Absolute Quadric (DAQ) assuming that skewness is zero,
     * principal point is located at origin of coordinates and that aspect ratio
     * of focal distances is known.
     * @param result instance where resulting estimated Dual Absolute Quadric 
     * will be stored.
     * @throws DualAbsoluteQuadricEstimatorException if an error occurs during
     * estimation, usually because repeated cameras are provided, or cameras
     * corresponding to critical motion sequences such as pure paraller 
     * translations are provided, where no additional data is really provided.
     */    
    private void estimateZeroSkewnessPrincipalPointAtOriginAndKnownFocalDistanceAspectRatio(
            DualAbsoluteQuadric result)
            throws DualAbsoluteQuadricEstimatorException {
        
        try {
            int nCams = Math.min(mCameras.size(), mMaxCameras);
            
            WeightSelection selection = WeightSelection.selectWeights(mWeights, 
                    mSortWeights, nCams);
            boolean[] selected = selection.getSelected();

            Matrix a = new Matrix(DualAbsoluteQuadric.N_PARAMS, 
                    DualAbsoluteQuadric.N_PARAMS);
            Matrix row = new Matrix(4, DualAbsoluteQuadric.N_PARAMS);
            Matrix transRow = new Matrix(DualAbsoluteQuadric.N_PARAMS, 4);
            Matrix tmp = new Matrix(DualAbsoluteQuadric.N_PARAMS, 
                    DualAbsoluteQuadric.N_PARAMS);
            
            Matrix cameraMatrix;
            double p11, p12, p13, p14;
            double p21, p22, p23, p24;
            double p31, p32, p33, p34;
            int eqCounter;
            int cameraCounter = 0;  
            double weight;
            double previousNorm = 1.0;
            for (PinholeCamera camera : mCameras) {
                
                if (selected[cameraCounter]) {
                    eqCounter = 0;
                    
                    //normalize cameras to increase accuracy
                    camera.normalize();
                
                    cameraMatrix = camera.getInternalMatrix();
                                
                    p11 = cameraMatrix.getElementAt(0, 0);
                    p21 = cameraMatrix.getElementAt(1, 0);
                    p31 = cameraMatrix.getElementAt(2, 0);
                
                    p12 = cameraMatrix.getElementAt(0, 1);
                    p22 = cameraMatrix.getElementAt(1, 1);
                    p32 = cameraMatrix.getElementAt(2, 1);
                
                    p13 = cameraMatrix.getElementAt(0, 2);
                    p23 = cameraMatrix.getElementAt(1, 2);
                    p33 = cameraMatrix.getElementAt(2, 2);
                
                    p14 = cameraMatrix.getElementAt(0, 3);
                    p24 = cameraMatrix.getElementAt(1, 3);
                    p34 = cameraMatrix.getElementAt(2, 3);
                    
                    weight = mWeights[cameraCounter];
                
                    //1st row
                    fill2ndRowAnd1stRowEquation(p11, p21,
                            p12, p22,
                            p13, p23,
                            p14, p24, row, eqCounter);
                    applyWeight(row, eqCounter, weight);
                    eqCounter++;
                
                    //2nd row
                    fill3rdRowAnd1stRowEquation(p11, p31,
                            p12, p32,
                            p13, p33,
                            p14, p34, row, eqCounter);
                    applyWeight(row, eqCounter, weight);
                    eqCounter++;
                
                    //3rd row
                    fill3rdRowAnd2ndRowEquation(p21, p31,
                            p22, p32,
                            p23, p33,
                            p24, p34, row, eqCounter);
                    applyWeight(row, eqCounter, weight);
                    eqCounter++;
                
                    //4th row
                    fill1stRowEqualTo2ndRowEquation(p11, p21,
                            p12, p22,
                            p13, p23,
                            p14, p24, row, eqCounter);
                    applyWeight(row, eqCounter, weight);
                    
                    
                    row.transpose(transRow); //transRow = row'
                    transRow.multiply(row, tmp);
                    
                    tmp.multiplyByScalar(1.0 / previousNorm);
                    
                    //a += 1.0 / previousNorm * tmp
                    a.add(tmp);
                    //normalize
                    previousNorm = Utils.normF(a);
                    a.multiplyByScalar(1.0 / previousNorm);                                      
                }
                
                cameraCounter++;
            }
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            enforceRank3IfNeeded(decomposer, result);
            
        } catch (AlgebraException | SortingException | NumericalException e) {
            throw new DualAbsoluteQuadricEstimatorException(e);
        }
    }

    /**
     * Estimates Dual Absolute Quadric (DAQ) assuming that skewness is zero,
     * and principal point is located at origin of coordinates.
     * @param result instance where resulting estimated Dual Absolute Quadrics 
     * will be stored.
     * @throws DualAbsoluteQuadricEstimatorException if an error occurs during
     * estimation, usually because repeated cameras are provided, or cameras
     * corresponding to critical motion sequences such as pure paraller 
     * translations are provided, where no additional data is really provided.
     */    
    private void estimateZeroSkewnessAndPrincipalPointAtOrigin(
            DualAbsoluteQuadric result)
            throws DualAbsoluteQuadricEstimatorException {
        
        try {
            int nCams = Math.min(mCameras.size(), mMaxCameras);
            
            WeightSelection selection = WeightSelection.selectWeights(mWeights, 
                    mSortWeights, nCams);
            boolean[] selected = selection.getSelected();

            Matrix a = new Matrix(DualAbsoluteQuadric.N_PARAMS, 
                    DualAbsoluteQuadric.N_PARAMS);
            Matrix row = new Matrix(3, DualAbsoluteQuadric.N_PARAMS);
            Matrix transRow = new Matrix(DualAbsoluteQuadric.N_PARAMS, 3);
            Matrix tmp = new Matrix(DualAbsoluteQuadric.N_PARAMS, 
                    DualAbsoluteQuadric.N_PARAMS);
            
            Matrix cameraMatrix;
            double p11, p12, p13, p14;
            double p21, p22, p23, p24;
            double p31, p32, p33, p34;
            int eqCounter;
            int cameraCounter = 0;  
            double weight;
            double previousNorm = 1.0;
            for (PinholeCamera camera : mCameras) {
                
                if (selected[cameraCounter]) {
                    eqCounter = 0;
                    
                    //normalize cameras to increase accuracy
                    camera.normalize();
                
                    cameraMatrix = camera.getInternalMatrix();
                                
                    p11 = cameraMatrix.getElementAt(0, 0);
                    p21 = cameraMatrix.getElementAt(1, 0);
                    p31 = cameraMatrix.getElementAt(2, 0);
                
                    p12 = cameraMatrix.getElementAt(0, 1);
                    p22 = cameraMatrix.getElementAt(1, 1);
                    p32 = cameraMatrix.getElementAt(2, 1);
                
                    p13 = cameraMatrix.getElementAt(0, 2);
                    p23 = cameraMatrix.getElementAt(1, 2);
                    p33 = cameraMatrix.getElementAt(2, 2);
                
                    p14 = cameraMatrix.getElementAt(0, 3);
                    p24 = cameraMatrix.getElementAt(1, 3);
                    p34 = cameraMatrix.getElementAt(2, 3);
                    
                    weight = mWeights[cameraCounter];
                
                    //1st row
                    fill2ndRowAnd1stRowEquation(p11, p21,
                            p12, p22,
                            p13, p23,
                            p14, p24, a, eqCounter);
                    applyWeight(row, eqCounter, weight);
                    eqCounter++;
                
                    //2nd row
                    fill3rdRowAnd1stRowEquation(p11, p31,
                            p12, p32,
                            p13, p33,
                            p14, p34, a, eqCounter);
                    applyWeight(row, eqCounter, weight);
                    eqCounter++;
                
                    //3rd row
                    fill3rdRowAnd2ndRowEquation(p21, p31,
                            p22, p32,
                            p23, p33,
                            p24, p34, a, eqCounter);
                    applyWeight(row, eqCounter, weight);
                    
                    row.transpose(transRow); //transRow = row'
                    transRow.multiply(row, tmp);
                    
                    tmp.multiplyByScalar(1.0 / previousNorm);
                    
                    //a += 1.0 / previousNorm * tmp
                    a.add(tmp);
                    //normalize
                    previousNorm = Utils.normF(a);
                    a.multiplyByScalar(1.0 / previousNorm);                                      
                }
                
                cameraCounter++;
            }
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            enforceRank3IfNeeded(decomposer, result);
            
        } catch (AlgebraException | SortingException | NumericalException e) {
            throw new DualAbsoluteQuadricEstimatorException(e);
        }
    }

    /**
     * Estimates Dual Absolute Quadric (DAQ) assumint that principal point is
     * zero.
     * @param result instance where resulting estimated Dual Absolute Quadrics 
     * will be stored.
     * @throws DualAbsoluteQuadricEstimatorException if an error occurs during
     * estimation, usually because repeated cameras are provided, or cameras
     * corresponding to critical motion sequences such as pure paraller 
     * translations are provided, where no additional data is really provided.
     */    
    private void estimatePrincipalPointAtOrigin(
            DualAbsoluteQuadric result) 
            throws DualAbsoluteQuadricEstimatorException {
        
        try {
            int nCams = Math.min(mCameras.size(), mMaxCameras);
            
            WeightSelection selection = WeightSelection.selectWeights(mWeights, 
                    mSortWeights, nCams);
            boolean[] selected = selection.getSelected();

            Matrix a = new Matrix(DualAbsoluteQuadric.N_PARAMS, 
                    DualAbsoluteQuadric.N_PARAMS);
            Matrix row = new Matrix(2, DualAbsoluteQuadric.N_PARAMS);
            Matrix transRow = new Matrix(DualAbsoluteQuadric.N_PARAMS, 2);
            Matrix tmp = new Matrix(DualAbsoluteQuadric.N_PARAMS, 
                    DualAbsoluteQuadric.N_PARAMS);
            
            Matrix cameraMatrix;
            double p11, p12, p13, p14;
            double p21, p22, p23, p24;
            double p31, p32, p33, p34;
            int eqCounter;
            int cameraCounter = 0;  
            double weight;
            double previousNorm = 1.0;
            for (PinholeCamera camera : mCameras) {
                
                if (selected[cameraCounter]) {
                    eqCounter = 0;
                    
                    //normalize cameras to increase accuracy
                    camera.normalize();
                
                    cameraMatrix = camera.getInternalMatrix();
                                
                    p11 = cameraMatrix.getElementAt(0, 0);
                    p21 = cameraMatrix.getElementAt(1, 0);
                    p31 = cameraMatrix.getElementAt(2, 0);
                
                    p12 = cameraMatrix.getElementAt(0, 1);
                    p22 = cameraMatrix.getElementAt(1, 1);
                    p32 = cameraMatrix.getElementAt(2, 1);
                
                    p13 = cameraMatrix.getElementAt(0, 2);
                    p23 = cameraMatrix.getElementAt(1, 2);
                    p33 = cameraMatrix.getElementAt(2, 2);
                
                    p14 = cameraMatrix.getElementAt(0, 3);
                    p24 = cameraMatrix.getElementAt(1, 3);
                    p34 = cameraMatrix.getElementAt(2, 3);
                    
                    weight = mWeights[cameraCounter];
                
                    //1st row
                    fill3rdRowAnd1stRowEquation(p11, p31,
                            p12, p32,
                            p13, p33,
                            p14, p34, a, eqCounter);
                    applyWeight(row, eqCounter, weight);
                    eqCounter++;
                
                    //2nd row
                    fill3rdRowAnd2ndRowEquation(p21, p31,
                            p22, p32,
                            p23, p33,
                            p24, p34, a, eqCounter);
                    applyWeight(row, eqCounter, weight);
                                    
                    row.transpose(transRow); //transRow = row'
                    transRow.multiply(row, tmp);
                    
                    tmp.multiplyByScalar(1.0 / previousNorm);
                    
                    //a += 1.0 / previousNorm * tmp
                    a.add(tmp);
                    //normalize
                    previousNorm = Utils.normF(a);
                    a.multiplyByScalar(1.0 / previousNorm);                                      
                }
                
                cameraCounter++;
            }
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            enforceRank3IfNeeded(decomposer, result);
            
        } catch (AlgebraException | SortingException | NumericalException e) {
            throw new DualAbsoluteQuadricEstimatorException(e);
        }
    }
    
    /**
     * Apply provided weight to matrix at provided row.
     * @param a matrix to apply weight to.
     * @param row row within matrix to apply weight.
     * @param weight weight to be applied.
     */
    private void applyWeight(Matrix a, int row, double weight) {
        int cols = a.getColumns();
        for(int i = 0; i < cols; i++) {
            a.setElementAt(row, i, a.getElementAt(row, i) * weight);
        }
    }
}
