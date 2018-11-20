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

import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

/**
 * Compares two fundamental matrices to determine how similar they are.
 * This is an abstract class, subclasses of this class will implement different 
 * methods to compare fundamental matrices.
 * This class will typically be used for testing or quality assessment purposes.
 */
@SuppressWarnings("WeakerAccess")
public abstract class FundamentalMatrixComparator {
    
    /**
     * Fundamental matrix to be considered as ground truth to compare against.
     */
    protected FundamentalMatrix mGroundTruthFundamentalMatrix;
        
    /**
     * Other fundamental matrix being compared.
     */
    protected FundamentalMatrix mOtherFundamentalMatrix;
    
    /**
     * Listener to handle events generated by this class.
     */
    protected FundamentalMatrixComparatorListener mListener;
    
    /**
     * Indicates whether this instance is busy doing computations.
     */
    protected boolean mLocked;
    
    /**
     * Constructor.
     */
    public FundamentalMatrixComparator() { }
    
    /**
     * Constructor.
     * @param groundTruthFundamentalMatrix fundamental matrix to be considered
     * as ground truth to compare against.
     * @param otherFundamentalMatrix other fundamental matrix being compared.
     */
    public FundamentalMatrixComparator(
            FundamentalMatrix groundTruthFundamentalMatrix,
            FundamentalMatrix otherFundamentalMatrix) {
        mGroundTruthFundamentalMatrix = groundTruthFundamentalMatrix;
        mOtherFundamentalMatrix = otherFundamentalMatrix;
    }

    /**
     * Constructor.
     * @param listener listener to handle events generated by this class.
     */
    public FundamentalMatrixComparator(
            FundamentalMatrixComparatorListener listener) {
        mListener = listener;
    }
    
    /**
     * Constructor.
     * @param groundTruthFundamentalMatrix fundamental matrix to be considered
     * as ground truth to compare against.
     * @param otherFundamentalMatrix other fundamental matrix being compared.
     * @param listener listener to handle events generated by this class.
     */
    public FundamentalMatrixComparator(
            FundamentalMatrix groundTruthFundamentalMatrix,
            FundamentalMatrix otherFundamentalMatrix,
            FundamentalMatrixComparatorListener listener) {
        this(groundTruthFundamentalMatrix, otherFundamentalMatrix);
        mListener = listener;
    }
    
    /**
     * Obtains fundamental matrix to be considered as ground truth to compare 
     * against.
     * @return fundamental matrix to be considered as ground truth.
     */
    public FundamentalMatrix getGroundTruthFundamentalMatrix() {
        return mGroundTruthFundamentalMatrix;
    }
    
    /**
     * Sets fundamental matrix to be considered as ground truth to compare 
     * against.
     * @param groundTruthFundamentalMatrix fundamental matrix to be considered
     * as ground truth.
     * @throws LockedException if this instance is locked.
     */
    public void setGroundTruthFundamentalMatrix(
            FundamentalMatrix groundTruthFundamentalMatrix) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mGroundTruthFundamentalMatrix = groundTruthFundamentalMatrix;
    }
    
    /**
     * Obtains other fundamental matrix being compared.
     * @return other fundamental matrix being compared.
     */
    public FundamentalMatrix getOtherFundamentalMatrix() {
        return mOtherFundamentalMatrix;
    }
    
    /**
     * Sets other fundamental matrix being compared.
     * @param otherFundamentalMatrix other fundamental matrix being compared.
     * @throws LockedException if this instance is locked.
     */
    public void setOtherFundamentalMatrix(
            FundamentalMatrix otherFundamentalMatrix) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mOtherFundamentalMatrix = otherFundamentalMatrix;
    }
    
    /**
     * Returns listener to handle events generated by instances of this class.
     * @return listener to handle events generated by instances of this class.
     */
    public FundamentalMatrixComparatorListener getListener() {
        return mListener;
    }
    
    /**
     * Sets listener to handle events generated by instances of this class.
     * @param listener listener to handle events generated by instances of this
     * class.
     * @throws LockedException if this instance is locked.
     */
    public void setListener(FundamentalMatrixComparatorListener listener)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mListener = listener;
    }
    
    /**
     * Returns boolean indicating whether an instance of this class is busy 
     * doing computations.
     * @return true if this instance is locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }
    
    /**
     * Indicates whether this comparator is ready to start the comparison of
     * two fundamental matrices. This is true when both ground truth and
     * the other fundamental matrix has been provided and both have their
     * internal matrices defined.
     * @return true if this comparator is ready, false otherwise.
     */
    public boolean isReady() {
        return mGroundTruthFundamentalMatrix != null && 
                mGroundTruthFundamentalMatrix.isInternalMatrixAvailable() &&
                mOtherFundamentalMatrix != null &&
                mOtherFundamentalMatrix.isInternalMatrixAvailable();
    }
    
    /**
     * Compares two fundamental matrices and returns the comparison value.
     * Comparison value will depend on the method implemented to compare both
     * fundamental matrices.
     * @return comparison value. Typically the smaller the absolute value the
     * more similar the fundamental matrices are.
     * @throws NotReadyException if this comparator is not  yet ready to start
     * the comparison.
     * @throws LockedException if this instance is locked.
     * @throws FundamentalMatrixComparatorException if comparison fails due to
     * some other reason.
     */
    public abstract double compare() throws NotReadyException, LockedException,
            FundamentalMatrixComparatorException;
    
    /**
     * Returns type of comparator.
     * @return type of comparator.
     */
    public abstract FundamentalMatrixComparatorType getType();    
}
