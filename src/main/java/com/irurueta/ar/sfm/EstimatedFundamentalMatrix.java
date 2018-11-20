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
package com.irurueta.ar.sfm;

import com.irurueta.algebra.Matrix;
import com.irurueta.ar.epipolar.FundamentalMatrix;

import java.io.Serializable;
import java.util.BitSet;
import java.util.List;

/**
 * Contains data of estimated fundamental matrix.
 */
public class EstimatedFundamentalMatrix implements Serializable {
    
    /**
     * Default quality score value.
     */
    public static final double DEFAULT_QUALITY_SCORE = 1.0;
    
    /**
     * Id to identify this instance. This is useful in case that this data is
     * stored in some sort of database and must be set externally.
     */
    private String mId;
    
    /**
     * Estimted fundamental matrix.
     */
    private FundamentalMatrix mFundamentalMatrix;
    
    /**
     * Quality score of estimated fundamental matrix. The larger the value,
     * the better the quality.
     */
    private double mQualityScore = DEFAULT_QUALITY_SCORE;
    
    /**
     * Covariance of estimated fundamental matrix. This can be computed during 
     * estimation.
     */
    private Matrix mCovariance;
    
    /**
     * Id of first view related by fundamental matrix.
     */
    private int mViewId1;
    
    /**
     * Id of second view related by fundamental matrix.
     */
    private int mViewId2;
    
    /**
     * Indicates which samples used for fundamental matrix estimation where 
     * considered inliers.
     */
    private BitSet mInliers;
    
    /**
     * Left samples used for fundamental matrix estimation.
     */
    private List<Sample2D> mLeftSamples;
    
    /**
     * Right samples used for fundamental matrix estimation.
     */
    private List<Sample2D> mRightSamples;
        
    /**
     * Constructor.
     */
    public EstimatedFundamentalMatrix() { }
    
    /**
     * Gets id to identify this instance. This is useful in case that this data 
     * is stored in some sort of database and must be set externally.
     * @return id to identify this instance.
     */
    public String getId() {
        return mId;
    }
    
    /**
     * Sets id to identify this instance. This is useful in case that this data 
     * is stored in some sort of database and must be set externally.
     * @param id id to identify this instance.
     */
    public void setId(String id) {
        mId = id;
    }
        
    /**
     * Gets estimated fundamental matrix.
     * @return estimated fundamental matrix.
     */
    public FundamentalMatrix getFundamentalMatrix() {
        return mFundamentalMatrix;
    }
    
    /**
     * Sets estimated fundamental matrix.
     * @param fundamentalMatrix estimated fundamental matrix.
     */
    public void setFundamentalMatrix(FundamentalMatrix fundamentalMatrix) {
        mFundamentalMatrix = fundamentalMatrix;
    }

    /**
     * Gets quality score of estimated fundamental matrix. The larger the value,
     * the better the quality.
     * @return quality score of estimated fundamental matrix.
     */
    public double getQualityScore() {
        return mQualityScore;
    }
    
    /**
     * Sets quality score of estimated fundamental matrix. The larger the value,
     * the better the quality.
     * @param qualityScore quality score of estimated fundamental matrix.
     */
    public void setQualityScore(double qualityScore) {
        mQualityScore = qualityScore;
    }

    /**
     * Gets covariance of estimated fundamental matrix. This can be computed 
     * during estimation.
     * @return covariance of estimated fundamental matrix.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }
    
    /**
     * Sets covariance of estimated fundamental matrix. This can be computed
     * during estimation.
     * @param covariance covariance of estimated fundamental matrix.
     */
    public void setCovariance(Matrix covariance) {
        mCovariance = covariance;
    }

    /**
     * Gets id of first view related by fundamental matrix.
     * @return id of first view related by fundamental matrix.
     */
    public int getViewId1() {
        return mViewId1;
    }
    
    /**
     * Sets id of first view related by fundamental matrix.
     * @param viewId1 id of first view related by fundamental matrix.
     */
    public void setViewId1(int viewId1) {
        mViewId1 = viewId1;
    }

    /**
     * Gets id of second view related by fundamental matrix.
     * @return id of second view related by fundamental matrix.
     */
    public int getViewId2() {
        return mViewId2;
    }
    
    /**
     * Sets id of second view related by fundamental matrix.
     * @param viewId2 id of second view related by fundamental matrix.
     */
    public void setViewId2(int viewId2) {
        mViewId2 = viewId2;
    }    
    
    /**
     * Indicates which samples used for fundamental matrix estimation where
     * considered inliers.
     * @return which samples used for fundamental matrix estimation where
     * considered inliers.
     */
    public BitSet getInliers() {
        return mInliers;
    }
    
    /**
     * Specifies which samples used for fundamental matrix estimation where
     * considered inliers.
     * @param inliers which samples used for fundamental matrix estimation where
     * considered inliers.
     */
    public void setInliers(BitSet inliers) {
        mInliers = inliers;
    }    
    
    /**
     * Gets left samples used for fundamental matrix estimation.
     * @return left samples used for fundamental matrix estimation.
     */
    public List<Sample2D> getLeftSamples() {
        return mLeftSamples;
    }
    
    /**
     * Sets left samples used for fundamental matrix estimation.
     * @param leftSamples left samples used for fundamental matrix estimation.
     */
    public void setLeftSamples(List<Sample2D> leftSamples) {
        mLeftSamples = leftSamples;
    }

    /**
     * Gets right samples used for fundamental matrix estimation.
     * @return right samples used for fundamental matrix estimation.
     */
    public List<Sample2D> getRightSamples() {
        return mRightSamples;
    }
    
    /**
     * Sets right samples used for fundamental matrix estimation.
     * @param rightSamples right samples used for fundamental matrix estimation.
     */
    public void setRightSamples(List<Sample2D> rightSamples) {
        mRightSamples = rightSamples;
    }    
}
