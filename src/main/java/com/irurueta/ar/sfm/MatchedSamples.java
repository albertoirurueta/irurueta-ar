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

import java.io.Serializable;
import java.util.BitSet;

/**
 * Contains data relating matched 2D points and their reconstructions.
 */
public class MatchedSamples implements Serializable {
    
    /**
     * Default quality score value.
     */
    public static final double DEFAULT_QUALITY_SCORE = 1.0;

    /**
     * 2D matched samples on different views.
     * Each of these points correspond to projections of the same 3D point into
     * different views.
     */
    private Sample2D[] mSamples;
    
    /**
     * Cameras associated to the views of each of the matched points.
     */
    private EstimatedCamera[] mCameras;
    
    /**
     * Id's of views where matched points belong to.
     */
    private int[] mViewIds;
    
    /**
     * 3D reconstructed point. Initially, might not be available
     */
    private ReconstructedPoint3D mReconstructedPoint;
    
    /**
     * Quality score of a match.
     */
    private double mQualityScore = DEFAULT_QUALITY_SCORE;
    
    /**
     * Indicates whether match between a pair of views has been considered an 
     * inlier or not.
     * Position 0 of this bitset corresponds to viewIds in positions 0 and 1,
     * position 1 of bitset corresponts to viewIds in positions 1 and 2, and so 
     * on.
     */
    private BitSet mInliers;
    
    /**
     * Constructor.
     */
    public MatchedSamples() { }
    
    /**
     * Gets 2D matched samples on different views containing matched points.
     * Each of these points correspond to projections of the same 3D point into
     * different views.
     * @return 2D matched samples on different views.
     */
    public Sample2D[] getSamples() {
        return mSamples;
    }
    
    /**
     * Sets 2D matched matches on different views containing matched points.
     * Each of these points correspond to projections of the same 3D point into
     * different views.
     * @param samples 2D matched samples on different views.
     */
    public void setSamples(Sample2D[] samples) {
        mSamples = samples;
    }
    
    /**
     * Gets cameras associated to the views of each of the matched points.
     * @return cameras associated to the views of each of the matched points.
     */
    public EstimatedCamera[] getCameras() {
        return mCameras;
    }
    
    /**
     * Sets cameras associated to the views of each of the matched points.
     * @param cameras cameras associated to the views of each of the matched 
     * points.
     */
    public void setCameras(EstimatedCamera[] cameras) {
        mCameras = cameras;
    }

    /**
     * Gets id's of views where matched points belong to.
     * @return id's of view where matched points belong to.
     */
    public int[] getViewIds() {
        return mViewIds;
    }
    
    /**
     * Sets id's of views where matched points belong to.
     * @param viewIds id's of views where matched points belong to.
     */
    public void setViewIds(int[] viewIds) {
        mViewIds = viewIds;
    }

    /**
     * Gets 3D reconstructed point.
     * @return 3D reconstructed point.
     */
    public ReconstructedPoint3D getReconstructedPoint() {
        return mReconstructedPoint;
    }
    
    /**
     * Sets 3D reconstructed point.
     * @param reconstructedPoint 3D reconstructed point.
     */
    public void setReconstructedPoint(ReconstructedPoint3D reconstructedPoint) {
        mReconstructedPoint = reconstructedPoint;
        if (mSamples != null) {
            for (Sample2D sample : mSamples) {
                sample.setReconstructedPoint(reconstructedPoint);
            }
        }
    }    
    
    /**
     * Gets quality score of match. The larger the value, the better the 
     * quality. This is used for robust estimators such as PROSAC or PROMedS.
     * This value is typically obtained from algorithms determining scores for
     * matches.
     * @return quality score of match.
     */
    public double getQualityScore() {
        return mQualityScore;
    }
    
    /**
     * Sets quality score of match. The larger the value, the better the
     * quality. This is used for robust estimators such as PROSAC or PROMedS.
     * This value is typically obtained from algorithms determining scores for
     * matches.
     * @param qualityScore quality score of match.
     */
    public void setQualityScore(double qualityScore) {
        mQualityScore = qualityScore;
    }
    
    /**
     * Indicates whether match between a pair of views has been considered an
     * inlier or not.
     * Position 0 of this bitset corresponds to viewIds in positions 0 and 1,
     * position 1 of bitset corresponds to viewIds in positions 1 and 2, and so 
     * on.
     * @return indicates whether match between a pair of views has been 
     * considered an inlier or not.
     */
    public BitSet getInliers() {
        return mInliers;
    }
    
    /**
     * Specifies whether match between a pair of views has been considered an
     * inlier or not.
     * Position 0 of this bitset corresponds to viewIds in positions 0 and 1,
     * position 1 of bitset corresponds to viewIds in positions 1 and 2, and so
     * on.
     * @param inliers set indicating whether a match between a pair of views has
     * been considered an inlier or not.
     */
    public void setInliers(BitSet inliers) {
        mInliers = inliers;
    }    
}
