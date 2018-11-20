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
import com.irurueta.geometry.Point2D;

import java.io.Serializable;

/**
 * Contains data of a 2D point sample on a given view.
 */
public class Sample2D implements Serializable {
    
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
     * Id of view where 2D point has been sampled.
     */
    private int mViewId;
    
    /**
     * 2D sampled point coordinates.
     */
    private Point2D mPoint;

    /**
     * 3D reconstructed point.
     */
    private ReconstructedPoint3D mReconstructedPoint;
    
    /**
     * Quality score of sampled point. The larger the value, the
     * better the quality. This is used for robust estimators such
     * as PROSAC or PROMedS.
     * This value is typically obtained from algorithms determining quality of
     * points of interest.
     */
    private double mQualityScore = DEFAULT_QUALITY_SCORE;
        
    /**
     * Covariance of sampled points. This is obtained from the algorithms
     * determining points of interest or point correspondences.
     * If covariance cannot be determined, a typical value might be to
     * consider 1 pixel accuracy.
     * This might be null if covariance cannot be determined.
     */
    private Matrix mCovariance;
    
    /**
     * Color data of sampled point (i.e. RGB or YUV values), if available.
     */
    private PointColorData mColorData;
    
    /**
     * Constructor.
     */
    public Sample2D() { }
    
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
     * Gets id of view where 2D point has been sampled.
     * @return id of view where 2D point has been sampled.
     */
    public int getViewId() {
        return mViewId;
    }
    
    /**
     * Sets id of view where 2D point has been sampled.
     * @param viewId id of view where 2D point has been sampled.
     */
    public void setViewId(int viewId) {
        mViewId = viewId;
    }
    
    /**
     * Gets 2D sampled point coordinates.
     * @return 2D sampled point coordinates.
     */
    public Point2D getPoint() {
        return mPoint;
    }
    
    /**
     * Sets 2D sampled point coordinates.
     * @param point 2D sampled point coordinates.
     */
    public void setPoint(Point2D point) {
        mPoint = point;
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
    }

    /**
     * Gets quality score of sampled point. The larger the value, the
     * better the quality. This is used for robust estimators such as
     * PROSAC or PROMEdS.
     * This value is typically obtained from algorithms determining quality of 
     * points of interest.
     * @return quality score of sampled point.
     */
    public double getQualityScore() {
        return mQualityScore;
    }
    
    /**
     * Sets quality score of sampled point. The larger the value, the better
     * the quality. This is used for robust estimators such as PROSAC or 
     * PROMedS.
     * This value is typically obtained from algorithms determining quality of 
     * points of interest.
     * @param qualityScore quality score of sampled point.
     */
    public void setQualityScore(double qualityScore) {
        mQualityScore = qualityScore;
    }

    /**
     * Gets covariance of sampled points. This is obtained from the algorithms
     * determining points of interest or point correspondences.
     * If covariance cannot be determined, a typical value might be to
     * consider 1 pixel accuracy.
     * This might be null if covariance cannot be determined.
     * @return covariance of sampled points or null.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }
    
    /**
     * Sets covariance of sampled points. This is obtained from the algorithms
     * determining points of interest or point correspondences.
     * If covariance cannot be determined, a typical value might be to
     * consider 1 pixel accuracy.
     * This might be null if covariance cannot be determined.
     * @param covariance covariance of sampled points.
     */
    public void setCovariance(Matrix covariance) {
        mCovariance = covariance;
    }

    /**
     * Gets color data of sampled point (i.e. RGB or YUV values), if available.
     * @return color data of sampled point or null.
     */
    public PointColorData getColorData() {
        return mColorData;
    }    
    
    /**
     * Sets color data of sampled point (i.e. RGB or YUV values), if available.
     * @param colorData color data of sampled point or null.
     */
    public void setColorData(PointColorData colorData) {
        mColorData = colorData;
    }
}
