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
     * ID to identify this instance. This is useful in case that this data is
     * stored in some sort of database and must be set externally.
     */
    private String id;

    /**
     * ID of view where 2D point has been sampled.
     */
    private int viewId;

    /**
     * 2D sampled point coordinates.
     */
    private Point2D point;

    /**
     * 3D reconstructed point.
     */
    private ReconstructedPoint3D reconstructedPoint;

    /**
     * Quality score of sampled point. The larger the value, the
     * better the quality. This is used for robust estimators such
     * as PROSAC or PROMedS.
     * This value is typically obtained from algorithms determining quality of
     * points of interest.
     */
    private double qualityScore = DEFAULT_QUALITY_SCORE;

    /**
     * Covariance of sampled points. This is obtained from the algorithms
     * determining points of interest or point correspondences.
     * If covariance cannot be determined, a typical value might be to
     * consider 1 pixel accuracy.
     * This might be null if covariance cannot be determined.
     */
    private Matrix covariance;

    /**
     * Color data of sampled point (i.e. RGB or YUV values), if available.
     */
    private PointColorData colorData;

    /**
     * Gets id to identify this instance. This is useful in case that this data
     * is stored in some sort of database and must be set externally.
     *
     * @return id to identify this instance.
     */
    public String getId() {
        return id;
    }

    /**
     * Sets id to identify this instance. This is useful in case that this data
     * is stored in some sort of database and must be set externally.
     *
     * @param id id to identify this instance.
     */
    public void setId(final String id) {
        this.id = id;
    }

    /**
     * Gets id of view where 2D point has been sampled.
     *
     * @return id of view where 2D point has been sampled.
     */
    public int getViewId() {
        return viewId;
    }

    /**
     * Sets id of view where 2D point has been sampled.
     *
     * @param viewId id of view where 2D point has been sampled.
     */
    public void setViewId(final int viewId) {
        this.viewId = viewId;
    }

    /**
     * Gets 2D sampled point coordinates.
     *
     * @return 2D sampled point coordinates.
     */
    public Point2D getPoint() {
        return point;
    }

    /**
     * Sets 2D sampled point coordinates.
     *
     * @param point 2D sampled point coordinates.
     */
    public void setPoint(final Point2D point) {
        this.point = point;
    }

    /**
     * Gets 3D reconstructed point.
     *
     * @return 3D reconstructed point.
     */
    public ReconstructedPoint3D getReconstructedPoint() {
        return reconstructedPoint;
    }

    /**
     * Sets 3D reconstructed point.
     *
     * @param reconstructedPoint 3D reconstructed point.
     */
    public void setReconstructedPoint(final ReconstructedPoint3D reconstructedPoint) {
        this.reconstructedPoint = reconstructedPoint;
    }

    /**
     * Gets quality score of sampled point. The larger the value, the
     * better the quality. This is used for robust estimators such as
     * PROSAC or PROMEdS.
     * This value is typically obtained from algorithms determining quality of
     * points of interest.
     *
     * @return quality score of sampled point.
     */
    public double getQualityScore() {
        return qualityScore;
    }

    /**
     * Sets quality score of sampled point. The larger the value, the better
     * the quality. This is used for robust estimators such as PROSAC or
     * PROMedS.
     * This value is typically obtained from algorithms determining quality of
     * points of interest.
     *
     * @param qualityScore quality score of sampled point.
     */
    public void setQualityScore(final double qualityScore) {
        this.qualityScore = qualityScore;
    }

    /**
     * Gets covariance of sampled points. This is obtained from the algorithms
     * determining points of interest or point correspondences.
     * If covariance cannot be determined, a typical value might be to
     * consider 1 pixel accuracy.
     * This might be null if covariance cannot be determined.
     *
     * @return covariance of sampled points or null.
     */
    public Matrix getCovariance() {
        return covariance;
    }

    /**
     * Sets covariance of sampled points. This is obtained from the algorithms
     * determining points of interest or point correspondences.
     * If covariance cannot be determined, a typical value might be to
     * consider 1 pixel accuracy.
     * This might be null if covariance cannot be determined.
     *
     * @param covariance covariance of sampled points.
     */
    public void setCovariance(final Matrix covariance) {
        this.covariance = covariance;
    }

    /**
     * Gets color data of sampled point (i.e. RGB or YUV values), if available.
     *
     * @return color data of sampled point or null.
     */
    public PointColorData getColorData() {
        return colorData;
    }

    /**
     * Sets color data of sampled point (i.e. RGB or YUV values), if available.
     *
     * @param colorData color data of sampled point or null.
     */
    public void setColorData(final PointColorData colorData) {
        this.colorData = colorData;
    }
}
