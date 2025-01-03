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

package com.irurueta.ar.sfm;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point3D;

import java.io.Serializable;

/**
 * Contains data of a reconstructed 3D point.
 */
public class ReconstructedPoint3D implements Serializable {

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
     * Coordinates of reconstructed 3D point.
     */
    private Point3D point;

    /**
     * Indicates whether reconstructed point is an inlier or not.
     */
    private boolean inlier;

    /**
     * Quality score of sampled point. The larger the value, the better the
     * quality. This is used for robust estimators such as PROSAC or PROMedS.
     * This value is typically obtained from algorithms determining point
     * correspondences.
     */
    private double qualityScore = DEFAULT_QUALITY_SCORE;

    /**
     * Covariance of reconstructed point. This can be computed during point
     * triangularization.
     */
    private Matrix covariance;

    /**
     * Color data of reconstructed point (i.e. RGB or YUV values), if available.
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
     * Gets coordinates of reconstructed 3D point.
     *
     * @return coordinates of reconstructed 3D point.
     */
    public Point3D getPoint() {
        return point;
    }

    /**
     * Sets coordinates of reconstructed 3D point.
     *
     * @param point coordinates of reconstructed 3D point.
     */
    public void setPoint(final Point3D point) {
        this.point = point;
    }

    /**
     * Indicates whether reconstructed point is an inlier or not.
     *
     * @return true if reconstructed point is an inlier, false otherwise.
     */
    public boolean isInlier() {
        return inlier;
    }

    /**
     * Specifies whether reconstructed point is an inlier or not.
     *
     * @param inlier true if reconstructed point is an inlier, false otherwise.
     */
    public void setInlier(final boolean inlier) {
        this.inlier = inlier;
    }

    /**
     * Gets quality score of sampled point. The larger the value, the better the
     * quality. This is used for robust estimators such as PROSAC or PROMedS.
     * This value is typically obtained from algorithms determining point
     * correspondences.
     *
     * @return quality score of sampled point.
     */
    public double getQualityScore() {
        return qualityScore;
    }

    /**
     * Sets quality score of sampled point. The larger the value, the better the
     * quality. This is used for robust estimators such as PROSAC or PROMedS.
     * This value is typically obtained from algorithms determining point
     * correspondences.
     *
     * @param qualityScore quality score of sampled point.
     */
    public void setQualityScore(final double qualityScore) {
        this.qualityScore = qualityScore;
    }

    /**
     * Gets covariance of reconstructed point. This is obtained from the
     * algorithms determining points of interest or point correspondences.
     * This might be null if covariance cannot be determined.
     *
     * @return covariance of reconstructed point.
     */
    public Matrix getCovariance() {
        return covariance;
    }

    /**
     * Sets covariance of reconstructed point. This is obtained from the
     * algorithms determining points of interest or point correspondences.
     *
     * @param covariance covariance of reconstructed point.
     */
    public void setCovariance(final Matrix covariance) {
        this.covariance = covariance;
    }

    /**
     * Gets color data of reconstructed point (i.e. RGB or YUV values), if
     * available.
     *
     * @return color data of reconstructed point or null.
     */
    public PointColorData getColorData() {
        return colorData;
    }

    /**
     * Sets color data of reconstructed point (i.e. RGB or YUV values), if
     * available.
     *
     * @param colorData color data of reconstructed point or null.
     */
    public void setColorData(final PointColorData colorData) {
        this.colorData = colorData;
    }
}
