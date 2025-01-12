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
import com.irurueta.geometry.PinholeCamera;

import java.io.Serializable;

/**
 * Contains data of estimated camera.
 */
public class EstimatedCamera implements Serializable {

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
     * ID of view for which camera has been estimated.
     */
    private int viewId;

    /**
     * Estimated camera.
     */
    private PinholeCamera camera;

    /**
     * Quality score of estimated camera. The larger the value, the better the
     * quality. This is used for robust estimators such as PROSAC or PROMedS.
     * This value is typically obtained during camera estimation.
     */
    private double qualityScore = DEFAULT_QUALITY_SCORE;

    /**
     * Covariance of estimated camera. This can be computed during camera
     * estimation.
     */
    private Matrix covariance;

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
     * Gets id of view for which camera has been estimated.
     *
     * @return id of view for which camera has been estimated.
     */
    public int getViewId() {
        return viewId;
    }

    /**
     * Sets id of view for which camera has been estimated.
     *
     * @param viewId id of view for which camera has been estimated.
     */
    public void setViewId(final int viewId) {
        this.viewId = viewId;
    }

    /**
     * Gets estimated camera.
     *
     * @return estimated camera.
     */
    public PinholeCamera getCamera() {
        return camera;
    }

    /**
     * Sets estimated camera.
     *
     * @param camera estimated camera.
     */
    public void setCamera(final PinholeCamera camera) {
        this.camera = camera;
    }

    /**
     * Gets quality score of estimated camera. The larger the value, the better
     * the quality. This is used for robust estimators such as PROSAC or
     * PROMedS.
     * This value is typically obtained from algorithms determining point
     * correspondences.
     *
     * @return quality score of estimated camera.
     */
    public double getQualityScore() {
        return qualityScore;
    }

    /**
     * Sets quality score of estimated camera. The larger the value, the better
     * the quality. This is used for robust estimators such as PROSAC or
     * PROMedS.
     * This value is typically obtained from algorithms determining point
     * correspondences.
     *
     * @param qualityScore quality score of estimated camera.
     */
    public void setQualityScore(final double qualityScore) {
        this.qualityScore = qualityScore;
    }

    /**
     * Gets covariance of estimated camera. This can be computed during camera
     * estimation.
     *
     * @return covariance of estimated camera.
     */
    public Matrix getCovariance() {
        return covariance;
    }

    /**
     * Sets covariance of estimated camera. This can be computed during camera
     * estimation.
     *
     * @param covariance covariance of estimated camera.
     */
    public void setCovariance(final Matrix covariance) {
        this.covariance = covariance;
    }
}
