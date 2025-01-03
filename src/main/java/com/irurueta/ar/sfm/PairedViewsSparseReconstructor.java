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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.Point3D;

import java.util.ArrayList;

/**
 * Class in charge of estimating pairs of cameras and 3D reconstruction points from
 * sparse image point correspondences.
 */
public class PairedViewsSparseReconstructor extends BasePairedViewsSparseReconstructor<
        PairedViewsSparseReconstructorConfiguration, PairedViewsSparseReconstructor,
        PairedViewsSparseReconstructorListener> {

    /**
     * Constructor.
     *
     * @param configuration configuration for this re-constructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not provided.
     */
    public PairedViewsSparseReconstructor(
            final PairedViewsSparseReconstructorConfiguration configuration,
            final PairedViewsSparseReconstructorListener listener) {
        super(configuration, listener);
    }

    /**
     * Constructor with default configuration.
     *
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not provided.
     */
    public PairedViewsSparseReconstructor(final PairedViewsSparseReconstructorListener listener) {
        this(new PairedViewsSparseReconstructorConfiguration(), listener);
    }

    /**
     * Indicates whether implementations of a re-constructor uses absolute orientation or
     * not.
     *
     * @return true if absolute orientation is used, false, otherwise.
     */
    @Override
    protected boolean hasAbsoluteOrientation() {
        return false;
    }

    /**
     * Transforms metric cameras on current pair of views so that they are referred to
     * last kept location and rotation.
     *
     * @param isInitialPairOfViews   true if initial pair of views is being processed, false otherwise.
     * @param hasAbsoluteOrientation true if absolute orientation is required, false otherwise.
     * @return true if cameras were successfully transformed.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    protected boolean transformPairOfCamerasAndPoints(
            final boolean isInitialPairOfViews, final boolean hasAbsoluteOrientation) {
        final var previousMetricCamera = previousMetricEstimatedCamera.getCamera();
        final var currentMetricCamera = currentMetricEstimatedCamera.getCamera();
        if (previousMetricCamera == null || currentMetricCamera == null) {
            return false;
        }

        currentScale = listener.onBaselineRequested(this, previousViewId, currentViewId,
                previousMetricEstimatedCamera, currentMetricEstimatedCamera);
        final var sqrScale = currentScale * currentScale;

        final var scaleTransformation = new MetricTransformation3D(currentScale);

        if (isInitialPairOfViews) {
            // the first pair of views does not require setting translation and rotation
            referenceEuclideanTransformation = scaleTransformation;
        } else {
            // additional pairs also need to translate and rotate
            final var invRot = lastEuclideanCameraRotation.inverseRotationAndReturnNew();
            final var translation = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            translation[0] = lastEuclideanCameraCenter.getInhomX();
            translation[1] = lastEuclideanCameraCenter.getInhomY();
            translation[2] = lastEuclideanCameraCenter.getInhomZ();
            referenceEuclideanTransformation = scaleTransformation.combineAndReturnNew(
                    new MetricTransformation3D(invRot, translation, 1.0));
            referenceEuclideanTransformation.setRotation(invRot);
            referenceEuclideanTransformation.setTranslation(lastEuclideanCameraCenter);
        }

        try {
            // transform cameras
            final var previousEuclideanCamera = referenceEuclideanTransformation.transformAndReturnNew(
                    previousMetricCamera);
            final var currentEuclideanCamera = referenceEuclideanTransformation.transformAndReturnNew(
                    currentMetricCamera);

            previousEuclideanEstimatedCamera = new EstimatedCamera();
            previousEuclideanEstimatedCamera.setCamera(previousEuclideanCamera);
            previousEuclideanEstimatedCamera.setViewId(previousMetricEstimatedCamera.getViewId());
            previousEuclideanEstimatedCamera.setQualityScore(previousMetricEstimatedCamera.getQualityScore());
            if (previousMetricEstimatedCamera.getCovariance() != null) {
                previousEuclideanEstimatedCamera.setCovariance(
                        previousMetricEstimatedCamera.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
            }

            currentEuclideanEstimatedCamera = new EstimatedCamera();
            currentEuclideanEstimatedCamera.setCamera(currentEuclideanCamera);
            currentEuclideanEstimatedCamera.setViewId(currentMetricEstimatedCamera.getViewId());
            currentEuclideanEstimatedCamera.setQualityScore(currentMetricEstimatedCamera.getQualityScore());
            if (currentMetricEstimatedCamera.getCovariance() != null) {
                currentEuclideanEstimatedCamera.setCovariance(
                        currentMetricEstimatedCamera.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
            }

            // transform points
            euclideanReconstructedPoints = new ArrayList<>();
            for (final var metricReconstructedPoint : metricReconstructedPoints) {
                final var metricPoint = metricReconstructedPoint.getPoint();
                final var euclideanPoint = referenceEuclideanTransformation.transformAndReturnNew(metricPoint);
                final var euclideanReconstructedPoint = new ReconstructedPoint3D();
                euclideanReconstructedPoint.setPoint(euclideanPoint);
                euclideanReconstructedPoint.setInlier(metricReconstructedPoint.isInlier());
                euclideanReconstructedPoint.setId(metricReconstructedPoint.getId());
                euclideanReconstructedPoint.setColorData(metricReconstructedPoint.getColorData());
                if (metricReconstructedPoint.getCovariance() != null) {
                    euclideanReconstructedPoint.setCovariance(
                            metricReconstructedPoint.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
                }
                euclideanReconstructedPoint.setQualityScore(metricReconstructedPoint.getQualityScore());
                euclideanReconstructedPoints.add(euclideanReconstructedPoint);
            }

        } catch (final AlgebraException e) {
            return false;
        }

        return super.transformPairOfCamerasAndPoints(isInitialPairOfViews, hasAbsoluteOrientation);
    }
}
