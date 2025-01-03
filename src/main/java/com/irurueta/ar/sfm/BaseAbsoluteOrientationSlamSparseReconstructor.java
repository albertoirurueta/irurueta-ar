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

import com.irurueta.ar.slam.AbsoluteOrientationBaseSlamEstimator;
import com.irurueta.ar.slam.BaseCalibrationData;
import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Rotation3D;

import java.util.ArrayList;

/**
 * Base class in charge of estimating cameras and 3D reconstructed points from sparse
 * image point correspondences in multiple views and also in charge of estimating overall
 * scene scale and absolute orientation by means of SLAM (Simultaneous Location And Mapping)
 * using data obtained from sensors like accelerometers or gyroscopes.
 * NOTE: absolute orientation slam estimators are not very accurate during estimation of
 * the orientation state, for that reason we take into account the initial orientation.
 *
 * @param <D> type defining calibration data.
 * @param <C> type of configuration.
 * @param <R> type of re-constructor.
 * @param <L> type of listener.
 * @param <S> type of SLAM estimator.
 */
public abstract class BaseAbsoluteOrientationSlamSparseReconstructor<
        D extends BaseCalibrationData,
        C extends BaseSlamSparseReconstructorConfiguration<D, C>,
        R extends BaseSlamSparseReconstructor<D, C, R, L, S>,
        L extends BaseSlamSparseReconstructorListener<R>,
        S extends AbsoluteOrientationBaseSlamEstimator<D>> extends
        BaseSlamSparseReconstructor<D, C, R, L, S> {

    /**
     * First sample of orientation received.
     */
    private Rotation3D firstOrientation;

    /**
     * Inverse of first orientation.
     */
    private Rotation3D invFirstOrientation;

    /**
     * Constructor.
     *
     * @param configuration configuration for this re-constructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    protected BaseAbsoluteOrientationSlamSparseReconstructor(final C configuration, final L listener) {
        super(configuration, listener);
    }

    /**
     * Provides a new orientation sample to update SLAM estimator.
     * If re-constructor is not running, calling this method has no effect.
     *
     * @param timestamp   timestamp of accelerometer sample since epoch time and
     *                    expressed in nanoseconds.
     * @param orientation new orientation.
     */
    public void updateOrientationSample(final long timestamp, final Rotation3D orientation) {
        if (slamEstimator != null) {
            slamEstimator.updateOrientationSample(timestamp, orientation);
        }
        if (firstOrientation == null) {
            // make a copy of orientation
            firstOrientation = orientation.toQuaternion();
            invFirstOrientation = firstOrientation.inverseRotationAndReturnNew();
        }
    }

    /**
     * Updates scene scale and orientation using SLAM data.
     *
     * @param isInitialPairOfViews true if initial pair of views is being processed, false otherwise.
     * @return true if scale was successfully updated, false otherwise.
     */
    @SuppressWarnings("DuplicatedCode")
    protected boolean updateScaleAndOrientation(final boolean isInitialPairOfViews) {

        try {
            final var metricCamera1 = previousMetricEstimatedCamera.getCamera();
            final var metricCamera2 = currentMetricEstimatedCamera.getCamera();

            double slamPosX;
            double slamPosY;
            double slamPosZ;
            double scale;
            if (isInitialPairOfViews) {
                // obtain baseline (camera separation from slam estimator data
                slamPosX = slamEstimator.getStatePositionX();
                slamPosY = slamEstimator.getStatePositionY();
                slamPosZ = slamEstimator.getStatePositionZ();

                slamPosition.setInhomogeneousCoordinates(slamPosX, slamPosY, slamPosZ);

                metricCamera1.decompose(false, true);
                metricCamera2.decompose(false, true);

                final var center1 = metricCamera1.getCameraCenter();
                final var center2 = metricCamera2.getCameraCenter();

                final var baseline = center1.distanceTo(slamPosition);
                final var estimatedBaseline = center1.distanceTo(center2);

                scale = currentScale = baseline / estimatedBaseline;
            } else {
                scale = currentScale;
            }

            // R1' = R1*Rdiff
            // Rdiff = R1^T*R1'

            // where R1' is the desired orientation (obtained by sampling a
            // sensor)
            // and R1 is always the identity for the 1st camera.
            // Hence R1' = Rdiff

            // t1' is the desired translation which is zero for the 1st
            // camera.

            // We want: P1' = K*[R1' t1'] = K*[R1' 0]
            // And we have P1 = K[I 0]

            // We need a transformation T so that:
            // P1' = P1*T^-1 = K[I 0][R1' 0]
            //                       [0   1]

            // Hence: T^-1 = [R1' 0]
            //               [0   1]

            // or T = [R1'^T 0]
            //        [0     1]

            // because we are also applying a transformation of scale s,
            // the combination of both transformations is
            // T = [s*R1'^T 0]
            //     [0       1]

            final var scaleAndOrientationTransformation = new MetricTransformation3D(scale);
            scaleAndOrientationTransformation.setRotation(invFirstOrientation);

            // update scale of cameras
            final var euclideanCamera1 = scaleAndOrientationTransformation.transformAndReturnNew(metricCamera1);
            final var euclideanCamera2 = scaleAndOrientationTransformation.transformAndReturnNew(metricCamera2);

            euclideanCamera2.decompose(false, true);
            slamEstimator.correctWithPositionMeasure(euclideanCamera2.getCameraCenter(),
                    configuration.getCameraPositionCovariance());

            if (!isInitialPairOfViews) {
                slamPosX = slamEstimator.getStatePositionX();
                slamPosY = slamEstimator.getStatePositionY();
                slamPosZ = slamEstimator.getStatePositionZ();
                slamPosition.setInhomogeneousCoordinates(slamPosX, slamPosY, slamPosZ);

                // adjust scale of current camera
                final var euclideanCenter2 = euclideanCamera2.getCameraCenter();

                final var euclideanPosX = euclideanCenter2.getInhomX();
                final var euclideanPosY = euclideanCenter2.getInhomY();
                final var euclideanPosZ = euclideanCenter2.getInhomZ();

                final var scaleVariationX = euclideanPosX / slamPosX;
                final var scaleVariationY = euclideanPosY / slamPosY;
                final var scaleVariationZ = euclideanPosZ / slamPosZ;

                final var scaleVariation = (scaleVariationX + scaleVariationY + scaleVariationZ) / 3.0;
                scale *= scaleVariation;
                currentScale = scale;
                scaleAndOrientationTransformation.setScale(currentScale);

                // update camera
                scaleAndOrientationTransformation.transform(metricCamera2, euclideanCamera2);
            }
            final var sqrScale = scale * scale;

            previousEuclideanEstimatedCamera = new EstimatedCamera();
            previousEuclideanEstimatedCamera.setCamera(euclideanCamera1);
            previousEuclideanEstimatedCamera.setViewId(previousMetricEstimatedCamera.getViewId());
            previousEuclideanEstimatedCamera.setQualityScore(previousMetricEstimatedCamera.getQualityScore());
            if (previousMetricEstimatedCamera.getCovariance() != null) {
                previousEuclideanEstimatedCamera.setCovariance(previousMetricEstimatedCamera.getCovariance()
                        .multiplyByScalarAndReturnNew(sqrScale));
            }

            currentEuclideanEstimatedCamera = new EstimatedCamera();
            currentEuclideanEstimatedCamera.setCamera(euclideanCamera2);
            currentEuclideanEstimatedCamera.setViewId(currentMetricEstimatedCamera.getViewId());
            currentEuclideanEstimatedCamera.setQualityScore(currentMetricEstimatedCamera.getQualityScore());
            if (currentMetricEstimatedCamera.getCovariance() != null) {
                currentEuclideanEstimatedCamera.setCovariance(currentMetricEstimatedCamera.getCovariance()
                        .multiplyByScalarAndReturnNew(sqrScale));
            }

            // update scale of reconstructed points
            final var numPoints = activeMetricReconstructedPoints.size();
            final var metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (final var reconstructedPoint : activeMetricReconstructedPoints) {
                metricReconstructedPoints3D.add(reconstructedPoint.getPoint());
            }

            final var euclideanReconstructedPoints3D = scaleAndOrientationTransformation.transformPointsAndReturnNew(
                    metricReconstructedPoints3D);

            // set scaled points into result
            activeEuclideanReconstructedPoints = new ArrayList<>();
            ReconstructedPoint3D euclideanPoint;
            ReconstructedPoint3D metricPoint;
            for (var i = 0; i < numPoints; i++) {
                metricPoint = activeMetricReconstructedPoints.get(i);

                euclideanPoint = new ReconstructedPoint3D();
                euclideanPoint.setId(metricPoint.getId());
                euclideanPoint.setPoint(euclideanReconstructedPoints3D.get(i));
                euclideanPoint.setInlier(metricPoint.isInlier());
                euclideanPoint.setQualityScore(metricPoint.getQualityScore());
                if (metricPoint.getCovariance() != null) {
                    euclideanPoint.setCovariance(metricPoint.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
                }
                euclideanPoint.setColorData(metricPoint.getColorData());

                activeEuclideanReconstructedPoints.add(euclideanPoint);
            }

            return true;

        } catch (final Exception e) {
            failed = true;
            //noinspection unchecked
            listener.onFail((R) this);

            return false;
        }
    }
}
