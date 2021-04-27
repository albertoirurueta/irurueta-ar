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
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Rotation3D;

import java.util.ArrayList;
import java.util.List;

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
 * @param <R> type of reconstructor.
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
    private Rotation3D mFirstOrientation;

    /**
     * Inverse of first orientation.
     */
    private Rotation3D mInvFirstOrientation;

    /**
     * Constructor.
     *
     * @param configuration configuration for this reconstructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    protected BaseAbsoluteOrientationSlamSparseReconstructor(
            final C configuration, final L listener) {
        super(configuration, listener);
    }

    /**
     * Provides a new orientation sample to update SLAM estimator.
     * If reconstructor is not running, calling this method has no effect.
     *
     * @param timestamp   timestamp of accelerometer sample since epoch time and
     *                    expressed in nanoseconds.
     * @param orientation new orientation.
     */
    public void updateOrientationSample(final long timestamp,
                                        final Rotation3D orientation) {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateOrientationSample(timestamp, orientation);
        }
        if (mFirstOrientation == null) {
            // make a copy of orientation
            mFirstOrientation = orientation.toQuaternion();
            mInvFirstOrientation = mFirstOrientation.inverseRotationAndReturnNew();
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
            final PinholeCamera metricCamera1 = mPreviousMetricEstimatedCamera.getCamera();
            final PinholeCamera metricCamera2 = mCurrentMetricEstimatedCamera.getCamera();

            double slamPosX;
            double slamPosY;
            double slamPosZ;
            double scale;
            if (isInitialPairOfViews) {
                // obtain baseline (camera separation from slam estimator data
                slamPosX = mSlamEstimator.getStatePositionX();
                slamPosY = mSlamEstimator.getStatePositionY();
                slamPosZ = mSlamEstimator.getStatePositionZ();

                mSlamPosition.setInhomogeneousCoordinates(slamPosX, slamPosY, slamPosZ);

                metricCamera1.decompose(false, true);
                metricCamera2.decompose(false, true);

                final Point3D center1 = metricCamera1.getCameraCenter();
                final Point3D center2 = metricCamera2.getCameraCenter();

                final double baseline = center1.distanceTo(mSlamPosition);
                final double estimatedBaseline = center1.distanceTo(center2);

                scale = mCurrentScale = baseline / estimatedBaseline;
            } else {
                scale = mCurrentScale;
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

            final MetricTransformation3D scaleAndOrientationTransformation =
                    new MetricTransformation3D(scale);
            scaleAndOrientationTransformation.setRotation(mInvFirstOrientation);

            // update scale of cameras
            final PinholeCamera euclideanCamera1 = scaleAndOrientationTransformation.transformAndReturnNew(
                    metricCamera1);
            final PinholeCamera euclideanCamera2 = scaleAndOrientationTransformation.transformAndReturnNew(
                    metricCamera2);

            euclideanCamera2.decompose(false, true);
            mSlamEstimator.correctWithPositionMeasure(euclideanCamera2.getCameraCenter(),
                    mConfiguration.getCameraPositionCovariance());

            if (!isInitialPairOfViews) {
                slamPosX = mSlamEstimator.getStatePositionX();
                slamPosY = mSlamEstimator.getStatePositionY();
                slamPosZ = mSlamEstimator.getStatePositionZ();
                mSlamPosition.setInhomogeneousCoordinates(slamPosX, slamPosY, slamPosZ);

                // adjust scale of current camera
                final Point3D euclideanCenter2 = euclideanCamera2.getCameraCenter();

                final double euclideanPosX = euclideanCenter2.getInhomX();
                final double euclideanPosY = euclideanCenter2.getInhomY();
                final double euclideanPosZ = euclideanCenter2.getInhomZ();

                final double scaleVariationX = euclideanPosX / slamPosX;
                final double scaleVariationY = euclideanPosY / slamPosY;
                final double scaleVariationZ = euclideanPosZ / slamPosZ;

                final double scaleVariation = (scaleVariationX + scaleVariationY + scaleVariationZ) / 3.0;
                scale *= scaleVariation;
                mCurrentScale = scale;
                scaleAndOrientationTransformation.setScale(mCurrentScale);

                // update camera
                scaleAndOrientationTransformation.transform(metricCamera2, euclideanCamera2);
            }
            final double sqrScale = scale * scale;

            mPreviousEuclideanEstimatedCamera = new EstimatedCamera();
            mPreviousEuclideanEstimatedCamera.setCamera(euclideanCamera1);
            mPreviousEuclideanEstimatedCamera.setViewId(mPreviousMetricEstimatedCamera.getViewId());
            mPreviousEuclideanEstimatedCamera.setQualityScore(mPreviousMetricEstimatedCamera.getQualityScore());
            if (mPreviousMetricEstimatedCamera.getCovariance() != null) {
                mPreviousEuclideanEstimatedCamera.setCovariance(
                        mPreviousMetricEstimatedCamera.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
            }

            mCurrentEuclideanEstimatedCamera = new EstimatedCamera();
            mCurrentEuclideanEstimatedCamera.setCamera(euclideanCamera2);
            mCurrentEuclideanEstimatedCamera.setViewId(mCurrentMetricEstimatedCamera.getViewId());
            mCurrentEuclideanEstimatedCamera.setQualityScore(mCurrentMetricEstimatedCamera.getQualityScore());
            if (mCurrentMetricEstimatedCamera.getCovariance() != null) {
                mCurrentEuclideanEstimatedCamera.setCovariance(
                        mCurrentMetricEstimatedCamera.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
            }

            // update scale of reconstructed points
            final int numPoints = mActiveMetricReconstructedPoints.size();
            final List<Point3D> metricReconstructedPoints3D = new ArrayList<>();
            for (final ReconstructedPoint3D reconstructedPoint : mActiveMetricReconstructedPoints) {
                metricReconstructedPoints3D.add(reconstructedPoint.getPoint());
            }

            final List<Point3D> euclideanReconstructedPoints3D =
                    scaleAndOrientationTransformation.transformPointsAndReturnNew(
                            metricReconstructedPoints3D);

            // set scaled points into result
            mActiveEuclideanReconstructedPoints = new ArrayList<>();
            ReconstructedPoint3D euclideanPoint;
            ReconstructedPoint3D metricPoint;
            for (int i = 0; i < numPoints; i++) {
                metricPoint = mActiveMetricReconstructedPoints.get(i);

                euclideanPoint = new ReconstructedPoint3D();
                euclideanPoint.setId(metricPoint.getId());
                euclideanPoint.setPoint(euclideanReconstructedPoints3D.get(i));
                euclideanPoint.setInlier(metricPoint.isInlier());
                euclideanPoint.setQualityScore(metricPoint.getQualityScore());
                if (metricPoint.getCovariance() != null) {
                    euclideanPoint.setCovariance(metricPoint.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
                }
                euclideanPoint.setColorData(metricPoint.getColorData());

                mActiveEuclideanReconstructedPoints.add(euclideanPoint);
            }

            return true;

        } catch (final Exception e) {
            mFailed = true;
            //noinspection unchecked
            mListener.onFail((R) this);

            return false;
        }
    }
}
