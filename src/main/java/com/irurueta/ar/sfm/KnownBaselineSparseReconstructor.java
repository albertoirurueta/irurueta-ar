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

import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point3D;

import java.util.ArrayList;
import java.util.List;

/**
 * Class in charge of estimating cameras and 3D reconstructed points from sparse
 * image point correspondences from multiple views and known initial camera baseline
 * (camera separation), so that cameras and reconstructed points are obtained with
 * exact scale.
 */
public class KnownBaselineSparseReconstructor extends
        BaseSparseReconstructor<KnownBaselineSparseReconstructorConfiguration,
                KnownBaselineSparseReconstructor, KnownBaselineSparseReconstructorListener> {

    /**
     * Constructor.
     *
     * @param configuration configuration for this reconstructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    public KnownBaselineSparseReconstructor(
            final KnownBaselineSparseReconstructorConfiguration configuration,
            final KnownBaselineSparseReconstructorListener listener) {
        super(configuration, listener);
    }

    /**
     * Constructor with default configuration.
     *
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener is not provided.
     */
    public KnownBaselineSparseReconstructor(
            final KnownBaselineSparseReconstructorListener listener) {
        this(new KnownBaselineSparseReconstructorConfiguration(), listener);
    }

    /**
     * Called when processing one frame is successfully finished. This can be done to estimate scale on those
     * implementations where scale can be measured or is already known.
     *
     * @param isInitialPairOfViews true if initial pair of views is being processed, false otherwise.
     * @return true if post processing succeeded, false otherwise.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    protected boolean postProcessOne(final boolean isInitialPairOfViews) {
        try {
            final PinholeCamera metricCamera1 = mPreviousMetricEstimatedCamera.getCamera();
            final PinholeCamera metricCamera2 = mCurrentMetricEstimatedCamera.getCamera();

            metricCamera1.decompose();
            metricCamera2.decompose();

            final double scale;
            if (isInitialPairOfViews) {
                // reconstruction succeeded, so we update scale of cameras and
                // reconstructed points
                final double baseline = mConfiguration.getBaseline();

                final Point3D center1 = metricCamera1.getCameraCenter();
                final Point3D center2 = metricCamera2.getCameraCenter();

                final double estimatedBaseline = center1.distanceTo(center2);

                scale = mCurrentScale = baseline / estimatedBaseline;
            } else {
                scale = mCurrentScale;
            }

            final double sqrScale = scale * scale;

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            // update scale of cameras
            final PinholeCamera euclideanCamera1 = scaleTransformation.transformAndReturnNew(metricCamera1);
            final PinholeCamera euclideanCamera2 = scaleTransformation.transformAndReturnNew(metricCamera2);

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
            for (ReconstructedPoint3D activeMetricReconstructedPoint : mActiveMetricReconstructedPoints) {
                metricReconstructedPoints3D.add(activeMetricReconstructedPoint.
                        getPoint());
            }

            final List<Point3D> euclideanReconstructedPoints3D = scaleTransformation.transformPointsAndReturnNew(
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
            mListener.onFail(this);

            return false;
        }
    }
}
