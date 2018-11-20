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
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Rotation3D;

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
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not provided.
     */
    public PairedViewsSparseReconstructor(PairedViewsSparseReconstructorConfiguration configuration,
            PairedViewsSparseReconstructorListener listener) throws NullPointerException {
        super(configuration, listener);
    }

    /**
     * Constructor with default configuration.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not provided.
     */
    public PairedViewsSparseReconstructor(PairedViewsSparseReconstructorListener listener)
            throws NullPointerException {
        this(new PairedViewsSparseReconstructorConfiguration(), listener);
    }

    /**
     * Indicates whether implementations of a reconstructor uses absolute orientation or
     * not.
     * @return true if absolute orientation is used, false, otherwise.
     */
    @Override
    protected boolean hasAbsoluteOrientation() {
        return false;
    }

    /**
     * Transforms metric cameras on current pair of views so that they are referred to
     * last kept location and rotation.
     * @param isInitialPairOfViews true if initial pair of views is being processed, false otherwise.
     * @param hasAbsoluteOrientation true if absolute orientation is required, false otherwise.
     * @return true if cameras were successfully transformed.
     */
    @Override
    protected boolean transformPairOfCamerasAndPoints(boolean isInitialPairOfViews,
                                                      boolean hasAbsoluteOrientation) {
        PinholeCamera previousMetricCamera = mPreviousMetricEstimatedCamera.getCamera();
        PinholeCamera currentMetricCamera = mCurrentMetricEstimatedCamera.getCamera();
        if (previousMetricCamera == null || currentMetricCamera == null) {
            return false;
        }

        mCurrentScale = mListener.onBaselineRequested(this, mPreviousViewId, mCurrentViewId,
                mPreviousMetricEstimatedCamera, mCurrentMetricEstimatedCamera);
        double sqrScale = mCurrentScale * mCurrentScale;

        MetricTransformation3D scaleTransformation = new MetricTransformation3D(mCurrentScale);

        if (isInitialPairOfViews) {
            //first pair of views does not require setting translation and rotation
            mReferenceEuclideanTransformation = scaleTransformation;
        } else {
             //additional pairs also need to translate and rotate
            Rotation3D invRot = mLastEuclideanCameraRotation.inverseRotationAndReturnNew();
            double[] translation = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            translation[0] = mLastEuclideanCameraCenter.getInhomX();
            translation[1] = mLastEuclideanCameraCenter.getInhomY();
            translation[2] = mLastEuclideanCameraCenter.getInhomZ();
            mReferenceEuclideanTransformation = scaleTransformation.
                    combineAndReturnNew(new MetricTransformation3D(invRot, translation, 1.0));
            mReferenceEuclideanTransformation.setRotation(invRot);
            mReferenceEuclideanTransformation.setTranslation(mLastEuclideanCameraCenter);
        }

        try {
            //transform cameras
            PinholeCamera previousEuclideanCamera = mReferenceEuclideanTransformation.
                    transformAndReturnNew(previousMetricCamera);
            PinholeCamera currentEuclideanCamera = mReferenceEuclideanTransformation.
                    transformAndReturnNew(currentMetricCamera);

            mPreviousEuclideanEstimatedCamera = new EstimatedCamera();
            mPreviousEuclideanEstimatedCamera.setCamera(previousEuclideanCamera);
            mPreviousEuclideanEstimatedCamera.setViewId(mPreviousMetricEstimatedCamera.getViewId());
            mPreviousEuclideanEstimatedCamera.setQualityScore(mPreviousMetricEstimatedCamera.getQualityScore());
            if (mPreviousMetricEstimatedCamera.getCovariance() != null) {
                mPreviousEuclideanEstimatedCamera.setCovariance(
                        mPreviousMetricEstimatedCamera.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
            }

            mCurrentEuclideanEstimatedCamera = new EstimatedCamera();
            mCurrentEuclideanEstimatedCamera.setCamera(currentEuclideanCamera);
            mCurrentEuclideanEstimatedCamera.setViewId(mCurrentMetricEstimatedCamera.getViewId());
            mCurrentEuclideanEstimatedCamera.setQualityScore(mCurrentMetricEstimatedCamera.getQualityScore());
            if (mCurrentMetricEstimatedCamera.getCovariance() != null) {
                mCurrentEuclideanEstimatedCamera.setCovariance(
                        mCurrentMetricEstimatedCamera.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
            }

            //transform points
            mEuclideanReconstructedPoints = new ArrayList<>();
            ReconstructedPoint3D euclideanReconstructedPoint;
            Point3D metricPoint, euclideanPoint;
            for (ReconstructedPoint3D metricReconstructedPoint : mMetricReconstructedPoints) {
                metricPoint = metricReconstructedPoint.getPoint();
                euclideanPoint = mReferenceEuclideanTransformation.transformAndReturnNew(
                        metricPoint);
                euclideanReconstructedPoint = new ReconstructedPoint3D();
                euclideanReconstructedPoint.setPoint(euclideanPoint);
                euclideanReconstructedPoint.setInlier(metricReconstructedPoint.isInlier());
                euclideanReconstructedPoint.setId(metricReconstructedPoint.getId());
                euclideanReconstructedPoint.setColorData(metricReconstructedPoint.getColorData());
                if (metricReconstructedPoint.getCovariance() != null) {
                    euclideanReconstructedPoint.setCovariance(metricReconstructedPoint.getCovariance().
                            multiplyByScalarAndReturnNew(sqrScale));
                }
                euclideanReconstructedPoint.setQualityScore(metricReconstructedPoint.getQualityScore());
                mEuclideanReconstructedPoints.add(euclideanReconstructedPoint);
            }

        } catch (AlgebraException e) {
            return false;
        }

        return super.transformPairOfCamerasAndPoints(isInitialPairOfViews, hasAbsoluteOrientation);
    }
}
