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
 * image point correspondences in two views and known camera baseline (camera
 * separation), so that both cameras and reconstructed points are obtained with
 * exact scale.
 */
public class KnownBaselineTwoViewsSparseReconstructor extends
        BaseTwoViewsSparseReconstructor<
                KnownBaselineTwoViewsSparseReconstructorConfiguration,
                KnownBaselineTwoViewsSparseReconstructor,
                KnownBaselineTwoViewsSparseReconstructorListener> {

    /**
     * Constructor.
     *
     * @param configuration configuration for this re-constructor.
     * @param listener      listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     *                              provided.
     */
    public KnownBaselineTwoViewsSparseReconstructor(
            final KnownBaselineTwoViewsSparseReconstructorConfiguration configuration,
            final KnownBaselineTwoViewsSparseReconstructorListener listener) {
        super(configuration, listener);
    }

    /**
     * Constructor with default configuration.
     *
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener is not provided.
     */
    public KnownBaselineTwoViewsSparseReconstructor(
            final KnownBaselineTwoViewsSparseReconstructorListener listener) {
        this(new KnownBaselineTwoViewsSparseReconstructorConfiguration(),
                listener);
    }

    /**
     * Called when processing one frame is successfully finished. This can be done to estimate scale on
     * those implementations where scale can be measured or is already known.
     *
     * @return true if post-processing succeeded, false otherwise.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    protected boolean postProcessOne() {
        try {
            // reconstruction succeeded, so we update scale of cameras and
            // reconstructed points
            final double baseline = mConfiguration.getBaseline();

            final PinholeCamera camera1 = mEstimatedCamera1.getCamera();
            final PinholeCamera camera2 = mEstimatedCamera2.getCamera();

            camera1.decompose();
            camera2.decompose();

            final Point3D center1 = camera1.getCameraCenter();
            final Point3D center2 = camera2.getCameraCenter();

            final double estimatedBaseline = center1.distanceTo(center2);

            final double scale = baseline / estimatedBaseline;

            final MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            // update scale of cameras
            scaleTransformation.transform(camera1);
            scaleTransformation.transform(camera2);

            mEstimatedCamera1.setCamera(camera1);
            mEstimatedCamera2.setCamera(camera2);

            // update scale of reconstructed points
            final int numPoints = mReconstructedPoints.size();
            final List<Point3D> reconstructedPoints3D = new ArrayList<>();
            for (final ReconstructedPoint3D reconstructedPoint : mReconstructedPoints) {
                reconstructedPoints3D.add(reconstructedPoint.
                        getPoint());
            }

            scaleTransformation.transformAndOverwritePoints(
                    reconstructedPoints3D);

            // set scaled points into result
            for (int i = 0; i < numPoints; i++) {
                mReconstructedPoints.get(i).setPoint(
                        reconstructedPoints3D.get(i));
            }

            return true;
        } catch (final Exception e) {
            mFailed = true;
            mListener.onFail(this);

            return false;
        }
    }
}
