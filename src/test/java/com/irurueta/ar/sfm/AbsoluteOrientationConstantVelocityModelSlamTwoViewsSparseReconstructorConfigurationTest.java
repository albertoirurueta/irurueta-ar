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

import com.irurueta.ar.slam.AbsoluteOrientationConstantVelocityModelSlamCalibrationData;
import org.junit.Test;

import static org.junit.Assert.*;

public class AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfigurationTest {

    @Test
    public void testConstructor() {
        final AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();

        // check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getInitialCamerasAspectRatio(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(cfg.getPrincipalPointY(), 0.0, 0.0);
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getInitialCamerasMarkValidTriangulatedPoints(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(cfg.getInitialIntrinsic1());
        assertNull(cfg.getInitialIntrinsic2());
        assertEquals(cfg.isGeneralSceneAllowed(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
        assertNull(cfg.getCalibrationData());
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                SlamTwoViewsSparseReconstructorConfiguration.DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                SlamTwoViewsSparseReconstructorConfiguration.DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);
    }

    @Test
    public void testMake() {
        final AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration cfg =
                AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration.make();

        // check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getInitialCamerasAspectRatio(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(cfg.getPrincipalPointY(), 0.0, 0.0);
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getInitialCamerasMarkValidTriangulatedPoints(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(cfg.getInitialIntrinsic1());
        assertNull(cfg.getInitialIntrinsic2());
        assertEquals(cfg.isGeneralSceneAllowed(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
        assertNull(cfg.getCalibrationData());
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                SlamTwoViewsSparseReconstructorConfiguration.DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                SlamTwoViewsSparseReconstructorConfiguration.DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);
    }

    @Test
    public void testGetSetCalibrationData() {
        final AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();

        // check default value
        assertNull(cfg.getCalibrationData());

        // set new value
        final AbsoluteOrientationConstantVelocityModelSlamCalibrationData calibrationData =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrationData();
        assertSame(cfg, cfg.setCalibrationData(calibrationData));

        // check correctness
        assertSame(cfg.getCalibrationData(), calibrationData);
    }

    @Test
    public void testIsSetNotifyAvailableSlamDataEnabled() {
        final AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);

        // set new value
        assertSame(cfg.setNotifyAvailableSlamDataEnabled(
                !AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE), cfg);

        // check correctness
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                !AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
    }

    @Test
    public void testIsSetNotifyEstimatedSlamCameraEnabled() {
        final AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);

        // set new value
        assertSame(cfg.setNotifyEstimatedSlamCameraEnabled(
                !AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA), cfg);

        // check correctness
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                !AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);
    }
}
