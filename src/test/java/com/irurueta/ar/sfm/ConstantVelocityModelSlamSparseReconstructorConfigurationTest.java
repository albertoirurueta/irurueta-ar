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
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.SerializationHelper;
import com.irurueta.ar.epipolar.CorrectorType;
import com.irurueta.ar.epipolar.estimators.FundamentalMatrixEstimatorMethod;
import com.irurueta.ar.slam.ConstantVelocityModelSlamCalibrationData;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.*;

public class ConstantVelocityModelSlamSparseReconstructorConfigurationTest {

    @Test
    public void testConstructor() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default values
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getRobustFundamentalMatrixEstimatorMethod());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_REFINE_FUNDAMENTAL_MATRIX, cfg.isFundamentalMatrixRefined());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE, cfg.isFundamentalMatrixCovarianceKept());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, cfg.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS, cfg.getFundamentalMatrixMaxIterations());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, cfg.getFundamentalMatrixThreshold(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS,
                cfg.getFundamentalMatrixComputeAndKeepInliers());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD, cfg.getInitialCamerasEstimatorMethod());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR,
                cfg.getDaqUseHomogeneousPointTriangulator());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO, cfg.getInitialCamerasAspectRatio(), 0.0);
        assertEquals(0.0, cfg.getPrincipalPointX(), 0.0);
        assertEquals(0.0, cfg.getPrincipalPointY(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE, cfg.getInitialCamerasCorrectorType());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS,
                cfg.getInitialCamerasMarkValidTriangulatedPoints());
        assertNull(cfg.getInitialIntrinsic1());
        assertNull(cfg.getInitialIntrinsic2());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ALLOW_GENERAL_SCENE, cfg.isGeneralSceneAllowed());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE,
                cfg.isPlanarSceneAllowed());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD,
                cfg.getRobustPlanarHomographyEstimatorMethod());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_REFINE_PLANAR_HOMOGRAPHY, cfg.isPlanarHomographyRefined());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE, cfg.isPlanarHomographyCovarianceKept());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, cfg.getPlanarHomographyConfidence(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS, cfg.getPlanarHomographyMaxIterations());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, cfg.getPlanarHomographyThreshold(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS,
                cfg.getUseDAQForAdditionalCamerasIntrinsics());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS,
                cfg.getUseDIACForAdditionalCamerasIntrinsics());
        assertNull(cfg.getAdditionalCamerasIntrinsics());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS, cfg.getAdditionalCamerasSkewness(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT,
                cfg.getAdditionalCamerasHorizontalPrincipalPoint(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT,
                cfg.getAdditionalCamerasVerticalPrincipalPoint(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_ASPECT_RATIO, cfg.getAdditionalCamerasAspectRatio(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION,
                cfg.getUseEPnPForAdditionalCamerasEstimation());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION,
                cfg.getUseUPnPForAdditionalCamerasEstimation());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD,
                cfg.getAdditionalCamerasRobustEstimationMethod());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION,
                cfg.getAdditionalCamerasAllowPlanarConfiguration());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2,
                cfg.getAdditionalCamerasAllowNullspaceDimension2());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3,
                cfg.getAdditionalCamerasAllowNullspaceDimension3());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD,
                cfg.getAdditionalCamerasPlanarThreshold(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_REFINE_ADDITIONAL_CAMERAS, cfg.areAdditionalCamerasRefined());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS, cfg.isAdditionalCamerasCovarianceKept());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT,
                cfg.getAdditionalCamerasUseFastRefinement());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE, cfg.getAdditionalCamerasConfidence(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS, cfg.getAdditionalCamerasMaxIterations());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_THRESHOLD, cfg.getAdditionalCamerasThreshold(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS,
                cfg.getAdditionalCamerasComputeAndKeepInliers());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getAdditionalCamerasComputeAndKeepResiduals());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED,
                cfg.isAdditionalCamerasSuggestSkewnessValueEnabled());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE,
                cfg.getAdditionalCamerasSuggestedSkewnessValue(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(),
                0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED,
                cfg.isAdditionalCamerasSuggestAspectRatioEnabled());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE,
                cfg.getAdditionalCamerasSuggestedAspectRatioValue(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED,
                cfg.isAdditionalCamerasSuggestPrincipalPointEnabled());
        assertNull(cfg.getAdditionalCamerasSuggestedPrincipalPointValue());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR, cfg.isHomogeneousPointTriangulatorUsed());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD, cfg.getRobustPointTriangulatorMethod());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_POINT_TRIANGULATOR_CONFIDENCE, cfg.getPointTriangulatorConfidence(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS, cfg.getPointTriangulatorMaxIterations());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_POINT_TRIANGULATOR_THRESHOLD, cfg.getPointTriangulatorThreshold(), 0.0);
        assertNull(cfg.getCalibrationData());
        assertNotNull(cfg.getCameraPositionCovariance());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE, cfg.isNotifyAvailableSlamDataEnabled());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA, cfg.isNotifyEstimatedSlamCameraEnabled());
    }

    @Test
    public void testMake() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                ConstantVelocityModelSlamSparseReconstructorConfiguration.make();

        // check default values
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getRobustFundamentalMatrixEstimatorMethod());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_REFINE_FUNDAMENTAL_MATRIX, cfg.isFundamentalMatrixRefined());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE, cfg.isFundamentalMatrixCovarianceKept());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, cfg.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS, cfg.getFundamentalMatrixMaxIterations());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, cfg.getFundamentalMatrixThreshold(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS,
                cfg.getFundamentalMatrixComputeAndKeepInliers());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD, cfg.getInitialCamerasEstimatorMethod());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR,
                cfg.getDaqUseHomogeneousPointTriangulator());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO, cfg.getInitialCamerasAspectRatio(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_X, cfg.getPrincipalPointX(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_Y, cfg.getPrincipalPointY(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE, cfg.getInitialCamerasCorrectorType());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS,
                cfg.getInitialCamerasMarkValidTriangulatedPoints());
        assertNull(cfg.getInitialIntrinsic1());
        assertNull(cfg.getInitialIntrinsic2());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE,
                cfg.isGeneralSceneAllowed());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE,
                cfg.isPlanarSceneAllowed());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD,
                cfg.getRobustPlanarHomographyEstimatorMethod());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_REFINE_PLANAR_HOMOGRAPHY, cfg.isPlanarHomographyRefined());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE, cfg.isPlanarHomographyCovarianceKept());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, cfg.getPlanarHomographyConfidence(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS, cfg.getPlanarHomographyMaxIterations());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, cfg.getPlanarHomographyThreshold(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS,
                cfg.getUseDAQForAdditionalCamerasIntrinsics());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS,
                cfg.getUseDIACForAdditionalCamerasIntrinsics());
        assertNull(cfg.getAdditionalCamerasIntrinsics());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS, cfg.getAdditionalCamerasSkewness(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT,
                cfg.getAdditionalCamerasHorizontalPrincipalPoint(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT,
                cfg.getAdditionalCamerasVerticalPrincipalPoint(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_ASPECT_RATIO, cfg.getAdditionalCamerasAspectRatio(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION,
                cfg.getUseEPnPForAdditionalCamerasEstimation());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION,
                cfg.getUseUPnPForAdditionalCamerasEstimation());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD,
                cfg.getAdditionalCamerasRobustEstimationMethod());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION,
                cfg.getAdditionalCamerasAllowPlanarConfiguration());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2,
                cfg.getAdditionalCamerasAllowNullspaceDimension2());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3,
                cfg.getAdditionalCamerasAllowNullspaceDimension3());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD,
                cfg.getAdditionalCamerasPlanarThreshold(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_REFINE_ADDITIONAL_CAMERAS, cfg.areAdditionalCamerasRefined());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS, cfg.isAdditionalCamerasCovarianceKept());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT,
                cfg.getAdditionalCamerasUseFastRefinement());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE, cfg.getAdditionalCamerasConfidence(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS, cfg.getAdditionalCamerasMaxIterations());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_THRESHOLD, cfg.getAdditionalCamerasThreshold(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS,
                cfg.getAdditionalCamerasComputeAndKeepInliers());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getAdditionalCamerasComputeAndKeepResiduals());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED,
                cfg.isAdditionalCamerasSuggestSkewnessValueEnabled());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE,
                cfg.getAdditionalCamerasSuggestedSkewnessValue(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(),
                0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED,
                cfg.isAdditionalCamerasSuggestAspectRatioEnabled());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE,
                cfg.getAdditionalCamerasSuggestedAspectRatioValue(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED,
                cfg.isAdditionalCamerasSuggestPrincipalPointEnabled());
        assertNull(cfg.getAdditionalCamerasSuggestedPrincipalPointValue());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR, cfg.isHomogeneousPointTriangulatorUsed());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD, cfg.getRobustPointTriangulatorMethod());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_POINT_TRIANGULATOR_CONFIDENCE, cfg.getPointTriangulatorConfidence(), 0.0);
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS, cfg.getPointTriangulatorMaxIterations());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_POINT_TRIANGULATOR_THRESHOLD, cfg.getPointTriangulatorThreshold(), 0.0);
        assertNull(cfg.getCalibrationData());
        assertNotNull(cfg.getCameraPositionCovariance());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE, cfg.isNotifyAvailableSlamDataEnabled());
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA, cfg.isNotifyEstimatedSlamCameraEnabled());
    }

    @Test
    public void testGetSetNonRobustFundamentalMatrixEstimatorMethod() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getNonRobustFundamentalMatrixEstimatorMethod());

        // set new value
        assertSame(cfg, cfg.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM));

        // check correctness
        assertEquals(FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM,
                cfg.getNonRobustFundamentalMatrixEstimatorMethod());
    }

    @Test
    public void testGetSetRobustFundamentalMatrixEstimatorMethod() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getRobustFundamentalMatrixEstimatorMethod());

        // set new value
        assertSame(cfg, cfg.setRobustFundamentalMatrixEstimatorMethod(RobustEstimatorMethod.LMEDS));

        // check correctness
        assertEquals(RobustEstimatorMethod.LMEDS, cfg.getRobustFundamentalMatrixEstimatorMethod());
    }

    @Test
    public void testIsSetFundamentalMatrixRefined() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_REFINE_FUNDAMENTAL_MATRIX, cfg.isFundamentalMatrixRefined());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixRefined(false));

        // check correctness
        assertFalse(cfg.isFundamentalMatrixRefined());
    }

    @Test
    public void testIsSetFundamentalMatrixCovarianceKept() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE, cfg.isFundamentalMatrixCovarianceKept());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixCovarianceKept(true));

        // check correctness
        assertTrue(cfg.isFundamentalMatrixCovarianceKept());
    }

    @Test
    public void testGetSetFundamentalMatrixConfidence() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, cfg.getFundamentalMatrixConfidence(), 0.0);

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixConfidence(0.7));

        // check correctness
        assertEquals(0.7, cfg.getFundamentalMatrixConfidence(), 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixMaxIterations() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS, cfg.getFundamentalMatrixMaxIterations());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixMaxIterations(10));

        // check correctness
        assertEquals(10, cfg.getFundamentalMatrixMaxIterations());
    }

    @Test
    public void testGetSetFundamentalMatrixThreshold() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, cfg.getFundamentalMatrixThreshold(), 0.0);

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixThreshold(2.0));

        // check correctness
        assertEquals(2.0, cfg.getFundamentalMatrixThreshold(), 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepInliers() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS,
                cfg.getFundamentalMatrixComputeAndKeepInliers());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixComputeAndKeepInliers(false));

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepInliers());
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepResiduals() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getFundamentalMatrixComputeAndKeepResiduals());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixComputeAndKeepResiduals(false));

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetInitialCamerasEstimatorMethod() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD, cfg.getInitialCamerasEstimatorMethod());

        // set new value
        assertSame(cfg, cfg.setInitialCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC));

        // check correctness
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC,
                cfg.getInitialCamerasEstimatorMethod());
    }

    @Test
    public void testGetSetDaqUseHomogeneousPointTriangulator() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR,
                cfg.getDaqUseHomogeneousPointTriangulator());

        // set new value
        assertSame(cfg, cfg.setDaqUseHomogeneousPointTriangulator(false));

        // check correctness
        assertFalse(cfg.getDaqUseHomogeneousPointTriangulator());
    }

    @Test
    public void testGetSetInitialCamerasAspectRatio() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO, cfg.getInitialCamerasAspectRatio(), 0.0);

        // set new value
        assertSame(cfg.setInitialCamerasAspectRatio(0.5), cfg);

        // check correctness
        assertEquals(0.5, cfg.getInitialCamerasAspectRatio(), 0.0);
    }

    @Test
    public void testGetSetPrincipalPointX() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_X, cfg.getPrincipalPointX(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPrincipalPointX(10.0));

        // check correctness
        assertEquals(10.0, cfg.getPrincipalPointX(), 0.0);
    }

    @Test
    public void testGetSetPrincipalPointY() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_Y, cfg.getPrincipalPointY(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPrincipalPointY(10.0));

        // check correctness
        assertEquals(10.0, cfg.getPrincipalPointY(), 0.0);
    }

    @Test
    public void testGetSetInitialCamerasCorrectorType() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE, cfg.getInitialCamerasCorrectorType());

        // set new value
        assertSame(cfg, cfg.setInitialCamerasCorrectorType(CorrectorType.GOLD_STANDARD));

        // check correctness
        assertEquals(CorrectorType.GOLD_STANDARD, cfg.getInitialCamerasCorrectorType());
    }

    @Test
    public void testGetSetInitialCamerasMarkValidTriangulatedPoints() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS,
                cfg.getInitialCamerasMarkValidTriangulatedPoints());

        // set new value
        assertSame(cfg, cfg.setInitialCamerasMarkValidTriangulatedPoints(false));

        // check correctness
        assertFalse(cfg.getInitialCamerasMarkValidTriangulatedPoints());
    }

    @Test
    public void testGetSetInitialIntrinsic1() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertNull(cfg.getInitialIntrinsic1());

        // set new value
        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters();
        assertSame(cfg, cfg.setInitialIntrinsic1(intrinsic));

        // check correctness
        assertSame(intrinsic, cfg.getInitialIntrinsic1());
    }

    @Test
    public void testGetSetInitialIntrinsic2() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertNull(cfg.getInitialIntrinsic2());

        // set new value
        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters();
        assertSame(cfg, cfg.setInitialIntrinsic2(intrinsic));

        // check correctness
        assertSame(intrinsic, cfg.getInitialIntrinsic2());
    }

    @Test
    public void testIsSetGeneralSceneAllowed() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE,
                cfg.isGeneralSceneAllowed());

        // set new value
        assertSame(cfg, cfg.setGeneralSceneAllowed(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ALLOW_GENERAL_SCENE, cfg.isGeneralSceneAllowed());
    }

    @Test
    public void testIsSetPlanarSceneAllowed() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE,
                cfg.isPlanarSceneAllowed());

        // set new value
        assertSame(cfg, cfg.setPlanarSceneAllowed(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE,
                cfg.isPlanarSceneAllowed());
    }

    @Test
    public void testGetSetRobustPlanarHomographyEstimatorMethod() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD,
                cfg.getRobustPlanarHomographyEstimatorMethod());

        // set new value
        assertSame(cfg, cfg.setRobustPlanarHomographyEstimatorMethod(RobustEstimatorMethod.RANSAC));

        // check correctness
        assertEquals(RobustEstimatorMethod.RANSAC, cfg.getRobustPlanarHomographyEstimatorMethod());
    }

    @Test
    public void testIsSetPlanarHomographyRefined() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_REFINE_PLANAR_HOMOGRAPHY, cfg.isPlanarHomographyRefined());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyRefined(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_REFINE_PLANAR_HOMOGRAPHY, cfg.isPlanarHomographyRefined());
    }

    @Test
    public void testIsSetPlanarHomographyCovarianceKept() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE, cfg.isPlanarHomographyCovarianceKept());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyCovarianceKept(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE, cfg.isPlanarHomographyCovarianceKept());
    }

    @Test
    public void testGetSetPlanarHomographyConfidence() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, cfg.getPlanarHomographyConfidence(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyConfidence(0.5));

        // check correctness
        assertEquals(0.5, cfg.getPlanarHomographyConfidence(), 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyMaxIterations() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS, cfg.getPlanarHomographyMaxIterations());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyMaxIterations(100));

        // check correctness
        assertEquals(100, cfg.getPlanarHomographyMaxIterations());
    }

    @Test
    public void testGetSetPlanarHomographyThreshold() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(0.0, ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, cfg.getPlanarHomographyThreshold());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyThreshold(0.5));

        // check correctness
        assertEquals(0.5, cfg.getPlanarHomographyThreshold(), 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepInliers() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyComputeAndKeepInliers(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepResiduals() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyComputeAndKeepResiduals(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetUseDAQForAdditionalCamerasIntrinsics() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS,
                cfg.getUseDAQForAdditionalCamerasIntrinsics());

        // set new value
        assertSame(cfg, cfg.setUseDAQForAdditionalCamerasIntrinics(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS,
                cfg.getUseDAQForAdditionalCamerasIntrinsics());
    }

    @Test
    public void testGetSetUseDIACForAdditionalCamerasIntrinsics() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS,
                cfg.getUseDIACForAdditionalCamerasIntrinsics());

        // set new value
        assertSame(cfg, cfg.setUseDIACForAdditionalCamerasIntrinsics(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS,
                cfg.getUseDIACForAdditionalCamerasIntrinsics());
    }

    @Test
    public void testGetSetAdditionalCamerasIntrinsics() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertNull(cfg.getAdditionalCamerasIntrinsics());

        // set new value
        final PinholeCameraIntrinsicParameters intrinsics = new PinholeCameraIntrinsicParameters();
        assertSame(cfg, cfg.setAdditionalCamerasIntrinsics(intrinsics));

        // check correctness
        assertSame(intrinsics, cfg.getAdditionalCamerasIntrinsics());
    }

    @Test
    public void testGetSetAdditionalCamerasSkewness() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS,
                cfg.getAdditionalCamerasSkewness(), 0.0);

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasSkewness(1e-3));

        // check correctness
        assertEquals(1e-3, cfg.getAdditionalCamerasSkewness(), 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasHorizontalPrincipalPoint() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT,
                cfg.getAdditionalCamerasHorizontalPrincipalPoint(), 0.0);

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasHorizontalPrincipalPoint(320.0));

        // check correctness
        assertEquals(320.0, cfg.getAdditionalCamerasHorizontalPrincipalPoint(), 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasVerticalPrincipalPoint() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT,
                cfg.getAdditionalCamerasVerticalPrincipalPoint(), 0.0);

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasVerticalPrincipalPoint(240.0));

        // check correctness
        assertEquals(240.0, cfg.getAdditionalCamerasVerticalPrincipalPoint(), 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasAspectRatio() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ASPECT_RATIO, cfg.getAdditionalCamerasAspectRatio(),
                0.0);

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasAspectRatio(-1.0));

        // check correctness
        assertEquals(-1.0, cfg.getAdditionalCamerasAspectRatio(), 0.0);
    }

    @Test
    public void testGetSetUseEPnPForAdditionalCamerasEstimation() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION,
                cfg.getUseEPnPForAdditionalCamerasEstimation());

        // set new value
        assertSame(cfg, cfg.setUseEPnPForAdditionalCamerasEstimation(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION,
                cfg.getUseEPnPForAdditionalCamerasEstimation());
    }

    @Test
    public void testGetSetUseUPnPForAdditionalCamerasEstimation() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION,
                cfg.getUseUPnPForAdditionalCamerasEstimation());

        // set new value
        assertSame(cfg, cfg.setUseUPnPForAdditionalCamerasEstimation(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION,
                cfg.getUseUPnPForAdditionalCamerasEstimation());
    }

    @Test
    public void testGetSetAdditionalCamerasRobustEstimationMethod() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD,
                cfg.getAdditionalCamerasRobustEstimationMethod());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasRobustEstimationMethod(RobustEstimatorMethod.LMEDS));

        // check correctness
        assertEquals(RobustEstimatorMethod.LMEDS, cfg.getAdditionalCamerasRobustEstimationMethod());
    }

    @Test
    public void testGetSetAdditionalCamerasAllowPlanarConfiguration() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION,
                cfg.getAdditionalCamerasAllowPlanarConfiguration());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasAllowPlanarConfiguration(false));

        // check correctness
        assertFalse(cfg.getAdditionalCamerasAllowPlanarConfiguration());
    }

    @Test
    public void testGetSetAdditionalCamerasAllowNullspaceDimension2() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2,
                cfg.getAdditionalCamerasAllowNullspaceDimension2());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasAllowNullspaceDimension2(false));

        // check correctness
        assertFalse(cfg.getAdditionalCamerasAllowNullspaceDimension2());
    }

    @Test
    public void testGetSetAdditionalCamerasAllowNullspaceDimension3() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3,
                cfg.getAdditionalCamerasAllowNullspaceDimension3());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasAllowNullspaceDimension3(false));

        // check correctness
        assertFalse(cfg.getAdditionalCamerasAllowNullspaceDimension3());
    }

    @Test
    public void testGetSetAdditionalCamerasPlanarThreshold() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD,
                cfg.getAdditionalCamerasPlanarThreshold(), 0.0);

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasPlanarThreshold(1e-3));

        // check correctness
        assertEquals(1e-3, cfg.getAdditionalCamerasPlanarThreshold(), 0.0);
    }

    @Test
    public void testAreSetAdditionalCamerasRefined() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_REFINE_ADDITIONAL_CAMERAS, cfg.areAdditionalCamerasRefined());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasRefined(false));

        // check correctness
        assertFalse(cfg.areAdditionalCamerasRefined());
    }

    @Test
    public void testIsSetAdditionalCamerasCovarianceKept() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS, cfg.isAdditionalCamerasCovarianceKept());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasCovarianceKept(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS, cfg.isAdditionalCamerasCovarianceKept());
    }

    @Test
    public void testGetSetAdditionalCamerasUseFastRefinement() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT,
                cfg.getAdditionalCamerasUseFastRefinement());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasUseFastRefinement(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT,
                cfg.getAdditionalCamerasUseFastRefinement());
    }

    @Test
    public void testGetSetAdditionalCamerasConfidence() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE, cfg.getAdditionalCamerasConfidence(), 0.0);

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasConfidence(0.8));

        // check correctness
        assertEquals(0.8, cfg.getAdditionalCamerasConfidence(), 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasMaxIterations() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS, cfg.getAdditionalCamerasMaxIterations());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasMaxIterations(100));

        // check correctness
        assertEquals(100, cfg.getAdditionalCamerasMaxIterations());
    }

    @Test
    public void testGetSetAdditionalCamerasThreshold() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ADDITIONAL_CAMERAS_THRESHOLD, cfg.getAdditionalCamerasThreshold(), 0.0);

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasThreshold(2.0));

        // check correctness
        assertEquals(2.0, cfg.getAdditionalCamerasThreshold(), 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasComputeAndKeepInliers() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS,
                cfg.getAdditionalCamerasComputeAndKeepInliers());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasComputeAndKeepInliers(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS,
                cfg.getAdditionalCamerasComputeAndKeepInliers());
    }

    @Test
    public void testGetSetAdditionalCamerasComputeAndKeepResiduals() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getAdditionalCamerasComputeAndKeepResiduals());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasComputeAndKeepResiduals(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getAdditionalCamerasComputeAndKeepResiduals());
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestSkewnessValueEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED,
                cfg.isAdditionalCamerasSuggestSkewnessValueEnabled());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasSuggestSkewnessValueEnabled(true));

        // check correctness
        assertTrue(cfg.isAdditionalCamerasSuggestSkewnessValueEnabled());
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedSkewnessValue() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE,
                cfg.getAdditionalCamerasSuggestedSkewnessValue(), 0.0);

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasSuggestedSkewnessValue(1e-3));

        // check correctness
        assertEquals(1e-3, cfg.getAdditionalCamerasSuggestedSkewnessValue(), 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestHorizontalFocalLengthEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasSuggestHorizontalFocalLengthEnabled(true));

        // check correctness
        assertTrue(cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled());
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedHorizontalFocalLengthValue() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(0.0, cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(),
                0.0);

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasSuggestedHorizontalFocalLengthValue(320.0));

        // check correctness
        assertEquals(320.0, cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(),
                0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestVerticalFocalLengthEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasSuggestVerticalFocalLengthEnabled(true));

        // check correctness
        assertTrue(cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled());
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedVerticalFocalLengthValue() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(0.0, cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 0.0);

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasSuggestedVerticalFocalLengthValue(240.0));

        // check correctness
        assertEquals(240.0, cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(),
                0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestAspectRatioEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED,
                cfg.isAdditionalCamerasSuggestAspectRatioEnabled());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasSuggestAspectRatioEnabled(true));

        // check correctness
        assertTrue(cfg.isAdditionalCamerasSuggestAspectRatioEnabled());
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedAspectRatioValue() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE,
                cfg.getAdditionalCamerasSuggestedAspectRatioValue(), 0.0);

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasSuggestedAspectRatioValue(1.1));

        // check correctness
        assertEquals(1.1, cfg.getAdditionalCamerasSuggestedAspectRatioValue(), 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestPrincipalPointEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED,
                cfg.isAdditionalCamerasSuggestPrincipalPointEnabled());

        // set new value
        assertSame(cfg, cfg.setAdditionalCamerasSuggestPrincipalPointEnabled(true));

        assertTrue(cfg.isAdditionalCamerasSuggestPrincipalPointEnabled());
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedPrincipalPointValue() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertNull(cfg.getAdditionalCamerasSuggestedPrincipalPointValue());

        // set new value
        final InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D();
        assertSame(cfg, cfg.setAdditionalCamerasSuggestedPrincipalPointValue(principalPoint));

        // check correctness
        assertSame(principalPoint, cfg.getAdditionalCamerasSuggestedPrincipalPointValue());
    }

    @Test
    public void testIsSetHomogeneousPointTriangulatorUsed() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR, cfg.isHomogeneousPointTriangulatorUsed());

        // set new value
        assertSame(cfg, cfg.setHomogeneousPointTriangulatorUsed(false));

        // check correctness
        assertFalse(cfg.isHomogeneousPointTriangulatorUsed());
    }

    @Test
    public void testGetSetRobustPointTriangulatorMethod() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD, cfg.getRobustPointTriangulatorMethod());

        // set new value
        assertSame(cfg, cfg.setRobustPointTriangulatorMethod(RobustEstimatorMethod.MSAC));

        // check correctness
        assertEquals(RobustEstimatorMethod.MSAC, cfg.getRobustPointTriangulatorMethod());
    }

    @Test
    public void testGetSetPointTriangulatorConfidence() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_POINT_TRIANGULATOR_CONFIDENCE, cfg.getPointTriangulatorConfidence(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPointTriangulatorConfidence(0.8));

        // check correctness
        assertEquals(0.8, cfg.getPointTriangulatorConfidence(), 0.0);
    }

    @Test
    public void testGetSetPointTriangulatorMaxIterations() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS, cfg.getPointTriangulatorMaxIterations());

        // set new value
        assertSame(cfg, cfg.setPointTriangulatorMaxIterations(100));

        // check correctness
        assertEquals(100, cfg.getPointTriangulatorMaxIterations());
    }

    @Test
    public void testGetStPointTriangulatorThreshold() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_POINT_TRIANGULATOR_THRESHOLD, cfg.getPointTriangulatorThreshold(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPointTriangulatorThreshold(1e-3));

        // check correctness
        assertEquals(1e-3, cfg.getPointTriangulatorThreshold(), 0.0);
    }

    @Test
    public void testGetSetCalibrationData() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertNull(cfg.getCalibrationData());

        // set new value
        final ConstantVelocityModelSlamCalibrationData calibrationData =
                new ConstantVelocityModelSlamCalibrationData();
        assertSame(cfg, cfg.setCalibrationData(calibrationData));

        // check correctness
        assertSame(calibrationData, cfg.getCalibrationData());
    }

    @Test
    public void testGetSetCameraPositionVariance() throws AlgebraException {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertNotNull(cfg.getCameraPositionCovariance());

        // set new value
        final Matrix cov = new Matrix(3, 3);
        assertSame(cfg, cfg.setCameraPositionCovariance(cov));

        // check correctness
        assertSame(cov, cfg.getCameraPositionCovariance());
    }

    @Test
    public void testSetCameraPositionVariance() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertNotNull(cfg.getCameraPositionCovariance());

        // set new value
        final Matrix cov = Matrix.diagonal(new double[]{2.0, 2.0, 2.0});
        assertSame(cfg, cfg.setCameraPositionVariance(2.0));

        // check correctness
        assertEquals(cov, cfg.getCameraPositionCovariance());
    }

    @Test
    public void testIsSetNotifyAvailableSlamDataEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE, cfg.isNotifyAvailableSlamDataEnabled());

        // set new value
        assertSame(cfg.setNotifyAvailableSlamDataEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE), cfg);

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE, cfg.isNotifyAvailableSlamDataEnabled());
    }

    @Test
    public void testIsSetNotifyEstimatedSlamCameraEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA, cfg.isNotifyEstimatedSlamCameraEnabled());

        // set new value
        assertSame(cfg, cfg.setNotifyEstimatedSlamCameraEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA));

        // check correctness
        assertEquals(!ConstantVelocityModelSlamSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA, cfg.isNotifyEstimatedSlamCameraEnabled());
    }

    @Test
    public void testSerializeDeserialize() throws IOException, WrongSizeException, ClassNotFoundException {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg1 =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // set new values
        cfg1.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        cfg1.setRobustFundamentalMatrixEstimatorMethod(RobustEstimatorMethod.RANSAC);
        cfg1.setFundamentalMatrixRefined(false);
        cfg1.setFundamentalMatrixCovarianceKept(true);
        cfg1.setFundamentalMatrixConfidence(0.8);
        cfg1.setFundamentalMatrixMaxIterations(500);
        cfg1.setFundamentalMatrixThreshold(0.5);
        cfg1.setFundamentalMatrixComputeAndKeepInliers(false);
        cfg1.setFundamentalMatrixComputeAndKeepResiduals(false);
        cfg1.setInitialCamerasEstimatorMethod(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);
        cfg1.setDaqUseHomogeneousPointTriangulator(false);
        cfg1.setInitialCamerasAspectRatio(0.99);
        cfg1.setPrincipalPointX(1e-3);
        cfg1.setPrincipalPointY(-1e-3);
        cfg1.setInitialCamerasCorrectorType(CorrectorType.GOLD_STANDARD);
        cfg1.setInitialCamerasMarkValidTriangulatedPoints(false);
        final PinholeCameraIntrinsicParameters intrinsic1 = new PinholeCameraIntrinsicParameters();
        cfg1.setInitialIntrinsic1(intrinsic1);
        final PinholeCameraIntrinsicParameters intrinsic2 = new PinholeCameraIntrinsicParameters();
        cfg1.setInitialIntrinsic2(intrinsic2);
        cfg1.setGeneralSceneAllowed(false);
        cfg1.setPlanarSceneAllowed(false);
        cfg1.setRobustPlanarHomographyEstimatorMethod(RobustEstimatorMethod.RANSAC);
        cfg1.setPlanarHomographyRefined(false);
        cfg1.setPlanarHomographyCovarianceKept(true);
        cfg1.setPlanarHomographyConfidence(0.9);
        cfg1.setPlanarHomographyMaxIterations(500);
        cfg1.setPlanarHomographyThreshold(1e-2);
        cfg1.setPlanarHomographyComputeAndKeepInliers(false);
        cfg1.setPlanarHomographyComputeAndKeepResiduals(false);
        cfg1.setUseDAQForAdditionalCamerasIntrinics(false);
        cfg1.setUseDIACForAdditionalCamerasIntrinsics(true);
        final PinholeCameraIntrinsicParameters additionalIntrinsics =
                new PinholeCameraIntrinsicParameters();
        cfg1.setAdditionalCamerasIntrinsics(additionalIntrinsics);
        cfg1.setAdditionalCamerasSkewness(1e-4);
        cfg1.setAdditionalCamerasHorizontalPrincipalPoint(0.5);
        cfg1.setAdditionalCamerasVerticalPrincipalPoint(-0.5);
        cfg1.setAdditionalCamerasAspectRatio(0.99);
        cfg1.setUseEPnPForAdditionalCamerasEstimation(true);
        cfg1.setUseUPnPForAdditionalCamerasEstimation(false);
        cfg1.setAdditionalCamerasRobustEstimationMethod(RobustEstimatorMethod.LMEDS);
        cfg1.setAdditionalCamerasAllowPlanarConfiguration(false);
        cfg1.setAdditionalCamerasAllowNullspaceDimension2(false);
        cfg1.setAdditionalCamerasAllowNullspaceDimension3(false);
        cfg1.setAdditionalCamerasPlanarThreshold(1e9);
        cfg1.setAdditionalCamerasRefined(false);
        cfg1.setAdditionalCamerasCovarianceKept(false);
        cfg1.setAdditionalCamerasUseFastRefinement(false);
        cfg1.setAdditionalCamerasConfidence(0.8);
        cfg1.setAdditionalCamerasMaxIterations(200);
        cfg1.setAdditionalCamerasThreshold(0.5);
        cfg1.setAdditionalCamerasComputeAndKeepInliers(false);
        cfg1.setAdditionalCamerasComputeAndKeepResiduals(false);
        cfg1.setAdditionalCamerasSuggestSkewnessValueEnabled(true);
        cfg1.setAdditionalCamerasSuggestedSkewnessValue(1e-2);
        cfg1.setAdditionalCamerasSuggestHorizontalFocalLengthEnabled(true);
        cfg1.setAdditionalCamerasSuggestedHorizontalFocalLengthValue(1.0);
        cfg1.setAdditionalCamerasSuggestVerticalFocalLengthEnabled(true);
        cfg1.setAdditionalCamerasSuggestedVerticalFocalLengthValue(-1.0);
        cfg1.setAdditionalCamerasSuggestAspectRatioEnabled(true);
        cfg1.setAdditionalCamerasSuggestedAspectRatioValue(0.99);
        cfg1.setAdditionalCamerasSuggestPrincipalPointEnabled(true);
        final InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D();
        cfg1.setAdditionalCamerasSuggestedPrincipalPointValue(principalPoint);
        cfg1.setHomogeneousPointTriangulatorUsed(false);
        cfg1.setRobustPointTriangulatorMethod(RobustEstimatorMethod.PROSAC);
        cfg1.setPointTriangulatorConfidence(0.8);
        cfg1.setPointTriangulatorMaxIterations(100);
        cfg1.setPointTriangulatorThreshold(1.1);
        final ConstantVelocityModelSlamCalibrationData data =
                new ConstantVelocityModelSlamCalibrationData();
        cfg1.setCalibrationData(data);
        final Matrix cov = Matrix.identity(3, 3);
        cfg1.setCameraPositionCovariance(cov);
        cfg1.setNotifyAvailableSlamDataEnabled(false);
        cfg1.setNotifyEstimatedSlamCameraEnabled(false);

        // check
        assertEquals(FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM,
                cfg1.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(RobustEstimatorMethod.RANSAC, cfg1.getRobustFundamentalMatrixEstimatorMethod());
        assertFalse(cfg1.isFundamentalMatrixRefined());
        assertTrue(cfg1.isFundamentalMatrixCovarianceKept());
        assertEquals(0.8, cfg1.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(500, cfg1.getFundamentalMatrixMaxIterations());
        assertEquals(0.5, cfg1.getFundamentalMatrixThreshold(), 0.0);
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepInliers());
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC,
                cfg1.getInitialCamerasEstimatorMethod());
        assertFalse(cfg1.getDaqUseHomogeneousPointTriangulator());
        assertEquals(0.99, cfg1.getInitialCamerasAspectRatio(), 0.0);
        assertEquals(1e-3, cfg1.getPrincipalPointX(), 0.0);
        assertEquals(cfg1.getPrincipalPointY(), -1e-3, 0.0);
        assertEquals(CorrectorType.GOLD_STANDARD, cfg1.getInitialCamerasCorrectorType());
        assertFalse(cfg1.getInitialCamerasMarkValidTriangulatedPoints());
        assertSame(intrinsic1, cfg1.getInitialIntrinsic1());
        assertSame(intrinsic2, cfg1.getInitialIntrinsic2());
        assertFalse(cfg1.isGeneralSceneAllowed());
        assertFalse(cfg1.isPlanarSceneAllowed());
        assertEquals(RobustEstimatorMethod.RANSAC, cfg1.getRobustPlanarHomographyEstimatorMethod());
        assertFalse(cfg1.isPlanarHomographyRefined());
        assertTrue(cfg1.isPlanarHomographyCovarianceKept());
        assertEquals(0.9, cfg1.getPlanarHomographyConfidence(), 0.0);
        assertEquals(500, cfg1.getPlanarHomographyMaxIterations());
        assertEquals(1e-2, cfg1.getPlanarHomographyThreshold(), 0.0);
        assertFalse(cfg1.getPlanarHomographyComputeAndKeepInliers());
        assertFalse(cfg1.getPlanarHomographyComputeAndKeepResiduals());
        assertFalse(cfg1.getUseDAQForAdditionalCamerasIntrinsics());
        assertTrue(cfg1.getUseDIACForAdditionalCamerasIntrinsics());
        assertSame(additionalIntrinsics, cfg1.getAdditionalCamerasIntrinsics());
        assertEquals(1e-4, cfg1.getAdditionalCamerasSkewness(), 0.0);
        assertEquals(0.5, cfg1.getAdditionalCamerasHorizontalPrincipalPoint(), 0.0);
        assertEquals(-0.5, cfg1.getAdditionalCamerasVerticalPrincipalPoint(), 0.0);
        assertEquals(0.99, cfg1.getAdditionalCamerasAspectRatio(), 0.0);
        assertTrue(cfg1.getUseEPnPForAdditionalCamerasEstimation());
        assertFalse(cfg1.getUseUPnPForAdditionalCamerasEstimation());
        assertEquals(RobustEstimatorMethod.LMEDS, cfg1.getAdditionalCamerasRobustEstimationMethod());
        assertFalse(cfg1.getAdditionalCamerasAllowPlanarConfiguration());
        assertFalse(cfg1.getAdditionalCamerasAllowNullspaceDimension2());
        assertFalse(cfg1.getAdditionalCamerasAllowNullspaceDimension3());
        assertEquals(1e9, cfg1.getAdditionalCamerasPlanarThreshold(), 0.0);
        assertFalse(cfg1.areAdditionalCamerasRefined());
        assertFalse(cfg1.isAdditionalCamerasCovarianceKept());
        assertFalse(cfg1.getAdditionalCamerasUseFastRefinement());
        assertEquals(0.8, cfg1.getAdditionalCamerasConfidence(), 0.0);
        assertEquals(200, cfg1.getAdditionalCamerasMaxIterations());
        assertEquals(0.5, cfg1.getAdditionalCamerasThreshold(), 0.0);
        assertFalse(cfg1.getAdditionalCamerasComputeAndKeepInliers());
        assertFalse(cfg1.getAdditionalCamerasComputeAndKeepResiduals());
        assertTrue(cfg1.isAdditionalCamerasSuggestSkewnessValueEnabled());
        assertEquals(1e-2, cfg1.getAdditionalCamerasSuggestedSkewnessValue(), 0.0);
        assertTrue(cfg1.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled());
        assertEquals(1.0, cfg1.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(),
                0.0);
        assertTrue(cfg1.isAdditionalCamerasSuggestVerticalFocalLengthEnabled());
        assertEquals(-1.0, cfg1.getAdditionalCamerasSuggestedVerticalFocalLengthValue(),
                0.0);
        assertTrue(cfg1.isAdditionalCamerasSuggestAspectRatioEnabled());
        assertEquals(0.99, cfg1.getAdditionalCamerasSuggestedAspectRatioValue(), 0.0);
        assertTrue(cfg1.isAdditionalCamerasSuggestPrincipalPointEnabled());
        assertSame(principalPoint, cfg1.getAdditionalCamerasSuggestedPrincipalPointValue());
        assertFalse(cfg1.isHomogeneousPointTriangulatorUsed());
        assertEquals(RobustEstimatorMethod.PROSAC, cfg1.getRobustPointTriangulatorMethod());
        assertEquals(0.8, cfg1.getPointTriangulatorConfidence(), 0.0);
        assertEquals(100, cfg1.getPointTriangulatorMaxIterations());
        assertEquals(1.1, cfg1.getPointTriangulatorThreshold(), 0.0);
        assertSame(data, cfg1.getCalibrationData());
        assertSame(cov, cfg1.getCameraPositionCovariance());
        assertFalse(cfg1.isNotifyAvailableSlamDataEnabled());
        assertFalse(cfg1.isNotifyEstimatedSlamCameraEnabled());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(cfg1);
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg2 =
                SerializationHelper.deserialize(bytes);

        // check
        assertEquals(cfg1.getNonRobustFundamentalMatrixEstimatorMethod(),
                cfg2.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(cfg1.getRobustFundamentalMatrixEstimatorMethod(),
                cfg2.getRobustFundamentalMatrixEstimatorMethod());
        assertEquals(cfg1.isFundamentalMatrixRefined(), cfg2.isFundamentalMatrixRefined());
        assertEquals(cfg1.isFundamentalMatrixCovarianceKept(), cfg2.isFundamentalMatrixCovarianceKept());
        assertEquals(cfg1.getFundamentalMatrixConfidence(),
                cfg2.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(cfg1.getFundamentalMatrixMaxIterations(), cfg2.getFundamentalMatrixMaxIterations());
        assertEquals(cfg1.getFundamentalMatrixThreshold(),
                cfg2.getFundamentalMatrixThreshold(), 0.0);
        assertEquals(cfg1.getFundamentalMatrixComputeAndKeepInliers(),
                cfg2.getFundamentalMatrixComputeAndKeepInliers());
        assertEquals(cfg1.getFundamentalMatrixComputeAndKeepResiduals(),
                cfg2.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(cfg1.getInitialCamerasEstimatorMethod(),
                cfg2.getInitialCamerasEstimatorMethod());
        assertEquals(cfg1.getDaqUseHomogeneousPointTriangulator(),
                cfg2.getDaqUseHomogeneousPointTriangulator());
        assertEquals(cfg1.getInitialCamerasAspectRatio(), cfg2.getInitialCamerasAspectRatio(), 0.0);
        assertEquals(cfg1.getPrincipalPointX(), cfg2.getPrincipalPointX(), 0.0);
        assertEquals(cfg1.getPrincipalPointY(), cfg2.getPrincipalPointY(), 0.0);
        assertEquals(cfg1.getInitialCamerasCorrectorType(), cfg2.getInitialCamerasCorrectorType());
        assertEquals(cfg1.getInitialCamerasMarkValidTriangulatedPoints(),
                cfg2.getInitialCamerasMarkValidTriangulatedPoints());
        assertEquals(cfg1.getInitialIntrinsic1().getInternalMatrix(),
                cfg2.getInitialIntrinsic1().getInternalMatrix());
        assertEquals(cfg1.getInitialIntrinsic2().getInternalMatrix(),
                cfg2.getInitialIntrinsic2().getInternalMatrix());
        assertEquals(cfg1.isGeneralSceneAllowed(), cfg2.isGeneralSceneAllowed());
        assertEquals(cfg1.isPlanarSceneAllowed(), cfg2.isPlanarSceneAllowed());
        assertEquals(cfg1.getRobustPlanarHomographyEstimatorMethod(),
                cfg2.getRobustPlanarHomographyEstimatorMethod());
        assertEquals(cfg1.isPlanarHomographyRefined(), cfg2.isPlanarHomographyRefined());
        assertEquals(cfg1.isPlanarHomographyCovarianceKept(), cfg2.isPlanarHomographyCovarianceKept());
        assertEquals(cfg1.getPlanarHomographyConfidence(),
                cfg2.getPlanarHomographyConfidence(), 0.0);
        assertEquals(cfg1.getPlanarHomographyMaxIterations(), cfg2.getPlanarHomographyMaxIterations());
        assertEquals(cfg1.getPlanarHomographyThreshold(), cfg2.getPlanarHomographyThreshold(), 0.0);
        assertEquals(cfg1.getPlanarHomographyComputeAndKeepInliers(),
                cfg2.getPlanarHomographyComputeAndKeepInliers());
        assertEquals(cfg1.getPlanarHomographyComputeAndKeepResiduals(),
                cfg2.getPlanarHomographyComputeAndKeepResiduals());
        assertEquals(cfg1.getUseDAQForAdditionalCamerasIntrinsics(),
                cfg2.getUseDAQForAdditionalCamerasIntrinsics());
        assertEquals(cfg1.getUseDIACForAdditionalCamerasIntrinsics(),
                cfg2.getUseDIACForAdditionalCamerasIntrinsics());
        assertEquals(cfg1.getAdditionalCamerasIntrinsics().getInternalMatrix(),
                cfg2.getAdditionalCamerasIntrinsics().getInternalMatrix());
        assertEquals(cfg1.getAdditionalCamerasSkewness(), cfg2.getAdditionalCamerasSkewness(), 0.0);
        assertEquals(cfg1.getAdditionalCamerasHorizontalPrincipalPoint(),
                cfg2.getAdditionalCamerasHorizontalPrincipalPoint(), 0.0);
        assertEquals(cfg1.getAdditionalCamerasVerticalPrincipalPoint(),
                cfg2.getAdditionalCamerasVerticalPrincipalPoint(), 0.0);
        assertEquals(cfg1.getAdditionalCamerasAspectRatio(),
                cfg2.getAdditionalCamerasAspectRatio(), 0.0);
        assertEquals(cfg1.getUseEPnPForAdditionalCamerasEstimation(),
                cfg2.getUseEPnPForAdditionalCamerasEstimation());
        assertEquals(cfg1.getUseUPnPForAdditionalCamerasEstimation(),
                cfg2.getUseUPnPForAdditionalCamerasEstimation());
        assertEquals(cfg1.getAdditionalCamerasRobustEstimationMethod(),
                cfg2.getAdditionalCamerasRobustEstimationMethod());
        assertEquals(cfg1.getAdditionalCamerasAllowPlanarConfiguration(),
                cfg2.getAdditionalCamerasAllowPlanarConfiguration());
        assertEquals(cfg1.getAdditionalCamerasAllowNullspaceDimension2(),
                cfg2.getAdditionalCamerasAllowNullspaceDimension2());
        assertEquals(cfg1.getAdditionalCamerasAllowNullspaceDimension3(),
                cfg2.getAdditionalCamerasAllowNullspaceDimension3());
        assertEquals(cfg1.getAdditionalCamerasPlanarThreshold(),
                cfg2.getAdditionalCamerasPlanarThreshold(), 0.0);
        assertEquals(cfg1.areAdditionalCamerasRefined(),
                cfg2.areAdditionalCamerasRefined());
        assertEquals(cfg1.isAdditionalCamerasCovarianceKept(),
                cfg2.isAdditionalCamerasCovarianceKept());
        assertEquals(cfg1.getAdditionalCamerasUseFastRefinement(),
                cfg2.getAdditionalCamerasUseFastRefinement());
        assertEquals(cfg1.getAdditionalCamerasConfidence(),
                cfg2.getAdditionalCamerasConfidence(), 0.0);
        assertEquals(cfg1.getAdditionalCamerasMaxIterations(),
                cfg2.getAdditionalCamerasMaxIterations());
        assertEquals(cfg1.getAdditionalCamerasThreshold(),
                cfg2.getAdditionalCamerasThreshold(), 0.0);
        assertEquals(cfg1.getAdditionalCamerasComputeAndKeepInliers(),
                cfg2.getAdditionalCamerasComputeAndKeepInliers());
        assertEquals(cfg1.getAdditionalCamerasComputeAndKeepResiduals(),
                cfg2.getAdditionalCamerasComputeAndKeepResiduals());
        assertEquals(cfg1.isAdditionalCamerasSuggestSkewnessValueEnabled(),
                cfg2.isAdditionalCamerasSuggestSkewnessValueEnabled());
        assertEquals(cfg1.getAdditionalCamerasSuggestedSkewnessValue(),
                cfg2.getAdditionalCamerasSuggestedSkewnessValue(), 0.0);
        assertEquals(cfg1.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled(),
                cfg2.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled());
        assertEquals(cfg1.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(),
                cfg2.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(cfg1.isAdditionalCamerasSuggestVerticalFocalLengthEnabled(),
                cfg2.isAdditionalCamerasSuggestVerticalFocalLengthEnabled());
        assertEquals(cfg1.getAdditionalCamerasSuggestedVerticalFocalLengthValue(),
                cfg2.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(cfg1.isAdditionalCamerasSuggestAspectRatioEnabled(),
                cfg2.isAdditionalCamerasSuggestAspectRatioEnabled());
        assertEquals(cfg1.getAdditionalCamerasSuggestedAspectRatioValue(),
                cfg2.getAdditionalCamerasSuggestedAspectRatioValue(), 0.0);
        assertEquals(cfg1.isAdditionalCamerasSuggestPrincipalPointEnabled(),
                cfg2.isAdditionalCamerasSuggestPrincipalPointEnabled());
        assertEquals(cfg1.getAdditionalCamerasSuggestedPrincipalPointValue(),
                cfg2.getAdditionalCamerasSuggestedPrincipalPointValue());
        assertEquals(cfg1.isHomogeneousPointTriangulatorUsed(),
                cfg2.isHomogeneousPointTriangulatorUsed());
        assertEquals(cfg1.getRobustPointTriangulatorMethod(),
                cfg2.getRobustPointTriangulatorMethod());
        assertEquals(cfg1.getPointTriangulatorConfidence(),
                cfg2.getPointTriangulatorConfidence(), 0.0);
        assertEquals(cfg1.getPointTriangulatorMaxIterations(),
                cfg2.getPointTriangulatorMaxIterations());
        assertEquals(cfg1.getPointTriangulatorThreshold(),
                cfg2.getPointTriangulatorThreshold(), 0.0);
        assertNotSame(cfg1.getCalibrationData(),
                cfg2.getCalibrationData());
        assertEquals(cfg1.getCameraPositionCovariance(),
                cfg2.getCameraPositionCovariance());
        assertEquals(cfg1.isNotifyAvailableSlamDataEnabled(),
                cfg2.isNotifyAvailableSlamDataEnabled());
        assertEquals(cfg1.isNotifyEstimatedSlamCameraEnabled(),
                cfg2.isNotifyEstimatedSlamCameraEnabled());
    }
}
