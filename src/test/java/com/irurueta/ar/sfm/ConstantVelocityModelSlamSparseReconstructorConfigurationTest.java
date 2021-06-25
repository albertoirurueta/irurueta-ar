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

@SuppressWarnings("ConstantConditions")
public class ConstantVelocityModelSlamSparseReconstructorConfigurationTest {

    @Test
    public void testConstructor() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getInitialCamerasAspectRatio(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO,
                0.0);
        assertEquals(cfg.getPrincipalPointX(), 0.0, 0.0);
        assertEquals(cfg.getPrincipalPointY(), 0.0, 0.0);
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getInitialCamerasMarkValidTriangulatedPoints(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(cfg.getInitialIntrinsic1());
        assertNull(cfg.getInitialIntrinsic2());
        assertEquals(cfg.isGeneralSceneAllowed(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE,
                0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD,
                0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getUseDAQForAdditionalCamerasIntrinsics(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
        assertEquals(cfg.getUseDIACForAdditionalCamerasIntrinsics(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
        assertNull(cfg.getAdditionalCamerasIntrinsics());
        assertEquals(cfg.getAdditionalCamerasSkewness(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS,
                0.0);
        assertEquals(cfg.getAdditionalCamerasHorizontalPrincipalPoint(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT, 0.0);
        assertEquals(cfg.getAdditionalCamerasVerticalPrincipalPoint(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT, 0.0);
        assertEquals(cfg.getAdditionalCamerasAspectRatio(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_ASPECT_RATIO,
                0.0);
        assertEquals(cfg.getUseEPnPForAdditionalCamerasEstimation(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
        assertEquals(cfg.getUseUPnPForAdditionalCamerasEstimation(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
        assertEquals(cfg.getAdditionalCamerasRobustEstimationMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD);
        assertEquals(cfg.getAdditionalCamerasAllowPlanarConfiguration(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION);
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension2(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2);
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension3(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3);
        assertEquals(cfg.getAdditionalCamerasPlanarThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD,
                0.0);
        assertEquals(cfg.areAdditionalCamerasRefined(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_ADDITIONAL_CAMERAS);
        assertEquals(cfg.isAdditionalCamerasCovarianceKept(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS);
        assertEquals(cfg.getAdditionalCamerasUseFastRefinement(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT);
        assertEquals(cfg.getAdditionalCamerasConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE,
                0.0);
        assertEquals(cfg.getAdditionalCamerasMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS);
        assertEquals(cfg.getAdditionalCamerasThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_THRESHOLD, 0.0);
        assertEquals(cfg.getAdditionalCamerasComputeAndKeepInliers(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getAdditionalCamerasComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.isAdditionalCamerasSuggestSkewnessValueEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedSkewnessValue(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(), 0.0, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 0.0, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestAspectRatioEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedAspectRatioValue(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestPrincipalPointEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(cfg.getAdditionalCamerasSuggestedPrincipalPointValue());
        assertEquals(cfg.isHomogeneousPointTriangulatorUsed(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getRobustPointTriangulatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD);
        assertEquals(cfg.getPointTriangulatorConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_CONFIDENCE,
                0.0);
        assertEquals(cfg.getPointTriangulatorMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS);
        assertEquals(cfg.getPointTriangulatorThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_THRESHOLD,
                0.0);
        assertNull(cfg.getCalibrationData());
        assertNotNull(cfg.getCameraPositionCovariance());
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);
    }

    @Test
    public void testMake() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                ConstantVelocityModelSlamSparseReconstructorConfiguration.make();

        // check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getInitialCamerasAspectRatio(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO,
                0.0);
        assertEquals(cfg.getPrincipalPointX(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_X,
                0.0);
        assertEquals(cfg.getPrincipalPointY(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_Y,
                0.0);
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getInitialCamerasMarkValidTriangulatedPoints(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertNull(cfg.getInitialIntrinsic1());
        assertNull(cfg.getInitialIntrinsic2());
        assertEquals(cfg.isGeneralSceneAllowed(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE,
                0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD,
                0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getUseDAQForAdditionalCamerasIntrinsics(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
        assertEquals(cfg.getUseDIACForAdditionalCamerasIntrinsics(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
        assertNull(cfg.getAdditionalCamerasIntrinsics());
        assertEquals(cfg.getAdditionalCamerasSkewness(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS,
                0.0);
        assertEquals(cfg.getAdditionalCamerasHorizontalPrincipalPoint(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT, 0.0);
        assertEquals(cfg.getAdditionalCamerasVerticalPrincipalPoint(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT, 0.0);
        assertEquals(cfg.getAdditionalCamerasAspectRatio(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_ASPECT_RATIO,
                0.0);
        assertEquals(cfg.getUseEPnPForAdditionalCamerasEstimation(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
        assertEquals(cfg.getUseUPnPForAdditionalCamerasEstimation(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
        assertEquals(cfg.getAdditionalCamerasRobustEstimationMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD);
        assertEquals(cfg.getAdditionalCamerasAllowPlanarConfiguration(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION);
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension2(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2);
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension3(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3);
        assertEquals(cfg.getAdditionalCamerasPlanarThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD,
                0.0);
        assertEquals(cfg.areAdditionalCamerasRefined(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_ADDITIONAL_CAMERAS);
        assertEquals(cfg.isAdditionalCamerasCovarianceKept(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS);
        assertEquals(cfg.getAdditionalCamerasUseFastRefinement(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT);
        assertEquals(cfg.getAdditionalCamerasConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE,
                0.0);
        assertEquals(cfg.getAdditionalCamerasMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS);
        assertEquals(cfg.getAdditionalCamerasThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_THRESHOLD, 0.0);
        assertEquals(cfg.getAdditionalCamerasComputeAndKeepInliers(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getAdditionalCamerasComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.isAdditionalCamerasSuggestSkewnessValueEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedSkewnessValue(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(), 0.0, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 0.0, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestAspectRatioEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(cfg.getAdditionalCamerasSuggestedAspectRatioValue(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE, 0.0);
        assertEquals(cfg.isAdditionalCamerasSuggestPrincipalPointEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(cfg.getAdditionalCamerasSuggestedPrincipalPointValue());
        assertEquals(cfg.isHomogeneousPointTriangulatorUsed(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getRobustPointTriangulatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD);
        assertEquals(cfg.getPointTriangulatorConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_CONFIDENCE,
                0.0);
        assertEquals(cfg.getPointTriangulatorMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS);
        assertEquals(cfg.getPointTriangulatorThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_THRESHOLD,
                0.0);
        assertNull(cfg.getCalibrationData());
        assertNotNull(cfg.getCameraPositionCovariance());
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);
    }

    @Test
    public void testGetSetNonRobustFundamentalMatrixEstimatorMethod() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);

        // set new value
        assertSame(cfg.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM), cfg);

        // check correctness
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
    }

    @Test
    public void testGetSetRobustFundamentalMatrixEstimatorMethod() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);

        // set new value
        assertSame(cfg.setRobustFundamentalMatrixEstimatorMethod(
                RobustEstimatorMethod.LMedS), cfg);

        // check correctness
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testIsSetFundamentalMatrixRefined() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isFundamentalMatrixRefined(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);

        // set new value
        assertSame(cfg.setFundamentalMatrixRefined(false), cfg);

        // check correctness
        assertFalse(cfg.isFundamentalMatrixRefined());
    }

    @Test
    public void testIsSetFundamentalMatrixCovarianceKept() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);

        // set new value
        assertSame(cfg.setFundamentalMatrixCovarianceKept(true), cfg);

        // check correctness
        assertTrue(cfg.isFundamentalMatrixCovarianceKept());
    }

    @Test
    public void testGetSetFundamentalMatrixConfidence() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                0.0);

        // set new value
        assertSame(cfg.setFundamentalMatrixConfidence(0.7), cfg);

        // check correctness
        assertEquals(cfg.getFundamentalMatrixConfidence(), 0.7, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixMaxIterations() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);

        // set new value
        assertSame(cfg.setFundamentalMatrixMaxIterations(10), cfg);

        // check correctness
        assertEquals(cfg.getFundamentalMatrixMaxIterations(), 10);
    }

    @Test
    public void testGetSetFundamentalMatrixThreshold() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                0.0);

        // set new value
        assertSame(cfg.setFundamentalMatrixThreshold(2.0), cfg);

        // check correctness
        assertEquals(cfg.getFundamentalMatrixThreshold(), 2.0, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepInliers() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);

        // set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepInliers(false), cfg);

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepInliers());
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepResiduals() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);

        // set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepResiduals(false), cfg);

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetInitialCamerasEstimatorMethod() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD);

        // set new value
        assertSame(cfg.setInitialCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC),
                cfg);

        // check correctness
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);
    }

    @Test
    public void testGetSetDaqUseHomogeneousPointTriangulator() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);

        // set new value
        assertSame(cfg.setDaqUseHomogeneousPointTriangulator(false), cfg);

        // check correctness
        assertFalse(cfg.getDaqUseHomogeneousPointTriangulator());
    }

    @Test
    public void testGetSetInitialCamerasAspectRatio() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getInitialCamerasAspectRatio(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO,
                0.0);

        // set new value
        assertSame(cfg.setInitialCamerasAspectRatio(0.5), cfg);

        // check correctness
        assertEquals(cfg.getInitialCamerasAspectRatio(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointX() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPrincipalPointX(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_X,
                0.0);

        // set new value
        assertSame(cfg.setPrincipalPointX(10.0), cfg);

        // check correctness
        assertEquals(cfg.getPrincipalPointX(), 10.0, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointY() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPrincipalPointY(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_Y,
                0.0);

        // set new value
        assertSame(cfg.setPrincipalPointY(10.0), cfg);

        // check correctness
        assertEquals(cfg.getPrincipalPointY(), 10.0, 0.0);
    }

    @Test
    public void testGetSetInitialCamerasCorrectorType() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE);

        // set new value
        assertSame(cfg.setInitialCamerasCorrectorType(
                CorrectorType.GOLD_STANDARD), cfg);

        // check correctness
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                CorrectorType.GOLD_STANDARD);
    }

    @Test
    public void testGetSetInitialCamerasMarkValidTriangulatedPoints() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getInitialCamerasMarkValidTriangulatedPoints(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);

        // set new value
        assertSame(cfg.setInitialCamerasMarkValidTriangulatedPoints(false),
                cfg);

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
        assertSame(cfg.setInitialIntrinsic1(intrinsic), cfg);

        // check correctness
        assertSame(cfg.getInitialIntrinsic1(), intrinsic);
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
        assertSame(cfg.setInitialIntrinsic2(intrinsic), cfg);

        // check correctness
        assertSame(cfg.getInitialIntrinsic2(), intrinsic);
    }

    @Test
    public void testIsSetGeneralSceneAllowed() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isGeneralSceneAllowed(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE);

        // set new value
        assertSame(cfg.setGeneralSceneAllowed(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE), cfg);

        // check correctness
        assertEquals(cfg.isGeneralSceneAllowed(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE);
    }

    @Test
    public void testIsSetPlanarSceneAllowed() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isPlanarSceneAllowed(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE);

        // set new value
        assertSame(cfg.setPlanarSceneAllowed(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE), cfg);

        // check correctness
        assertEquals(cfg.isPlanarSceneAllowed(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE);
    }

    @Test
    public void testGetSetRobustPlanarHomographyEstimatorMethod() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);

        // set new value
        assertSame(cfg.setRobustPlanarHomographyEstimatorMethod(
                RobustEstimatorMethod.RANSAC), cfg);

        // check correctness
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                RobustEstimatorMethod.RANSAC);
    }

    @Test
    public void testIsSetPlanarHomographyRefined() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isPlanarHomographyRefined(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY);

        // set new value
        assertSame(cfg.setPlanarHomographyRefined(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY), cfg);

        // check correctness
        assertEquals(cfg.isPlanarHomographyRefined(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
    }

    @Test
    public void testIsSetPlanarHomographyCovarianceKept() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);

        // set new value
        assertSame(cfg.setPlanarHomographyCovarianceKept(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE),
                cfg);

        // check correctness
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
    }

    @Test
    public void testGetSetPlanarHomographyConfidence() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE,
                0.0);

        // set new value
        assertSame(cfg.setPlanarHomographyConfidence(0.5), cfg);

        // check correctness
        assertEquals(cfg.getPlanarHomographyConfidence(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyMaxIterations() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);

        // set new value
        assertSame(cfg.setPlanarHomographyMaxIterations(100), cfg);

        // check correctness
        assertEquals(cfg.getPlanarHomographyMaxIterations(), 100);
    }

    @Test
    public void testGetSetPlanarHomographyThreshold() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD,
                0.0);

        // set new value
        assertSame(cfg.setPlanarHomographyThreshold(0.5), cfg);

        // check correctness
        assertEquals(cfg.getPlanarHomographyThreshold(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepInliers() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);

        // set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepInliers(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS), cfg);

        // check correctness
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepResiduals() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);

        // set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepResiduals(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS), cfg);

        // check correctness
        assertEquals(!cfg.getPlanarHomographyComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testGetSetUseDAQForAdditionalCamerasIntrinsics() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getUseDAQForAdditionalCamerasIntrinsics(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS);

        // set new value
        assertSame(cfg.setUseDAQForAdditionalCamerasIntrinics(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS), cfg);

        // check correctness
        assertEquals(!cfg.getUseDAQForAdditionalCamerasIntrinsics(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
    }

    @Test
    public void testGetSetUseDIACForAdditionalCamerasIntrinsics() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getUseDIACForAdditionalCamerasIntrinsics(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS);

        // set new value
        assertSame(cfg.setUseDIACForAdditionalCamerasIntrinsics(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS), cfg);

        // check correctness
        assertEquals(!cfg.getUseDIACForAdditionalCamerasIntrinsics(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
    }

    @Test
    public void testGetSetAdditionalCamerasIntrinsics() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertNull(cfg.getAdditionalCamerasIntrinsics());

        // set new value
        final PinholeCameraIntrinsicParameters intrinsics = new PinholeCameraIntrinsicParameters();
        assertSame(cfg.setAdditionalCamerasIntrinsics(intrinsics), cfg);

        // check correctness
        assertSame(cfg.getAdditionalCamerasIntrinsics(), intrinsics);
    }

    @Test
    public void testGetSetAdditionalCamerasSkewness() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasSkewness(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS, 0.0);

        // set new value
        assertSame(cfg.setAdditionalCamerasSkewness(1e-3), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasSkewness(), 1e-3, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasHorizontalPrincipalPoint() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasHorizontalPrincipalPoint(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT, 0.0);

        // set new value
        assertSame(cfg.setAdditionalCamerasHorizontalPrincipalPoint(320), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasHorizontalPrincipalPoint(), 320, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasVerticalPrincipalPoint() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasVerticalPrincipalPoint(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT, 0.0);

        // set new value
        assertSame(cfg.setAdditionalCamerasVerticalPrincipalPoint(240), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasVerticalPrincipalPoint(), 240, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasAspectRatio() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasAspectRatio(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_ASPECT_RATIO,
                0.0);

        // set new value
        assertSame(cfg.setAdditionalCamerasAspectRatio(-1.0), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasAspectRatio(), -1.0, 0.0);
    }

    @Test
    public void testGetSetUseEPnPForAdditionalCamerasEstimation() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getUseEPnPForAdditionalCamerasEstimation(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);

        // set new value
        assertSame(cfg.setUseEPnPForAdditionalCamerasEstimation(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION), cfg);

        // check correctness
        assertEquals(cfg.getUseEPnPForAdditionalCamerasEstimation(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
    }

    @Test
    public void testGetSetUseUPnPForAdditionalCamerasEstimation() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getUseUPnPForAdditionalCamerasEstimation(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);

        // set new value
        assertSame(cfg.setUseUPnPForAdditionalCamerasEstimation(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION), cfg);

        // check correctness
        assertEquals(cfg.getUseUPnPForAdditionalCamerasEstimation(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
    }

    @Test
    public void testGetSetAdditionalCamerasRobustEstimationMethod() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasRobustEstimationMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD);

        // set new value
        assertSame(cfg.setAdditionalCamerasRobustEstimationMethod(
                RobustEstimatorMethod.LMedS), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasRobustEstimationMethod(),
                RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testGetSetAdditionalCamerasAllowPlanarConfiguration() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasAllowPlanarConfiguration(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION);

        // set new value
        assertSame(cfg.setAdditionalCamerasAllowPlanarConfiguration(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasAllowPlanarConfiguration(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION);
    }

    @Test
    public void testGetSetAdditionalCamerasAllowNullspaceDimension2() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension2(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2);

        // set new value
        assertSame(cfg.setAdditionalCamerasAllowNullspaceDimension2(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension2(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2);
    }

    @Test
    public void testGetSetAdditionalCamerasAllowNullspaceDimension3() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension3(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3);

        // set new value
        assertSame(cfg.setAdditionalCamerasAllowNullspaceDimension3(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension3(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3);
    }

    @Test
    public void testGetSetAdditionalCamerasPlanarThreshold() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasPlanarThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD,
                0.0);

        // set new value
        assertSame(cfg.setAdditionalCamerasPlanarThreshold(1e-3), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasPlanarThreshold(), 1e-3, 0.0);
    }

    @Test
    public void testAreSetAdditionalCamerasRefined() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.areAdditionalCamerasRefined(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_ADDITIONAL_CAMERAS);

        // set new value
        assertSame(cfg.setAdditionalCamerasRefined(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_ADDITIONAL_CAMERAS), cfg);

        // check correctness
        assertEquals(cfg.areAdditionalCamerasRefined(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_ADDITIONAL_CAMERAS);
    }

    @Test
    public void testIsSetAdditionalCamerasCovarianceKept() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isAdditionalCamerasCovarianceKept(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS);

        // set new value
        assertSame(cfg.setAdditionalCamerasCovarianceKept(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS),
                cfg);

        // check correctness
        assertEquals(cfg.isAdditionalCamerasCovarianceKept(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS);
    }

    @Test
    public void testGetSetAdditionalCamerasUseFastRefinement() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasUseFastRefinement(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT);

        // set new value
        assertSame(cfg.setAdditionalCamerasUseFastRefinement(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasUseFastRefinement(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT);
    }

    @Test
    public void testGetSetAdditionalCamerasConfidence() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE, 0.0);

        // set new value
        assertSame(cfg.setAdditionalCamerasConfidence(0.8), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasConfidence(), 0.8, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasMaxIterations() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS);

        // set new value
        assertSame(cfg.setAdditionalCamerasMaxIterations(100), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasMaxIterations(), 100);
    }

    @Test
    public void testGetSetAdditionalCamerasThreshold() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_THRESHOLD, 0.0);

        // set new value
        assertSame(cfg.setAdditionalCamerasThreshold(2.0), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasThreshold(), 2.0, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasComputeAndKeepInliers() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasComputeAndKeepInliers(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS);

        // set new value
        assertSame(cfg.setAdditionalCamerasComputeAndKeepInliers(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasComputeAndKeepInliers(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS);
    }

    @Test
    public void testGetSetAdditionalCamerasComputeAndKeepResiduals() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS);

        // set new value
        assertSame(cfg.setAdditionalCamerasComputeAndKeepResiduals(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasComputeAndKeepResiduals(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestSkewnessValueEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isAdditionalCamerasSuggestSkewnessValueEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED);

        // set new value
        assertSame(cfg.setAdditionalCamerasSuggestSkewnessValueEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED), cfg);

        // check correctness
        assertEquals(cfg.isAdditionalCamerasSuggestSkewnessValueEnabled(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedSkewnessValue() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasSuggestedSkewnessValue(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE, 0.0);

        // set new value
        assertSame(cfg.setAdditionalCamerasSuggestedSkewnessValue(1e-3), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasSuggestedSkewnessValue(), 1e-3, 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestHorizontalFocalLengthEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);

        // set new value
        assertSame(cfg.setAdditionalCamerasSuggestHorizontalFocalLengthEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED), cfg);

        // check correctness
        assertEquals(cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedHorizontalFocalLengthValue() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(), 0.0, 0.0);

        // set new value
        assertSame(cfg.setAdditionalCamerasSuggestedHorizontalFocalLengthValue(320), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(), 320, 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestVerticalFocalLengthEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);

        // set new value
        assertSame(cfg.setAdditionalCamerasSuggestVerticalFocalLengthEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED),
                cfg);

        // check correctness
        assertEquals(cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedVerticalFocalLengthValue() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 0.0, 0.0);

        // set new value
        assertSame(cfg.setAdditionalCamerasSuggestedVerticalFocalLengthValue(240), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 240, 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestAspectRatioEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isAdditionalCamerasSuggestAspectRatioEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED);

        // set new value
        assertSame(cfg.setAdditionalCamerasSuggestAspectRatioEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED), cfg);

        // check correctness
        assertEquals(cfg.isAdditionalCamerasSuggestAspectRatioEnabled(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedAspectRatioValue() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getAdditionalCamerasSuggestedAspectRatioValue(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE, 0.0);

        // set new value
        assertSame(cfg.setAdditionalCamerasSuggestedAspectRatioValue(1.1), cfg);

        // check correctness
        assertEquals(cfg.getAdditionalCamerasSuggestedAspectRatioValue(), 1.1, 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestPrincipalPointEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isAdditionalCamerasSuggestPrincipalPointEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED);

        // set new value
        assertSame(cfg.setAdditionalCamerasSuggestPrincipalPointEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED), cfg);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedPrincipalPointValue() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertNull(cfg.getAdditionalCamerasSuggestedPrincipalPointValue());

        // set new value
        final InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D();
        assertSame(cfg.setAdditionalCamerasSuggestedPrincipalPointValue(principalPoint), cfg);

        // check correctness
        assertSame(cfg.getAdditionalCamerasSuggestedPrincipalPointValue(), principalPoint);
    }

    @Test
    public void testIsSetHomogeneousPointTriangulatorUsed() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isHomogeneousPointTriangulatorUsed(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR);

        // set new value
        assertSame(cfg.setHomogeneousPointTriangulatorUsed(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR),
                cfg);

        // check correctness
        assertEquals(cfg.isHomogeneousPointTriangulatorUsed(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
    }

    @Test
    public void testGetSetRobustPointTriangulatorMethod() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getRobustPointTriangulatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD);

        // set new value
        assertSame(cfg.setRobustPointTriangulatorMethod(RobustEstimatorMethod.MSAC), cfg);

        // check correctness
        assertEquals(cfg.getRobustPointTriangulatorMethod(), RobustEstimatorMethod.MSAC);
    }

    @Test
    public void testGetSetPointTriangulatorConfidence() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPointTriangulatorConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_CONFIDENCE, 0.0);

        // set new value
        assertSame(cfg.setPointTriangulatorConfidence(0.8), cfg);

        // check correctness
        assertEquals(cfg.getPointTriangulatorConfidence(), 0.8, 0.0);
    }

    @Test
    public void testGetSetPointTriangulatorMaxIterations() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPointTriangulatorMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS);

        // set new value
        assertSame(cfg.setPointTriangulatorMaxIterations(100), cfg);

        // check correctness
        assertEquals(cfg.getPointTriangulatorMaxIterations(), 100);
    }

    @Test
    public void testGetStPointTriangulatorThreshold() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPointTriangulatorThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_THRESHOLD, 0.0);

        // set new value
        assertSame(cfg.setPointTriangulatorThreshold(1e-3), cfg);

        // check correctness
        assertEquals(cfg.getPointTriangulatorThreshold(), 1e-3, 0.0);
    }

    @Test
    public void testGetSetCalibrationData() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertNull(cfg.getCalibrationData());

        // set new value
        final ConstantVelocityModelSlamCalibrationData calibrationData = new ConstantVelocityModelSlamCalibrationData();
        assertSame(cfg.setCalibrationData(calibrationData), cfg);

        // check correctness
        assertSame(cfg.getCalibrationData(), calibrationData);
    }

    @Test
    public void testGetSetCameraPositionVariance() throws AlgebraException {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertNotNull(cfg.getCameraPositionCovariance());

        // set new value
        final Matrix cov = new Matrix(3, 3);
        assertSame(cfg.setCameraPositionCovariance(cov), cfg);

        // check correctness
        assertSame(cfg.getCameraPositionCovariance(), cov);
    }

    @Test
    public void testSetCameraPositionVariance() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertNotNull(cfg.getCameraPositionCovariance());

        // set new value
        final Matrix cov = Matrix.diagonal(new double[]{2.0, 2.0, 2.0});
        assertSame(cfg.setCameraPositionVariance(2.0), cfg);

        // check correctness
        assertEquals(cfg.getCameraPositionCovariance(), cov);
    }

    @Test
    public void testIsSetNotifyAvailableSlamDataEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);

        // set new value
        assertSame(cfg.setNotifyAvailableSlamDataEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE), cfg);

        // check correctness
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
    }

    @Test
    public void testIsSetNotifyEstimatedSlamCameraEnabled() {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);

        // set new value
        assertSame(cfg.setNotifyEstimatedSlamCameraEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA), cfg);

        // check correctness
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, WrongSizeException, ClassNotFoundException {
        final ConstantVelocityModelSlamSparseReconstructorConfiguration cfg1 =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        // set new values
        cfg1.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        cfg1.setRobustFundamentalMatrixEstimatorMethod(
                RobustEstimatorMethod.RANSAC);
        cfg1.setFundamentalMatrixRefined(false);
        cfg1.setFundamentalMatrixCovarianceKept(true);
        cfg1.setFundamentalMatrixConfidence(0.8);
        cfg1.setFundamentalMatrixMaxIterations(500);
        cfg1.setFundamentalMatrixThreshold(0.5);
        cfg1.setFundamentalMatrixComputeAndKeepInliers(false);
        cfg1.setFundamentalMatrixComputeAndKeepResiduals(false);
        cfg1.setInitialCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);
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
        final PinholeCameraIntrinsicParameters additionalIntrinsics = new PinholeCameraIntrinsicParameters();
        cfg1.setAdditionalCamerasIntrinsics(additionalIntrinsics);
        cfg1.setAdditionalCamerasSkewness(1e-4);
        cfg1.setAdditionalCamerasHorizontalPrincipalPoint(0.5);
        cfg1.setAdditionalCamerasVerticalPrincipalPoint(-0.5);
        cfg1.setAdditionalCamerasAspectRatio(0.99);
        cfg1.setUseEPnPForAdditionalCamerasEstimation(true);
        cfg1.setUseUPnPForAdditionalCamerasEstimation(false);
        cfg1.setAdditionalCamerasRobustEstimationMethod(
                RobustEstimatorMethod.LMedS);
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
        final ConstantVelocityModelSlamCalibrationData data = new ConstantVelocityModelSlamCalibrationData();
        cfg1.setCalibrationData(data);
        final Matrix cov = Matrix.identity(3, 3);
        cfg1.setCameraPositionCovariance(cov);
        cfg1.setNotifyAvailableSlamDataEnabled(false);
        cfg1.setNotifyEstimatedSlamCameraEnabled(false);

        // check
        assertEquals(cfg1.getNonRobustFundamentalMatrixEstimatorMethod(),
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        assertEquals(cfg1.getRobustFundamentalMatrixEstimatorMethod(),
                RobustEstimatorMethod.RANSAC);
        assertFalse(cfg1.isFundamentalMatrixRefined());
        assertTrue(cfg1.isFundamentalMatrixCovarianceKept());
        assertEquals(cfg1.getFundamentalMatrixConfidence(), 0.8, 0.0);
        assertEquals(cfg1.getFundamentalMatrixMaxIterations(), 500);
        assertEquals(cfg1.getFundamentalMatrixThreshold(), 0.5, 0.0);
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepInliers());
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(cfg1.getInitialCamerasEstimatorMethod(),
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);
        assertFalse(cfg1.getDaqUseHomogeneousPointTriangulator());
        assertEquals(cfg1.getInitialCamerasAspectRatio(), 0.99, 0.0);
        assertEquals(cfg1.getPrincipalPointX(), 1e-3, 0.0);
        assertEquals(cfg1.getPrincipalPointY(), -1e-3, 0.0);
        assertEquals(cfg1.getInitialCamerasCorrectorType(),
                CorrectorType.GOLD_STANDARD);
        assertFalse(cfg1.getInitialCamerasMarkValidTriangulatedPoints());
        assertSame(intrinsic1, cfg1.getInitialIntrinsic1());
        assertSame(intrinsic2, cfg1.getInitialIntrinsic2());
        assertFalse(cfg1.isGeneralSceneAllowed());
        assertFalse(cfg1.isPlanarSceneAllowed());
        assertEquals(RobustEstimatorMethod.RANSAC,
                cfg1.getRobustPlanarHomographyEstimatorMethod());
        assertFalse(cfg1.isPlanarHomographyRefined());
        assertTrue(cfg1.isPlanarHomographyCovarianceKept());
        assertEquals(cfg1.getPlanarHomographyConfidence(), 0.9, 0.0);
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
        assertEquals(RobustEstimatorMethod.LMedS,
                cfg1.getAdditionalCamerasRobustEstimationMethod());
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
        assertEquals(1.0, cfg1.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(), 0.0);
        assertTrue(cfg1.isAdditionalCamerasSuggestVerticalFocalLengthEnabled());
        assertEquals(-1.0, cfg1.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 0.0);
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
        assertEquals(cfg1.isFundamentalMatrixRefined(),
                cfg2.isFundamentalMatrixRefined());
        assertEquals(cfg1.isFundamentalMatrixCovarianceKept(),
                cfg2.isFundamentalMatrixCovarianceKept());
        assertEquals(cfg1.getFundamentalMatrixConfidence(),
                cfg2.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(cfg1.getFundamentalMatrixMaxIterations(),
                cfg2.getFundamentalMatrixMaxIterations());
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
        assertEquals(cfg1.getInitialCamerasAspectRatio(),
                cfg2.getInitialCamerasAspectRatio(), 0.0);
        assertEquals(cfg1.getPrincipalPointX(),
                cfg2.getPrincipalPointX(), 0.0);
        assertEquals(cfg1.getPrincipalPointY(),
                cfg2.getPrincipalPointY(), 0.0);
        assertEquals(cfg1.getInitialCamerasCorrectorType(),
                cfg2.getInitialCamerasCorrectorType());
        assertEquals(cfg1.getInitialCamerasMarkValidTriangulatedPoints(),
                cfg2.getInitialCamerasMarkValidTriangulatedPoints());
        assertEquals(cfg1.getInitialIntrinsic1().getInternalMatrix(),
                cfg2.getInitialIntrinsic1().getInternalMatrix());
        assertEquals(cfg1.getInitialIntrinsic2().getInternalMatrix(),
                cfg2.getInitialIntrinsic2().getInternalMatrix());
        assertEquals(cfg1.isGeneralSceneAllowed(),
                cfg2.isGeneralSceneAllowed());
        assertEquals(cfg1.isPlanarSceneAllowed(),
                cfg2.isPlanarSceneAllowed());
        assertEquals(cfg1.getRobustPlanarHomographyEstimatorMethod(),
                cfg2.getRobustPlanarHomographyEstimatorMethod());
        assertEquals(cfg1.isPlanarHomographyRefined(),
                cfg2.isPlanarHomographyRefined());
        assertEquals(cfg1.isPlanarHomographyCovarianceKept(),
                cfg2.isPlanarHomographyCovarianceKept());
        assertEquals(cfg1.getPlanarHomographyConfidence(),
                cfg2.getPlanarHomographyConfidence(), 0.0);
        assertEquals(cfg1.getPlanarHomographyMaxIterations(),
                cfg2.getPlanarHomographyMaxIterations());
        assertEquals(cfg1.getPlanarHomographyThreshold(),
                cfg2.getPlanarHomographyThreshold(), 0.0);
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
        assertEquals(cfg1.getAdditionalCamerasSkewness(),
                cfg2.getAdditionalCamerasSkewness(), 0.0);
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
