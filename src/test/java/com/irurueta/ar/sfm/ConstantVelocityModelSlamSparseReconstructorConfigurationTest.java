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
import com.irurueta.ar.epipolar.CorrectorType;
import com.irurueta.ar.epipolar.estimators.FundamentalMatrixEstimatorMethod;
import com.irurueta.ar.slam.ConstantVelocityModelSlamCalibrationData;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.*;

import static org.junit.Assert.*;

public class ConstantVelocityModelSlamSparseReconstructorConfigurationTest {

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default values
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
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                ConstantVelocityModelSlamSparseReconstructorConfiguration.make();

        //check default values
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
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);

        //set new value
        assertSame(cfg.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM), cfg);

        //check correctness
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
    }

    @Test
    public void testGetSetRobustFundamentalMatrixEstimatorMethod() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);

        //set new value
        assertSame(cfg.setRobustFundamentalMatrixEstimatorMethod(
                RobustEstimatorMethod.LMedS), cfg);

        //check correctness
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testIsSetFundamentalMatrixRefined() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isFundamentalMatrixRefined(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);

        //set new value
        assertSame(cfg.setFundamentalMatrixRefined(false), cfg);

        //check correctness
        assertFalse(cfg.isFundamentalMatrixRefined());
    }

    @Test
    public void testIsSetFundamentalMatrixCovarianceKept() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);

        //set new value
        assertSame(cfg.setFundamentalMatrixCovarianceKept(true), cfg);

        //check correctness
        assertTrue(cfg.isFundamentalMatrixCovarianceKept());
    }

    @Test
    public void testGetSetFundamentalMatrixConfidence() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                0.0);

        //set new value
        assertSame(cfg.setFundamentalMatrixConfidence(0.7), cfg);

        //check correctness
        assertEquals(cfg.getFundamentalMatrixConfidence(), 0.7, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixMaxIterations() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);

        //set new value
        assertSame(cfg.setFundamentalMatrixMaxIterations(10), cfg);

        //check correctness
        assertEquals(cfg.getFundamentalMatrixMaxIterations(), 10);
    }

    @Test
    public void testGetSetFundamentalMatrixThreshold() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                0.0);

        //set new value
        assertSame(cfg.setFundamentalMatrixThreshold(2.0), cfg);

        //check correctness
        assertEquals(cfg.getFundamentalMatrixThreshold(), 2.0, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepInliers() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);

        //set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepInliers(false), cfg);

        //check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepInliers());
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepResiduals() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);

        //set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepResiduals(false), cfg);

        //check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetInitialCamerasEstimatorMethod() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD);

        //set new value
        assertSame(cfg.setInitialCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC),
                cfg);

        //check correctness
        assertEquals(cfg.getInitialCamerasEstimatorMethod(),
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);
    }

    @Test
    public void testGetSetDaqUseHomogeneousPointTriangulator() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);

        //set new value
        assertSame(cfg.setDaqUseHomogeneousPointTriangulator(false), cfg);

        //check correctness
        assertFalse(cfg.getDaqUseHomogeneousPointTriangulator());
    }

    @Test
    public void testGetSetInitialCamerasAspectRatio() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getInitialCamerasAspectRatio(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO,
                0.0);

        //set new value
        assertSame(cfg.setInitialCamerasAspectRatio(0.5), cfg);

        //check correctness
        assertEquals(cfg.getInitialCamerasAspectRatio(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointX() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPrincipalPointX(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_X,
                0.0);

        //set new value
        assertSame(cfg.setPrincipalPointX(10.0), cfg);

        //check correctness
        assertEquals(cfg.getPrincipalPointX(), 10.0, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointY() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPrincipalPointY(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_Y,
                0.0);

        //set new value
        assertSame(cfg.setPrincipalPointY(10.0), cfg);

        //check correctness
        assertEquals(cfg.getPrincipalPointY(), 10.0, 0.0);
    }

    @Test
    public void testGetSetInitialCamerasCorrectorType() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE);

        //set new value
        assertSame(cfg.setInitialCamerasCorrectorType(
                CorrectorType.GOLD_STANDARD), cfg);

        //check correctness
        assertEquals(cfg.getInitialCamerasCorrectorType(),
                CorrectorType.GOLD_STANDARD);
    }

    @Test
    public void testGetSetInitialCamerasMarkValidTriangulatedPoints() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getInitialCamerasMarkValidTriangulatedPoints(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);

        //set new value
        assertSame(cfg.setInitialCamerasMarkValidTriangulatedPoints(false),
                cfg);

        //check correctness
        assertFalse(cfg.getInitialCamerasMarkValidTriangulatedPoints());
    }

    @Test
    public void testGetSetInitialIntrinsic1() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertNull(cfg.getInitialIntrinsic1());

        //set new value
        PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters();
        assertSame(cfg.setInitialIntrinsic1(intrinsic), cfg);

        //check correctness
        assertSame(cfg.getInitialIntrinsic1(), intrinsic);
    }

    @Test
    public void testGetSetInitialIntrinsic2() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertNull(cfg.getInitialIntrinsic2());

        //set new value
        PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters();
        assertSame(cfg.setInitialIntrinsic2(intrinsic), cfg);

        //check correctness
        assertSame(cfg.getInitialIntrinsic2(), intrinsic);
    }

    @Test
    public void testIsSetGeneralSceneAllowed() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isGeneralSceneAllowed(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE);

        //set new value
        assertSame(cfg.setGeneralSceneAllowed(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE), cfg);

        //check correctness
        assertEquals(cfg.isGeneralSceneAllowed(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE);
    }

    @Test
    public void testIsSetPlanarSceneAllowed() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isPlanarSceneAllowed(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE);

        //set new value
        assertSame(cfg.setPlanarSceneAllowed(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE), cfg);

        //check correctness
        assertEquals(cfg.isPlanarSceneAllowed(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE);
    }

    @Test
    public void testGetSetRobustPlanarHomographyEstimatorMethod() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);

        //set new value
        assertSame(cfg.setRobustPlanarHomographyEstimatorMethod(
                RobustEstimatorMethod.RANSAC), cfg);

        //check correctness
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                RobustEstimatorMethod.RANSAC);
    }

    @Test
    public void testIsSetPlanarHomographyRefined() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isPlanarHomographyRefined(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY);

        //set new value
        assertSame(cfg.setPlanarHomographyRefined(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY), cfg);

        //check correctness
        assertEquals(cfg.isPlanarHomographyRefined(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
    }

    @Test
    public void testIsSetPlanarHomographyCovarianceKept() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);

        //set new value
        assertSame(cfg.setPlanarHomographyCovarianceKept(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE),
                cfg);

        //check correctness
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
    }

    @Test
    public void testGetSetPlanarHomographyConfidence() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE,
                0.0);

        //set new value
        assertSame(cfg.setPlanarHomographyConfidence(0.5), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyConfidence(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyMaxIterations() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);

        //set new value
        assertSame(cfg.setPlanarHomographyMaxIterations(100), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyMaxIterations(), 100);
    }

    @Test
    public void testGetSetPlanarHomographyThreshold() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD,
                0.0);

        //set new value
        assertSame(cfg.setPlanarHomographyThreshold(0.5), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyThreshold(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepInliers() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);

        //set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepInliers(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS), cfg);

        //check correctness
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepResiduals() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);

        //set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepResiduals(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS), cfg);

        //check correctness
        assertEquals(!cfg.getPlanarHomographyComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testGetSetUseDAQForAdditionalCamerasIntrinsics() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getUseDAQForAdditionalCamerasIntrinsics(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS);

        //set new value
        assertSame(cfg.setUseDAQForAdditionalCamerasIntrinics(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS), cfg);

        //check correctness
        assertEquals(!cfg.getUseDAQForAdditionalCamerasIntrinsics(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
    }

    @Test
    public void testGetSetUseDIACForAdditionalCamerasIntrinsics() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getUseDIACForAdditionalCamerasIntrinsics(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS);

        //set new value
        assertSame(cfg.setUseDIACForAdditionalCamerasIntrinsics(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS), cfg);

        //check correctness
        assertEquals(!cfg.getUseDIACForAdditionalCamerasIntrinsics(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS);
    }

    @Test
    public void testGetSetAdditionalCamerasIntrinsics() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertNull(cfg.getAdditionalCamerasIntrinsics());

        //set new value
        PinholeCameraIntrinsicParameters intrinsics = new PinholeCameraIntrinsicParameters();
        assertSame(cfg.setAdditionalCamerasIntrinsics(intrinsics), cfg);

        //check correctness
        assertSame(cfg.getAdditionalCamerasIntrinsics(), intrinsics);
    }

    @Test
    public void testGetSetAdditionalCamerasSkewness() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasSkewness(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasSkewness(1e-3), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasSkewness(), 1e-3, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasHorizontalPrincipalPoint() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasHorizontalPrincipalPoint(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasHorizontalPrincipalPoint(320), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasHorizontalPrincipalPoint(), 320, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasVerticalPrincipalPoint() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasVerticalPrincipalPoint(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasVerticalPrincipalPoint(240), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasVerticalPrincipalPoint(), 240, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasAspectRatio() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasAspectRatio(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_ASPECT_RATIO,
                0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasAspectRatio(-1.0), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasAspectRatio(), -1.0, 0.0);
    }

    @Test
    public void testGetSetUseEPnPForAdditionalCamerasEstimation() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getUseEPnPForAdditionalCamerasEstimation(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);

        //set new value
        assertSame(cfg.setUseEPnPForAdditionalCamerasEstimation(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION), cfg);

        //check correctness
        assertEquals(cfg.getUseEPnPForAdditionalCamerasEstimation(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
    }

    @Test
    public void testGetSetUseUPnPForAdditionalCamerasEstimation() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getUseUPnPForAdditionalCamerasEstimation(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);

        //set new value
        assertSame(cfg.setUseUPnPForAdditionalCamerasEstimation(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION), cfg);

        //check correctness
        assertEquals(cfg.getUseUPnPForAdditionalCamerasEstimation(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION);
    }

    @Test
    public void testGetSetAdditionalCamerasRobustEstimationMethod() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasRobustEstimationMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD);

        //set new value
        assertSame(cfg.setAdditionalCamerasRobustEstimationMethod(
                RobustEstimatorMethod.LMedS), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasRobustEstimationMethod(),
                RobustEstimatorMethod.LMedS);
    }

    @Test
    public void testGetSetAdditionalCamerasAllowPlanarConfiguration() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasAllowPlanarConfiguration(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION);

        //set new value
        assertSame(cfg.setAdditionalCamerasAllowPlanarConfiguration(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasAllowPlanarConfiguration(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION);
    }

    @Test
    public void testGetSetAdditionalCamerasAllowNullspaceDimension2() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension2(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2);

        //set new value
        assertSame(cfg.setAdditionalCamerasAllowNullspaceDimension2(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension2(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2);
    }

    @Test
    public void testGetSetAdditionalCamerasAllowNullspaceDimension3() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension3(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3);

        //set new value
        assertSame(cfg.setAdditionalCamerasAllowNullspaceDimension3(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasAllowNullspaceDimension3(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3);
    }

    @Test
    public void testGetSetAdditionalCamerasPlanarThreshold() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasPlanarThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD,
                0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasPlanarThreshold(1e-3), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasPlanarThreshold(), 1e-3, 0.0);
    }

    @Test
    public void testAreSetAdditionalCamerasRefined() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.areAdditionalCamerasRefined(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_ADDITIONAL_CAMERAS);

        //set new value
        assertSame(cfg.setAdditionalCamerasRefined(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_ADDITIONAL_CAMERAS), cfg);

        //check correctness
        assertEquals(cfg.areAdditionalCamerasRefined(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_REFINE_ADDITIONAL_CAMERAS);
    }

    @Test
    public void testIsSetAdditionalCamerasCovarianceKept() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isAdditionalCamerasCovarianceKept(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS);

        //set new value
        assertSame(cfg.setAdditionalCamerasCovarianceKept(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS),
                cfg);

        //check correctness
        assertEquals(cfg.isAdditionalCamerasCovarianceKept(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS);
    }

    @Test
    public void testGetSetAdditionalCamerasUseFastRefinement() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasUseFastRefinement(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT);

        //set new value
        assertSame(cfg.setAdditionalCamerasUseFastRefinement(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasUseFastRefinement(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT);
    }

    @Test
    public void testGetSetAdditionalCamerasConfidence() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasConfidence(0.8), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasConfidence(), 0.8, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasMaxIterations() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS);

        //set new value
        assertSame(cfg.setAdditionalCamerasMaxIterations(100), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasMaxIterations(), 100);
    }

    @Test
    public void testGetSetAdditionalCamerasThreshold() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_THRESHOLD, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasThreshold(2.0), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasThreshold(), 2.0, 0.0);
    }

    @Test
    public void testGetSetAdditionalCamerasComputeAndKeepInliers() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasComputeAndKeepInliers(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS);

        //set new value
        assertSame(cfg.setAdditionalCamerasComputeAndKeepInliers(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS), cfg);

        //check correctnes
        assertEquals(cfg.getAdditionalCamerasComputeAndKeepInliers(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS);
    }

    @Test
    public void testGetSetAdditionalCamerasComputeAndKeepResiduals() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasComputeAndKeepResiduals(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS);

        //set new value
        assertSame(cfg.setAdditionalCamerasComputeAndKeepResiduals(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasComputeAndKeepResiduals(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestSkewnessValueEnabled() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isAdditionalCamerasSuggestSkewnessValueEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestSkewnessValueEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED), cfg);

        //check correctness
        assertEquals(cfg.isAdditionalCamerasSuggestSkewnessValueEnabled(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedSkewnessValue() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasSuggestedSkewnessValue(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestedSkewnessValue(1e-3), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasSuggestedSkewnessValue(), 1e-3, 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestHorizontalFocalLengthEnabled() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestHorizontalFocalLengthEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED),
                cfg);

        //check correctness
        assertEquals(cfg.isAdditionalCamerasSuggestHorizontalFocalLengthEnabled(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedHorizontalFocalLengthValue() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(), 0.0, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestedHorizontalFocalLengthValue(320), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasSuggestedHorizontalFocalLengthValue(), 320, 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestVerticalFocalLengthEnabled() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestVerticalFocalLengthEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED),
                cfg);

        //check correctness
        assertEquals(cfg.isAdditionalCamerasSuggestVerticalFocalLengthEnabled(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedVerticalFocalLengthValue() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 0.0, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestedVerticalFocalLengthValue(240), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasSuggestedVerticalFocalLengthValue(), 240, 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestAspectRatioEnabled() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isAdditionalCamerasSuggestAspectRatioEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestAspectRatioEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED), cfg);

        //check correctness
        assertEquals(cfg.isAdditionalCamerasSuggestAspectRatioEnabled(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedAspectRatioValue() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getAdditionalCamerasSuggestedAspectRatioValue(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE, 0.0);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestedAspectRatioValue(1.1), cfg);

        //check correctness
        assertEquals(cfg.getAdditionalCamerasSuggestedAspectRatioValue(), 1.1, 0.0);
    }

    @Test
    public void testIsSetAdditionalCamerasSuggestPrincipalPointEnabled() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isAdditionalCamerasSuggestPrincipalPointEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED);

        //set new value
        assertSame(cfg.setAdditionalCamerasSuggestPrincipalPointEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.
                        DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED), cfg);
    }

    @Test
    public void testGetSetAdditionalCamerasSuggestedPrincipalPointValue() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertNull(cfg.getAdditionalCamerasSuggestedPrincipalPointValue());

        //set new value
        InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D();
        assertSame(cfg.setAdditionalCamerasSuggestedPrincipalPointValue(principalPoint), cfg);

        //check correctness
        assertSame(cfg.getAdditionalCamerasSuggestedPrincipalPointValue(), principalPoint);
    }

    @Test
    public void testIsSetHomogeneousPointTriangulatorUsed() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isHomogeneousPointTriangulatorUsed(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR);

        //set new value
        assertSame(cfg.setHomogeneousPointTriangulatorUsed(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR),
                cfg);

        //check correctness
        assertEquals(cfg.isHomogeneousPointTriangulatorUsed(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
    }

    @Test
    public void testGetSetRobustPointTriangulatorMethod() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getRobustPointTriangulatorMethod(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD);

        //set new value
        assertSame(cfg.setRobustPointTriangulatorMethod(RobustEstimatorMethod.MSAC), cfg);

        //check correctness
        assertEquals(cfg.getRobustPointTriangulatorMethod(), RobustEstimatorMethod.MSAC);
    }

    @Test
    public void testGetSetPointTriangulatorConfidence() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPointTriangulatorConfidence(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_CONFIDENCE, 0.0);

        //set new value
        assertSame(cfg.setPointTriangulatorConfidence(0.8), cfg);

        //check correctness
        assertEquals(cfg.getPointTriangulatorConfidence(), 0.8, 0.0);
    }

    @Test
    public void testGetSetPointTriangulatorMaxIterations() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPointTriangulatorMaxIterations(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS);

        //set new value
        assertSame(cfg.setPointTriangulatorMaxIterations(100), cfg);

        //check correctness
        assertEquals(cfg.getPointTriangulatorMaxIterations(), 100);
    }

    @Test
    public void testGetStPointTriangulatorThreshold() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.getPointTriangulatorThreshold(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_POINT_TRIANGULATOR_THRESHOLD, 0.0);

        //set new value
        assertSame(cfg.setPointTriangulatorThreshold(1e-3), cfg);

        //check correctness
        assertEquals(cfg.getPointTriangulatorThreshold(), 1e-3, 0.0);
    }

    @Test
    public void testGetSetCalibrationData() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertNull(cfg.getCalibrationData());

        //set new value
        ConstantVelocityModelSlamCalibrationData calibrationData = new ConstantVelocityModelSlamCalibrationData();
        assertSame(cfg.setCalibrationData(calibrationData), cfg);

        //check correctness
        assertSame(cfg.getCalibrationData(), calibrationData);
    }

    @Test
    public void testGetSetCameraPositionVariance() throws AlgebraException {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertNotNull(cfg.getCameraPositionCovariance());

        //set new value
        Matrix cov = new Matrix(3,3);
        assertSame(cfg.setCameraPositionCovariance(cov), cfg);

        //check correctness
        assertSame(cfg.getCameraPositionCovariance(), cov);
    }

    @Test
    public void testSetCameraPositionVariance() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertNotNull(cfg.getCameraPositionCovariance());

        //set new value
        Matrix cov = Matrix.diagonal(new double[]{2.0, 2.0, 2.0});
        assertSame(cfg.setCameraPositionVariance(2.0), cfg);

        //check correctness
        assertEquals(cfg.getCameraPositionCovariance(), cov);
    }

    @Test
    public void testIsSetNotifyAvailableSlamDataEnabled() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);

        //set new value
        assertSame(cfg.setNotifyAvailableSlamDataEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE), cfg);

        //check correctness
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
    }

    @Test
    public void testIsSetNotifyEstimatedSlamCameraEnabled() {
        ConstantVelocityModelSlamSparseReconstructorConfiguration cfg =
                new ConstantVelocityModelSlamSparseReconstructorConfiguration();

        //check default value
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);

        //set new value
        assertSame(cfg.setNotifyEstimatedSlamCameraEnabled(
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA), cfg);

        //check correctness
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                !ConstantVelocityModelSlamSparseReconstructorConfiguration.DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);
    }
}
