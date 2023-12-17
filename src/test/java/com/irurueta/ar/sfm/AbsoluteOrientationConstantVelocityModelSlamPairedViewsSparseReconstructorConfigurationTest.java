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
import com.irurueta.ar.slam.AbsoluteOrientationConstantVelocityModelSlamCalibrationData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.*;

public class AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfigurationTest {

    private static final int POINT_INHOM_COORDS = 3;
    private static final double CAMERA_POSITION_VARIANCE = 1e-6;

    @Test
    public void testConstructor() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default values
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getRobustFundamentalMatrixEstimatorMethod());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_REFINE_FUNDAMENTAL_MATRIX, cfg.isFundamentalMatrixRefined());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE,
                cfg.isFundamentalMatrixCovarianceKept());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                cfg.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS, cfg.getFundamentalMatrixMaxIterations());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, cfg.getFundamentalMatrixThreshold(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS,
                cfg.getFundamentalMatrixComputeAndKeepInliers());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD, cfg.getPairedCamerasEstimatorMethod());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR,
                cfg.getDaqUseHomogeneousPointTriangulator());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, cfg.getPairedCamerasAspectRatio(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_X, cfg.getPrincipalPointX(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_Y, cfg.getPrincipalPointY(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE, cfg.getPairedCamerasCorrectorType());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS,
                cfg.getPairedCamerasMarkValidTriangulatedPoints());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_KNOWN_INTRINSIC_PARAMETERS, cfg.areIntrinsicParametersKnown());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_ALLOW_GENERAL_SCENE, cfg.isGeneralSceneAllowed());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_ALLOW_PLANAR_SCENE, cfg.isPlanarSceneAllowed());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD,
                cfg.getRobustPlanarHomographyEstimatorMethod());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_REFINE_PLANAR_HOMOGRAPHY, cfg.isPlanarHomographyRefined());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE, cfg.isPlanarHomographyCovarianceKept());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, cfg.getPlanarHomographyConfidence(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS, cfg.getPlanarHomographyMaxIterations());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, cfg.getPlanarHomographyThreshold(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());
        assertNull(cfg.getCalibrationData());
        assertNotNull(cfg.getCameraPositionCovariance());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE, cfg.isNotifyAvailableSlamDataEnabled());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA, cfg.isNotifyEstimatedSlamCameraEnabled());
    }

    @Test
    public void testMake() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.make();

        // check default values
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getRobustFundamentalMatrixEstimatorMethod());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_REFINE_FUNDAMENTAL_MATRIX, cfg.isFundamentalMatrixRefined());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE, cfg.isFundamentalMatrixCovarianceKept());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, cfg.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS, cfg.getFundamentalMatrixMaxIterations());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, cfg.getFundamentalMatrixThreshold(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS,
                cfg.getFundamentalMatrixComputeAndKeepInliers());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD, cfg.getPairedCamerasEstimatorMethod());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR, cfg.getDaqUseHomogeneousPointTriangulator());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, cfg.getPairedCamerasAspectRatio(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_X, cfg.getPrincipalPointX(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_Y, cfg.getPrincipalPointY(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE, cfg.getPairedCamerasCorrectorType());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS,
                cfg.getPairedCamerasMarkValidTriangulatedPoints());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_KNOWN_INTRINSIC_PARAMETERS, cfg.areIntrinsicParametersKnown());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_ALLOW_GENERAL_SCENE, cfg.isGeneralSceneAllowed());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_ALLOW_PLANAR_SCENE, cfg.isPlanarSceneAllowed());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD,
                cfg.getRobustPlanarHomographyEstimatorMethod());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_REFINE_PLANAR_HOMOGRAPHY, cfg.isPlanarHomographyRefined());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE, cfg.isPlanarHomographyCovarianceKept());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, cfg.getPlanarHomographyConfidence(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS, cfg.getPlanarHomographyMaxIterations());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, cfg.getPlanarHomographyThreshold(), 0.0);
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());
        assertNull(cfg.getCalibrationData());
        assertNotNull(cfg.getCameraPositionCovariance());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE, cfg.isNotifyAvailableSlamDataEnabled());
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA, cfg.isNotifyEstimatedSlamCameraEnabled());
    }

    @Test
    public void testGetSetNonRobustFundamentalMatrixEstimatorMethod() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
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
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getRobustFundamentalMatrixEstimatorMethod());

        // set new value
        assertSame(cfg, cfg.setRobustFundamentalMatrixEstimatorMethod(RobustEstimatorMethod.LMEDS));

        // check correctness
        assertEquals(RobustEstimatorMethod.LMEDS, cfg.getRobustFundamentalMatrixEstimatorMethod());
    }

    @Test
    public void testIsSetFundamentalMatrixRefined() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_REFINE_FUNDAMENTAL_MATRIX, cfg.isFundamentalMatrixRefined());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixRefined(false));

        // check correctness
        assertFalse(cfg.isFundamentalMatrixRefined());
    }

    @Test
    public void testIsSetFundamentalMatrixCovarianceKept() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE, cfg.isFundamentalMatrixCovarianceKept());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixCovarianceKept(true));

        // check correctness
        assertTrue(cfg.isFundamentalMatrixCovarianceKept());
    }

    @Test
    public void testGetSetFundamentalMatrixConfidence() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, cfg.getFundamentalMatrixConfidence(), 0.0);

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixConfidence(0.7));

        // check correctness
        assertEquals(0.7, cfg.getFundamentalMatrixConfidence(), 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixMaxIterations() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS, cfg.getFundamentalMatrixMaxIterations());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixMaxIterations(10));

        // check correctness
        assertEquals(10, cfg.getFundamentalMatrixMaxIterations());
    }

    @Test
    public void testGetSetFundamentalMatrixThreshold() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                cfg.getFundamentalMatrixThreshold(), 0.0);

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixThreshold(2.0));

        // check correctness
        assertEquals(2.0, cfg.getFundamentalMatrixThreshold(), 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepInliers() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS,
                cfg.getFundamentalMatrixComputeAndKeepInliers());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixComputeAndKeepInliers(false));

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepInliers());
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepResiduals() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getFundamentalMatrixComputeAndKeepResiduals());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixComputeAndKeepResiduals(false));

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetPairedCamerasEstimatorMethod() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD, cfg.getPairedCamerasEstimatorMethod());

        // set new value
        assertSame(cfg.setPairedCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC), cfg);

        // check correctness
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC,
                cfg.getPairedCamerasEstimatorMethod());
    }

    @Test
    public void testGetSetDaqUseHomogeneousPointTriangulator() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR,
                cfg.getDaqUseHomogeneousPointTriangulator());

        // set new value
        assertSame(cfg, cfg.setDaqUseHomogeneousPointTriangulator(false));

        // check correctness
        assertFalse(cfg.getDaqUseHomogeneousPointTriangulator());
    }

    @Test
    public void testGetSetPairedCamerasAspectRatio() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, cfg.getPairedCamerasAspectRatio(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPairedCamerasAspectRatio(0.5));

        // check correctness
        assertEquals(0.5, cfg.getPairedCamerasAspectRatio(), 0.0);
    }

    @Test
    public void testGetSetPrincipalPointX() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(0.0, cfg.getPrincipalPointX(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPrincipalPointX(10.0));

        // check correctness
        assertEquals(10.0, cfg.getPrincipalPointX(), 0.0);
    }

    @Test
    public void testGetSetPrincipalPointY() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(0.0, cfg.getPrincipalPointY(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPrincipalPointY(10.0));

        // check correctness
        assertEquals(10.0, cfg.getPrincipalPointY(), 0.0);
    }

    @Test
    public void testGetSetPairedCamerasCorrectorType() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE, cfg.getPairedCamerasCorrectorType());

        // set new value
        assertSame(cfg, cfg.setPairedCamerasCorrectorType(CorrectorType.GOLD_STANDARD));

        // check correctness
        assertEquals(CorrectorType.GOLD_STANDARD, cfg.getPairedCamerasCorrectorType());
    }

    @Test
    public void testGetSetPairedCamerasMarkValidTriangulatedPoints() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS,
                cfg.getPairedCamerasMarkValidTriangulatedPoints());

        // set new value
        assertSame(cfg, cfg.setPairedCamerasMarkValidTriangulatedPoints(false));

        // check correctness
        assertFalse(cfg.getPairedCamerasMarkValidTriangulatedPoints());
    }

    @Test
    public void testAreSetIntrinsicParametersKnown() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_KNOWN_INTRINSIC_PARAMETERS, cfg.areIntrinsicParametersKnown());

        // set new value
        assertSame(cfg, cfg.setIntrinsicParametersKnown(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS));

        // check correctness
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_KNOWN_INTRINSIC_PARAMETERS, cfg.areIntrinsicParametersKnown());
    }

    @Test
    public void testIsSetGeneralSceneAllowed() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_ALLOW_GENERAL_SCENE, cfg.isGeneralSceneAllowed());

        // set new value
        assertSame(cfg, cfg.setGeneralSceneAllowed(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE));

        // check correctness
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_ALLOW_GENERAL_SCENE, cfg.isGeneralSceneAllowed());
    }

    @Test
    public void testIsSetPlanarSceneAllowed() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_ALLOW_PLANAR_SCENE, cfg.isPlanarSceneAllowed());

        // set new value
        assertSame(cfg, cfg.setPlanarSceneAllowed(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE));

        // check correctness
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_ALLOW_PLANAR_SCENE, cfg.isPlanarSceneAllowed());
    }

    @Test
    public void testGetSetRobustPlanarHomographyEstimatorMethod() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD,
                cfg.getRobustPlanarHomographyEstimatorMethod());

        // set new value
        assertSame(cfg, cfg.setRobustPlanarHomographyEstimatorMethod(RobustEstimatorMethod.RANSAC));

        // check correctness
        assertEquals(RobustEstimatorMethod.RANSAC, cfg.getRobustPlanarHomographyEstimatorMethod());
    }

    @Test
    public void testIsSetPlanarHomographyRefined() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_REFINE_PLANAR_HOMOGRAPHY, cfg.isPlanarHomographyRefined());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyRefined(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY));

        // check correctness
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_REFINE_PLANAR_HOMOGRAPHY, cfg.isPlanarHomographyRefined());
    }

    @Test
    public void testIsSetPlanarHomographyCovarianceKept() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE, cfg.isPlanarHomographyCovarianceKept());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyCovarianceKept(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE));

        // check correctness
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE, cfg.isPlanarHomographyCovarianceKept());
    }

    @Test
    public void testGetSetPlanarHomographyConfidence() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, cfg.getPlanarHomographyConfidence(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyConfidence(0.5));

        // check correctness
        assertEquals(0.5, cfg.getPlanarHomographyConfidence(), 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyMaxIterations() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS, cfg.getPlanarHomographyMaxIterations());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyMaxIterations(100));

        // check correctness
        assertEquals(100, cfg.getPlanarHomographyMaxIterations());
    }

    @Test
    public void testGetSetPlanarHomographyThreshold() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, cfg.getPlanarHomographyThreshold(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyThreshold(0.5));

        // check correctness
        assertEquals(0.5, cfg.getPlanarHomographyThreshold(), 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepInliers() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyComputeAndKeepInliers(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS));

        // check correctness
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepResiduals() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyComputeAndKeepResiduals(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS));

        // check correctness
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                !cfg.getPlanarHomographyComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetCalibrationData() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertNull(cfg.getCalibrationData());

        // set new value
        final AbsoluteOrientationConstantVelocityModelSlamCalibrationData calibrationData =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrationData();
        assertSame(cfg, cfg.setCalibrationData(calibrationData));

        // check correctness
        assertSame(calibrationData, cfg.getCalibrationData());
    }

    @Test
    public void testGetSetCameraPositionCovarianceAndVariance() throws AlgebraException {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertNotNull(cfg.getCameraPositionCovariance());

        Matrix cov = Matrix.identity(POINT_INHOM_COORDS, POINT_INHOM_COORDS);
        cov.multiplyByScalar(CAMERA_POSITION_VARIANCE);
        assertEquals(cov, cfg.getCameraPositionCovariance());

        // set new value
        cov = Matrix.identity(POINT_INHOM_COORDS, POINT_INHOM_COORDS);
        cov.multiplyByScalar(2.0);
        assertSame(cfg, cfg.setCameraPositionCovariance(cov));

        // check
        assertSame(cov, cfg.getCameraPositionCovariance());

        // set variance
        assertSame(cfg, cfg.setCameraPositionVariance(5.0));

        // check
        cov = Matrix.identity(POINT_INHOM_COORDS, POINT_INHOM_COORDS);
        cov.multiplyByScalar(5.0);
        assertEquals(cov, cfg.getCameraPositionCovariance());
    }

    @Test
    public void testIsSetNotifyAvailableSlamDataEnabled() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE, cfg.isNotifyAvailableSlamDataEnabled());

        // set new value
        assertSame(cfg, cfg.setNotifyAvailableSlamDataEnabled(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE));

        // check correctness
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE, cfg.isNotifyAvailableSlamDataEnabled());
    }

    @Test
    public void testIsSetNotifyEstimatedSlamCameraEnabled() {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA, cfg.isNotifyEstimatedSlamCameraEnabled());

        // set new value
        assertSame(cfg, cfg.setNotifyEstimatedSlamCameraEnabled(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA));

        // check correctness
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA, cfg.isNotifyEstimatedSlamCameraEnabled());
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg1 =
                new AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration();

        // set new values
        assertSame(cfg1, cfg1.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM));
        assertSame(cfg1, cfg1.setRobustFundamentalMatrixEstimatorMethod(
                RobustEstimatorMethod.LMEDS));
        assertSame(cfg1, cfg1.setFundamentalMatrixRefined(false));
        assertSame(cfg1, cfg1.setFundamentalMatrixCovarianceKept(true));
        assertSame(cfg1, cfg1.setFundamentalMatrixConfidence(0.7));
        assertSame(cfg1, cfg1.setFundamentalMatrixMaxIterations(10));
        assertSame(cfg1, cfg1.setFundamentalMatrixThreshold(2.0));
        assertSame(cfg1, cfg1.setFundamentalMatrixComputeAndKeepInliers(false));
        assertSame(cfg1, cfg1.setFundamentalMatrixComputeAndKeepResiduals(false));
        assertSame(cfg1, cfg1.setPairedCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC));
        assertSame(cfg1, cfg1.setDaqUseHomogeneousPointTriangulator(false));
        assertSame(cfg1, cfg1.setPairedCamerasAspectRatio(0.5));
        assertSame(cfg1, cfg1.setPrincipalPointX(10.0));
        assertSame(cfg1, cfg1.setPrincipalPointY(10.0));
        assertSame(cfg1, cfg1.setPairedCamerasCorrectorType(CorrectorType.GOLD_STANDARD));
        assertSame(cfg1, cfg1.setPairedCamerasMarkValidTriangulatedPoints(false));
        assertSame(cfg1, cfg1.setIntrinsicParametersKnown(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS));
        assertSame(cfg1, cfg1.setGeneralSceneAllowed(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE));
        assertSame(cfg1, cfg1.setPlanarSceneAllowed(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE));
        assertSame(cfg1, cfg1.setRobustPlanarHomographyEstimatorMethod(RobustEstimatorMethod.RANSAC));
        assertSame(cfg1, cfg1.setPlanarHomographyRefined(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY));
        assertSame(cfg1, cfg1.setPlanarHomographyCovarianceKept(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE));
        assertSame(cfg1, cfg1.setPlanarHomographyConfidence(0.5));
        assertSame(cfg1, cfg1.setPlanarHomographyMaxIterations(100));
        assertSame(cfg1, cfg1.setPlanarHomographyThreshold(0.5));
        assertSame(cfg1, cfg1.setPlanarHomographyComputeAndKeepInliers(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS));
        assertSame(cfg1, cfg1.setPlanarHomographyComputeAndKeepResiduals(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS));
        final AbsoluteOrientationConstantVelocityModelSlamCalibrationData calibrationData =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrationData();
        assertSame(cfg1, cfg1.setCalibrationData(calibrationData));
        final Matrix cov = Matrix.identity(POINT_INHOM_COORDS, POINT_INHOM_COORDS);
        cov.multiplyByScalar(2.0);
        assertSame(cfg1, cfg1.setCameraPositionCovariance(cov));
        assertSame(cfg1, cfg1.setNotifyAvailableSlamDataEnabled(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE));
        assertSame(cfg1, cfg1.setNotifyEstimatedSlamCameraEnabled(
                !AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA));

        // check
        assertEquals(FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM,
                cfg1.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(RobustEstimatorMethod.LMEDS, cfg1.getRobustFundamentalMatrixEstimatorMethod());
        assertFalse(cfg1.isFundamentalMatrixRefined());
        assertTrue(cfg1.isFundamentalMatrixCovarianceKept());
        assertEquals(0.7, cfg1.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(10, cfg1.getFundamentalMatrixMaxIterations());
        assertEquals(2.0, cfg1.getFundamentalMatrixThreshold(), 0.0);
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepInliers());
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC,
                cfg1.getPairedCamerasEstimatorMethod());
        assertFalse(cfg1.getDaqUseHomogeneousPointTriangulator());
        assertEquals(0.5, cfg1.getPairedCamerasAspectRatio(), 0.0);
        assertEquals(10.0, cfg1.getPrincipalPointX(), 0.0);
        assertEquals(10.0, cfg1.getPrincipalPointY(), 0.0);
        assertEquals(CorrectorType.GOLD_STANDARD, cfg1.getPairedCamerasCorrectorType());
        assertFalse(cfg1.getPairedCamerasMarkValidTriangulatedPoints());
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_KNOWN_INTRINSIC_PARAMETERS, cfg1.areIntrinsicParametersKnown());
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_ALLOW_GENERAL_SCENE, cfg1.isGeneralSceneAllowed());
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_ALLOW_PLANAR_SCENE, cfg1.isPlanarSceneAllowed());
        assertEquals(RobustEstimatorMethod.RANSAC, cfg1.getRobustPlanarHomographyEstimatorMethod());
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_REFINE_PLANAR_HOMOGRAPHY, cfg1.isPlanarHomographyRefined());
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE, cfg1.isPlanarHomographyCovarianceKept());
        assertEquals(0.5, cfg1.getPlanarHomographyConfidence(), 0.0);
        assertEquals(100, cfg1.getPlanarHomographyMaxIterations());
        assertEquals(0.5, cfg1.getPlanarHomographyThreshold(), 0.0);
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg1.getPlanarHomographyComputeAndKeepInliers());
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg1.getPlanarHomographyComputeAndKeepResiduals());
        assertSame(calibrationData, cfg1.getCalibrationData());
        assertSame(cov, cfg1.getCameraPositionCovariance());
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE, cfg1.isNotifyAvailableSlamDataEnabled());
        assertEquals(!AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration.
                DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA, cfg1.isNotifyEstimatedSlamCameraEnabled());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(cfg1);
        final AbsoluteOrientationConstantVelocityModelSlamPairedViewsSparseReconstructorConfiguration cfg2 =
                SerializationHelper.deserialize(bytes);

        // check
        assertEquals(cfg1.getNonRobustFundamentalMatrixEstimatorMethod(),
                cfg2.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(cfg1.getRobustFundamentalMatrixEstimatorMethod(),
                cfg2.getRobustFundamentalMatrixEstimatorMethod());
        //noinspection EqualsWithItself
        assertEquals(cfg1.isFundamentalMatrixRefined(), cfg1.isFundamentalMatrixRefined());
        assertEquals(cfg1.isFundamentalMatrixCovarianceKept(), cfg2.isFundamentalMatrixCovarianceKept());
        assertEquals(cfg1.getFundamentalMatrixConfidence(),
                cfg2.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(cfg1.getFundamentalMatrixMaxIterations(), cfg2.getFundamentalMatrixMaxIterations());
        assertEquals(cfg1.getFundamentalMatrixThreshold(), cfg2.getFundamentalMatrixThreshold(), 0.0);
        assertEquals(cfg1.getFundamentalMatrixComputeAndKeepInliers(),
                cfg2.getFundamentalMatrixComputeAndKeepInliers());
        assertEquals(cfg1.getFundamentalMatrixComputeAndKeepResiduals(),
                cfg2.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(cfg1.getPairedCamerasEstimatorMethod(), cfg2.getPairedCamerasEstimatorMethod());
        assertEquals(cfg1.getDaqUseHomogeneousPointTriangulator(),
                cfg2.getDaqUseHomogeneousPointTriangulator());
        assertEquals(cfg1.getPairedCamerasAspectRatio(), cfg2.getPairedCamerasAspectRatio(), 0.0);
        assertEquals(cfg1.getPrincipalPointX(), cfg2.getPrincipalPointX(), 0.0);
        assertEquals(cfg1.getPrincipalPointY(), cfg2.getPrincipalPointY(), 0.0);
        assertEquals(cfg1.getPairedCamerasCorrectorType(), cfg2.getPairedCamerasCorrectorType());
        assertEquals(cfg1.getPairedCamerasMarkValidTriangulatedPoints(),
                cfg2.getPairedCamerasMarkValidTriangulatedPoints());
        assertEquals(cfg1.areIntrinsicParametersKnown(), cfg2.areIntrinsicParametersKnown());
        assertEquals(cfg1.isGeneralSceneAllowed(), cfg2.isGeneralSceneAllowed());
        assertEquals(cfg1.isPlanarSceneAllowed(), cfg2.isPlanarSceneAllowed());
        assertEquals(cfg1.getRobustPlanarHomographyEstimatorMethod(),
                cfg2.getRobustPlanarHomographyEstimatorMethod());
        assertEquals(cfg1.isPlanarHomographyRefined(), cfg2.isPlanarHomographyRefined());
        assertEquals(cfg1.isPlanarHomographyCovarianceKept(), cfg2.isPlanarHomographyCovarianceKept());
        assertEquals(cfg1.getPlanarHomographyConfidence(), cfg2.getPlanarHomographyConfidence(), 0.0);
        assertEquals(cfg1.getPlanarHomographyMaxIterations(), cfg2.getPlanarHomographyMaxIterations());
        assertEquals(cfg1.getPlanarHomographyThreshold(), cfg2.getPlanarHomographyThreshold(), 0.0);
        assertEquals(cfg1.getPlanarHomographyComputeAndKeepInliers(),
                cfg2.getPlanarHomographyComputeAndKeepInliers());
        assertEquals(cfg1.getPlanarHomographyComputeAndKeepResiduals(),
                cfg2.getPlanarHomographyComputeAndKeepResiduals());
        assertNotSame(cfg1.getCalibrationData(), cfg2.getCalibrationData());
        assertEquals(cfg1.getCameraPositionCovariance(), cfg2.getCameraPositionCovariance());
        assertEquals(cfg1.isNotifyAvailableSlamDataEnabled(), cfg2.isNotifyAvailableSlamDataEnabled());
        assertEquals(cfg1.isNotifyEstimatedSlamCameraEnabled(), cfg2.isNotifyEstimatedSlamCameraEnabled());
    }
}
