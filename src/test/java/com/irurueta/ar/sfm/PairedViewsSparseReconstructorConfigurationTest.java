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

import com.irurueta.ar.SerializationHelper;
import com.irurueta.ar.epipolar.CorrectorType;
import com.irurueta.ar.epipolar.estimators.FundamentalMatrixEstimatorMethod;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class PairedViewsSparseReconstructorConfigurationTest {

    @Test
    void testConstructor() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default values
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getRobustFundamentalMatrixEstimatorMethod());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_REFINE_FUNDAMENTAL_MATRIX,
                cfg.isFundamentalMatrixRefined());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE,
                cfg.isFundamentalMatrixCovarianceKept());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                cfg.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS,
                cfg.getFundamentalMatrixMaxIterations());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                cfg.getFundamentalMatrixThreshold(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS,
                cfg.getFundamentalMatrixComputeAndKeepInliers());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD,
                cfg.getPairedCamerasEstimatorMethod());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR,
                cfg.getDaqUseHomogeneousPointTriangulator());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO,
                cfg.getPairedCamerasAspectRatio(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_X,
                cfg.getPrincipalPointX(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_Y,
                cfg.getPrincipalPointY(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE,
                cfg.getPairedCamerasCorrectorType());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS,
                cfg.getPairedCamerasMarkValidTriangulatedPoints());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_KNOWN_INTRINSIC_PARAMETERS,
                cfg.areIntrinsicParametersKnown());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE,
                cfg.isGeneralSceneAllowed());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE,
                cfg.isPlanarSceneAllowed());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD,
                cfg.getRobustPlanarHomographyEstimatorMethod());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY,
                cfg.isPlanarHomographyRefined());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE,
                cfg.isPlanarHomographyCovarianceKept());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE,
                cfg.getPlanarHomographyConfidence(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS,
                cfg.getPlanarHomographyMaxIterations());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD,
                cfg.getPlanarHomographyThreshold(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());
    }

    @Test
    void testMake() {
        final var cfg = PairedViewsSparseReconstructorConfiguration.make();

        // check default values
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getRobustFundamentalMatrixEstimatorMethod());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_REFINE_FUNDAMENTAL_MATRIX,
                cfg.isFundamentalMatrixRefined());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE,
                cfg.isFundamentalMatrixCovarianceKept());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                cfg.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS,
                cfg.getFundamentalMatrixMaxIterations());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                cfg.getFundamentalMatrixThreshold(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS,
                cfg.getFundamentalMatrixComputeAndKeepInliers());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD,
                cfg.getPairedCamerasEstimatorMethod());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR,
                cfg.getDaqUseHomogeneousPointTriangulator());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO,
                cfg.getPairedCamerasAspectRatio(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_X,
                cfg.getPrincipalPointX(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_Y,
                cfg.getPrincipalPointY(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE,
                cfg.getPairedCamerasCorrectorType());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS,
                cfg.getPairedCamerasMarkValidTriangulatedPoints());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_KNOWN_INTRINSIC_PARAMETERS,
                cfg.areIntrinsicParametersKnown());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE,
                cfg.isGeneralSceneAllowed());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE,
                cfg.isPlanarSceneAllowed());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD,
                cfg.getRobustPlanarHomographyEstimatorMethod());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY,
                cfg.isPlanarHomographyRefined());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE,
                cfg.isPlanarHomographyCovarianceKept());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE,
                cfg.getPlanarHomographyConfidence(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS,
                cfg.getPlanarHomographyMaxIterations());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD,
                cfg.getPlanarHomographyThreshold(), 0.0);
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());
    }

    @Test
    void testGetSetNonRobustFundamentalMatrixEstimatorMethod() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getNonRobustFundamentalMatrixEstimatorMethod());

        // set new value
        assertSame(cfg, cfg.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM));

        // check correctness
        assertEquals(FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM,
                cfg.getNonRobustFundamentalMatrixEstimatorMethod());
    }

    @Test
    void testGetSetRobustFundamentalMatrixEstimatorMethod() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getRobustFundamentalMatrixEstimatorMethod());

        // set new value
        assertSame(cfg, cfg.setRobustFundamentalMatrixEstimatorMethod(RobustEstimatorMethod.LMEDS));

        // check correctness
        assertEquals(RobustEstimatorMethod.LMEDS, cfg.getRobustFundamentalMatrixEstimatorMethod());
    }

    @Test
    void testIsSetFundamentalMatrixRefined() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_REFINE_FUNDAMENTAL_MATRIX,
                cfg.isFundamentalMatrixRefined());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixRefined(false));

        // check correctness
        assertFalse(cfg.isFundamentalMatrixRefined());
    }

    @Test
    void testIsSetFundamentalMatrixCovarianceKept() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE,
                cfg.isFundamentalMatrixCovarianceKept());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixCovarianceKept(true));

        // check correctness
        assertTrue(cfg.isFundamentalMatrixCovarianceKept());
    }

    @Test
    void testGetSetFundamentalMatrixConfidence() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                cfg.getFundamentalMatrixConfidence(), 0.0);

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixConfidence(0.7));

        // check correctness
        assertEquals(0.7, cfg.getFundamentalMatrixConfidence(), 0.0);
    }

    @Test
    void testGetSetFundamentalMatrixMaxIterations() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS,
                cfg.getFundamentalMatrixMaxIterations());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixMaxIterations(10));

        // check correctness
        assertEquals(10, cfg.getFundamentalMatrixMaxIterations());
    }

    @Test
    void testGetSetFundamentalMatrixThreshold() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                cfg.getFundamentalMatrixThreshold(), 0.0);

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixThreshold(2.0));

        // check correctness
        assertEquals(2.0, cfg.getFundamentalMatrixThreshold(), 0.0);
    }

    @Test
    void testGetSetFundamentalMatrixComputeAndKeepInliers() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS,
                cfg.getFundamentalMatrixComputeAndKeepInliers());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixComputeAndKeepInliers(false));

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepInliers());
    }

    @Test
    void testGetSetFundamentalMatrixComputeAndKeepResiduals() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getFundamentalMatrixComputeAndKeepResiduals());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixComputeAndKeepResiduals(false));

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepResiduals());
    }

    @Test
    void testGetSetPairedCamerasEstimatorMethod() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD,
                cfg.getPairedCamerasEstimatorMethod());

        // set new value
        assertSame(cfg, cfg.setPairedCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC));

        // check correctness
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC, cfg.getPairedCamerasEstimatorMethod());
    }

    @Test
    void testGetSetDaqUseHomogeneousPointTriangulator() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR,
                cfg.getDaqUseHomogeneousPointTriangulator());

        // set new value
        assertSame(cfg, cfg.setDaqUseHomogeneousPointTriangulator(false));

        // check correctness
        assertFalse(cfg.getDaqUseHomogeneousPointTriangulator());
    }

    @Test
    void testGetSetPairedCamerasAspectRatio() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO,
                cfg.getPairedCamerasAspectRatio(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPairedCamerasAspectRatio(0.5));

        // check correctness
        assertEquals(0.5, cfg.getPairedCamerasAspectRatio(), 0.0);
    }

    @Test
    void testGetSetPrincipalPointX() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(0.0, cfg.getPrincipalPointX(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPrincipalPointX(10.0));

        // check correctness
        assertEquals(10.0, cfg.getPrincipalPointX(), 0.0);
    }

    @Test
    void testGetSetPrincipalPointY() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(0.0, cfg.getPrincipalPointY(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPrincipalPointY(10.0));

        // check correctness
        assertEquals(10.0, cfg.getPrincipalPointY(), 0.0);
    }

    @Test
    void testGetSetPairedCamerasCorrectorType() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE,
                cfg.getPairedCamerasCorrectorType());

        // set new value
        assertSame(cfg, cfg.setPairedCamerasCorrectorType(CorrectorType.GOLD_STANDARD));

        // check correctness
        assertEquals(CorrectorType.GOLD_STANDARD, cfg.getPairedCamerasCorrectorType());
    }

    @Test
    void testGetSetPairedCamerasMarkValidTriangulatedPoints() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS,
                cfg.getPairedCamerasMarkValidTriangulatedPoints());

        // set new value
        assertSame(cfg, cfg.setPairedCamerasMarkValidTriangulatedPoints(false));

        // check correctness
        assertFalse(cfg.getPairedCamerasMarkValidTriangulatedPoints());
    }

    @Test
    void testAreSetIntrinsicParametersKnown() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_KNOWN_INTRINSIC_PARAMETERS,
                cfg.areIntrinsicParametersKnown());

        // set new value
        assertSame(cfg, cfg.setIntrinsicParametersKnown(
                !PairedViewsSparseReconstructorConfiguration.DEFAULT_KNOWN_INTRINSIC_PARAMETERS));

        // check correctness
        assertEquals(!PairedViewsSparseReconstructorConfiguration.DEFAULT_KNOWN_INTRINSIC_PARAMETERS,
                cfg.areIntrinsicParametersKnown());
    }

    @Test
    void testIsSetGeneralSceneAllowed() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE,
                cfg.isGeneralSceneAllowed());

        // set new value
        assertSame(cfg, cfg.setGeneralSceneAllowed(
                !PairedViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE));

        // check correctness
        assertEquals(!PairedViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE,
                cfg.isGeneralSceneAllowed());
    }

    @Test
    void testIsSetPlanarSceneAllowed() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE,
                cfg.isPlanarSceneAllowed());

        // set new value
        assertSame(cfg, cfg.setPlanarSceneAllowed(
                !PairedViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE));

        // check correctness
        assertEquals(!PairedViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE,
                cfg.isPlanarSceneAllowed());
    }

    @Test
    void testGetSetRobustPlanarHomographyEstimatorMethod() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD,
                cfg.getRobustPlanarHomographyEstimatorMethod());

        // set new value
        assertSame(cfg, cfg.setRobustPlanarHomographyEstimatorMethod(RobustEstimatorMethod.RANSAC));

        // check correctness
        assertEquals(RobustEstimatorMethod.RANSAC, cfg.getRobustPlanarHomographyEstimatorMethod());
    }

    @Test
    void testIsSetPlanarHomographyRefined() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY,
                cfg.isPlanarHomographyRefined());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyRefined(
                !PairedViewsSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY));

        // check correctness
        assertEquals(!PairedViewsSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY,
                cfg.isPlanarHomographyRefined());
    }

    @Test
    void testIsSetPlanarHomographyCovarianceKept() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE,
                cfg.isPlanarHomographyCovarianceKept());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyCovarianceKept(
                !PairedViewsSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE));

        // check correctness
        assertEquals(!PairedViewsSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE,
                cfg.isPlanarHomographyCovarianceKept());
    }

    @Test
    void testGetSetPlanarHomographyConfidence() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE,
                cfg.getPlanarHomographyConfidence(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyConfidence(0.5));

        // check correctness
        assertEquals(0.5, cfg.getPlanarHomographyConfidence(), 0.0);
    }

    @Test
    void testGetSetPlanarHomographyMaxIterations() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS,
                cfg.getPlanarHomographyMaxIterations());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyMaxIterations(100));

        // check correctness
        assertEquals(100, cfg.getPlanarHomographyMaxIterations());
    }

    @Test
    void testGetSetPlanarHomographyThreshold() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD,
                cfg.getPlanarHomographyThreshold(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyThreshold(0.5));

        // check correctness
        assertEquals(0.5, cfg.getPlanarHomographyThreshold(), 0.0);
    }

    @Test
    void testGetSetPlanarHomographyComputeAndKeepInliers() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyComputeAndKeepInliers(
                !PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS));

        // check correctness
        assertEquals(!PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());
    }

    @Test
    void testGetSetPlanarHomographyComputeAndKeepResiduals() {
        final var cfg = new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyComputeAndKeepResiduals(
                !PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS));

        // check correctness
        assertEquals(!PairedViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var cfg1 = new PairedViewsSparseReconstructorConfiguration();

        // set new value
        cfg1.setNonRobustFundamentalMatrixEstimatorMethod(FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        cfg1.setRobustFundamentalMatrixEstimatorMethod(RobustEstimatorMethod.RANSAC);
        cfg1.setFundamentalMatrixRefined(false);
        cfg1.setFundamentalMatrixCovarianceKept(true);
        cfg1.setFundamentalMatrixConfidence(0.9);
        cfg1.setFundamentalMatrixMaxIterations(500);
        cfg1.setFundamentalMatrixThreshold(0.99);
        cfg1.setFundamentalMatrixComputeAndKeepInliers(false);
        cfg1.setFundamentalMatrixComputeAndKeepResiduals(false);
        cfg1.setPairedCamerasEstimatorMethod(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);
        cfg1.setDaqUseHomogeneousPointTriangulator(false);
        cfg1.setPairedCamerasAspectRatio(0.8);
        cfg1.setPrincipalPointX(1e-3);
        cfg1.setPrincipalPointY(-1e-3);
        cfg1.setPairedCamerasCorrectorType(CorrectorType.GOLD_STANDARD);
        cfg1.setPairedCamerasMarkValidTriangulatedPoints(false);
        cfg1.setIntrinsicParametersKnown(true);
        cfg1.setGeneralSceneAllowed(false);
        cfg1.setPlanarSceneAllowed(false);
        cfg1.setRobustPlanarHomographyEstimatorMethod(RobustEstimatorMethod.LMEDS);
        cfg1.setPlanarHomographyRefined(false);
        cfg1.setPlanarHomographyCovarianceKept(true);
        cfg1.setPlanarHomographyConfidence(0.7);
        cfg1.setPlanarHomographyMaxIterations(200);
        cfg1.setPlanarHomographyThreshold(1e-2);
        cfg1.setPlanarHomographyComputeAndKeepInliers(false);
        cfg1.setPlanarHomographyComputeAndKeepResiduals(false);

        // check
        assertEquals(FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM,
                cfg1.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(RobustEstimatorMethod.RANSAC, cfg1.getRobustFundamentalMatrixEstimatorMethod());
        assertFalse(cfg1.isFundamentalMatrixRefined());
        assertTrue(cfg1.isFundamentalMatrixCovarianceKept());
        assertEquals(0.9, cfg1.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(500, cfg1.getFundamentalMatrixMaxIterations());
        assertEquals(0.99, cfg1.getFundamentalMatrixThreshold(), 0.0);
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepInliers());
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC, cfg1.getPairedCamerasEstimatorMethod());
        assertFalse(cfg1.getDaqUseHomogeneousPointTriangulator());
        assertEquals(0.8, cfg1.getPairedCamerasAspectRatio(), 0.0);
        assertEquals(1e-3, cfg1.getPrincipalPointX(), 0.0);
        assertEquals(-1e-3, cfg1.getPrincipalPointY(), 0.0);
        assertEquals(CorrectorType.GOLD_STANDARD, cfg1.getPairedCamerasCorrectorType());
        assertFalse(cfg1.getPairedCamerasMarkValidTriangulatedPoints());
        assertTrue(cfg1.areIntrinsicParametersKnown());
        assertFalse(cfg1.isGeneralSceneAllowed());
        assertFalse(cfg1.isPlanarSceneAllowed());
        assertEquals(RobustEstimatorMethod.LMEDS, cfg1.getRobustPlanarHomographyEstimatorMethod());
        assertFalse(cfg1.isPlanarHomographyRefined());
        assertTrue(cfg1.isPlanarHomographyCovarianceKept());
        assertEquals(0.7, cfg1.getPlanarHomographyConfidence(), 0.0);
        assertEquals(200, cfg1.getPlanarHomographyMaxIterations());
        assertEquals(1e-2, cfg1.getPlanarHomographyThreshold(), 0.0);
        assertFalse(cfg1.getPlanarHomographyComputeAndKeepInliers());
        assertFalse(cfg1.getPlanarHomographyComputeAndKeepResiduals());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(cfg1);
        final var cfg2 = SerializationHelper.<PairedViewsSparseReconstructorConfiguration>deserialize(bytes);

        // check
        assertEquals(cfg1.getNonRobustFundamentalMatrixEstimatorMethod(),
                cfg2.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(cfg1.getRobustFundamentalMatrixEstimatorMethod(),
                cfg2.getRobustFundamentalMatrixEstimatorMethod());
        assertEquals(cfg1.isFundamentalMatrixRefined(), cfg2.isFundamentalMatrixRefined());
        assertEquals(cfg1.isFundamentalMatrixCovarianceKept(), cfg2.isFundamentalMatrixCovarianceKept());
        assertEquals(cfg1.getFundamentalMatrixConfidence(), cfg2.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(cfg1.getFundamentalMatrixMaxIterations(), cfg2.getFundamentalMatrixMaxIterations());
        assertEquals(cfg1.getFundamentalMatrixThreshold(), cfg2.getFundamentalMatrixThreshold(), 0.0);
        assertEquals(cfg1.getFundamentalMatrixComputeAndKeepInliers(),
                cfg2.getFundamentalMatrixComputeAndKeepInliers());
        assertEquals(cfg1.getFundamentalMatrixComputeAndKeepResiduals(),
                cfg2.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(cfg1.getPairedCamerasEstimatorMethod(), cfg2.getPairedCamerasEstimatorMethod());
        assertEquals(cfg1.getDaqUseHomogeneousPointTriangulator(), cfg2.getDaqUseHomogeneousPointTriangulator());
        assertEquals(cfg1.getPairedCamerasAspectRatio(), cfg2.getPairedCamerasAspectRatio(), 0.0);
        assertEquals(cfg1.getPrincipalPointX(), cfg2.getPrincipalPointX(), 0.0);
        assertEquals(cfg1.getPrincipalPointY(), cfg2.getPrincipalPointY(), 0.0);
        assertEquals(cfg1.getPairedCamerasCorrectorType(), cfg2.getPairedCamerasCorrectorType());
        assertEquals(cfg1.getPairedCamerasMarkValidTriangulatedPoints(),
                cfg2.getPairedCamerasMarkValidTriangulatedPoints());
        assertEquals(cfg1.areIntrinsicParametersKnown(), cfg2.areIntrinsicParametersKnown());
        assertEquals(cfg1.isGeneralSceneAllowed(), cfg2.isGeneralSceneAllowed());
        assertEquals(cfg1.isPlanarSceneAllowed(), cfg2.isPlanarSceneAllowed());
        assertEquals(cfg1.getRobustPlanarHomographyEstimatorMethod(), cfg2.getRobustPlanarHomographyEstimatorMethod());
        assertEquals(cfg1.isPlanarHomographyRefined(), cfg2.isPlanarHomographyRefined());
        assertEquals(cfg1.isPlanarHomographyCovarianceKept(), cfg2.isPlanarHomographyCovarianceKept());
        assertEquals(cfg1.getPlanarHomographyConfidence(), cfg2.getPlanarHomographyConfidence(), 0.0);
        assertEquals(cfg1.getPlanarHomographyMaxIterations(), cfg2.getPlanarHomographyMaxIterations());
        assertEquals(cfg1.getPlanarHomographyThreshold(), cfg2.getPlanarHomographyThreshold(), 0.0);
        assertEquals(cfg1.getPlanarHomographyComputeAndKeepInliers(), cfg2.getPlanarHomographyComputeAndKeepInliers());
        assertEquals(cfg1.getPlanarHomographyComputeAndKeepResiduals(),
                cfg2.getPlanarHomographyComputeAndKeepResiduals());
    }
}
