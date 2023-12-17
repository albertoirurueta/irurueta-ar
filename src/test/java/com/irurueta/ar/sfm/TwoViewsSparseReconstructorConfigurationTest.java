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
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.*;

public class TwoViewsSparseReconstructorConfigurationTest {

    @Test
    public void testConstructor() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default values
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getRobustFundamentalMatrixEstimatorMethod());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_REFINE_FUNDAMENTAL_MATRIX,
                cfg.isFundamentalMatrixRefined());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE,
                cfg.isFundamentalMatrixCovarianceKept());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                cfg.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS,
                cfg.getFundamentalMatrixMaxIterations());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                cfg.getFundamentalMatrixThreshold(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS,
                cfg.getFundamentalMatrixComputeAndKeepInliers());
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD,
                cfg.getInitialCamerasEstimatorMethod());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR,
                cfg.getDaqUseHomogeneousPointTriangulator());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO,
                cfg.getInitialCamerasAspectRatio(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_X,
                cfg.getPrincipalPointX(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_Y,
                cfg.getPrincipalPointY(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE,
                cfg.getInitialCamerasCorrectorType());
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS,
                cfg.getInitialCamerasMarkValidTriangulatedPoints());
        assertNull(cfg.getInitialIntrinsic1());
        assertNull(cfg.getInitialIntrinsic2());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE,
                cfg.isGeneralSceneAllowed());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE,
                cfg.isPlanarSceneAllowed());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD,
                cfg.getRobustPlanarHomographyEstimatorMethod());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY,
                cfg.isPlanarHomographyRefined());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE,
                cfg.isPlanarHomographyCovarianceKept());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE,
                cfg.getPlanarHomographyConfidence(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS,
                cfg.getPlanarHomographyMaxIterations());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD,
                cfg.getPlanarHomographyThreshold(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());
    }

    @Test
    public void testMake() {
        final TwoViewsSparseReconstructorConfiguration cfg = TwoViewsSparseReconstructorConfiguration.make();

        // check default values
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getNonRobustFundamentalMatrixEstimatorMethod());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getRobustFundamentalMatrixEstimatorMethod());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_REFINE_FUNDAMENTAL_MATRIX,
                cfg.isFundamentalMatrixRefined());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE,
                cfg.isFundamentalMatrixCovarianceKept());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                cfg.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS,
                cfg.getFundamentalMatrixMaxIterations());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                cfg.getFundamentalMatrixThreshold(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS,
                cfg.getFundamentalMatrixComputeAndKeepInliers());
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD,
                cfg.getInitialCamerasEstimatorMethod());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR,
                cfg.getDaqUseHomogeneousPointTriangulator());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO,
                cfg.getInitialCamerasAspectRatio(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_X,
                cfg.getPrincipalPointX(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_Y,
                cfg.getPrincipalPointY(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE,
                cfg.getInitialCamerasCorrectorType());
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS,
                cfg.getInitialCamerasMarkValidTriangulatedPoints());
        assertNull(cfg.getInitialIntrinsic1());
        assertNull(cfg.getInitialIntrinsic2());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE,
                cfg.isGeneralSceneAllowed());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE,
                cfg.isPlanarSceneAllowed());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD,
                cfg.getRobustPlanarHomographyEstimatorMethod());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY,
                cfg.isPlanarHomographyRefined());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE,
                cfg.isPlanarHomographyCovarianceKept());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE,
                cfg.getPlanarHomographyConfidence(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS,
                cfg.getPlanarHomographyMaxIterations());
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD,
                cfg.getPlanarHomographyThreshold(), 0.0);
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetNonRobustFundamentalMatrixEstimatorMethod() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.
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
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD,
                cfg.getRobustFundamentalMatrixEstimatorMethod());

        // set new value
        assertSame(cfg, cfg.setRobustFundamentalMatrixEstimatorMethod(RobustEstimatorMethod.LMEDS));

        // check correctness
        assertEquals(RobustEstimatorMethod.LMEDS, cfg.getRobustFundamentalMatrixEstimatorMethod());
    }

    @Test
    public void testIsSetFundamentalMatrixRefined() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_REFINE_FUNDAMENTAL_MATRIX,
                cfg.isFundamentalMatrixRefined());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixRefined(false));

        // check correctness
        assertFalse(cfg.isFundamentalMatrixRefined());
    }

    @Test
    public void testIsSetFundamentalMatrixCovarianceKept() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE,
                cfg.isFundamentalMatrixCovarianceKept());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixCovarianceKept(true));

        // check correctness
        assertTrue(cfg.isFundamentalMatrixCovarianceKept());
    }

    @Test
    public void testGetSetFundamentalMatrixConfidence() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE,
                cfg.getFundamentalMatrixConfidence(), 0.0);

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixConfidence(0.7));

        // check correctness
        assertEquals(0.7, cfg.getFundamentalMatrixConfidence(), 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixMaxIterations() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS,
                cfg.getFundamentalMatrixMaxIterations());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixMaxIterations(10));

        // check correctness
        assertEquals(10, cfg.getFundamentalMatrixMaxIterations());
    }

    @Test
    public void testGetSetFundamentalMatrixThreshold() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD,
                cfg.getFundamentalMatrixThreshold(), 0.0);

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixThreshold(2.0));

        // check correctness
        assertEquals(2.0, cfg.getFundamentalMatrixThreshold(), 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepInliers() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS,
                cfg.getFundamentalMatrixComputeAndKeepInliers());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixComputeAndKeepInliers(false));

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepInliers());
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepResiduals() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getFundamentalMatrixComputeAndKeepResiduals());

        // set new value
        assertSame(cfg, cfg.setFundamentalMatrixComputeAndKeepResiduals(false));

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetInitialCamerasEstimatorMethod() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD,
                cfg.getInitialCamerasEstimatorMethod());

        // set new value
        assertSame(cfg, cfg.setInitialCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC));

        // check correctness
        assertEquals(InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC,
                cfg.getInitialCamerasEstimatorMethod());
    }

    @Test
    public void testGetSetDaqUseHomogeneousPointTriangulator() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR,
                cfg.getDaqUseHomogeneousPointTriangulator());

        // set new value
        assertSame(cfg, cfg.setDaqUseHomogeneousPointTriangulator(false));

        // check correctness
        assertFalse(cfg.getDaqUseHomogeneousPointTriangulator());
    }

    @Test
    public void testGetSetInitialCamerasAspectRatio() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO,
                cfg.getInitialCamerasAspectRatio(), 0.0);

        // set new value
        assertSame(cfg, cfg.setInitialCamerasAspectRatio(0.5));

        // check correctness
        assertEquals(0.5, cfg.getInitialCamerasAspectRatio(), 0.0);
    }

    @Test
    public void testGetSetPrincipalPointX() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(0.0, cfg.getPrincipalPointX(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPrincipalPointX(10.0));

        // check correctness
        assertEquals(10.0, cfg.getPrincipalPointX(), 0.0);
    }

    @Test
    public void testGetSetPrincipalPointY() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(0.0, cfg.getPrincipalPointY(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPrincipalPointY(10.0));

        // check correctness
        assertEquals(10.0, cfg.getPrincipalPointY(), 0.0);
    }

    @Test
    public void testGetSetInitialCamerasCorrectorType() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE,
                cfg.getInitialCamerasCorrectorType());

        // set new value
        assertSame(cfg, cfg.setInitialCamerasCorrectorType(CorrectorType.GOLD_STANDARD));

        // check correctness
        assertEquals(CorrectorType.GOLD_STANDARD, cfg.getInitialCamerasCorrectorType());
    }

    @Test
    public void testGetSetInitialCamerasMarkValidTriangulatedPoints() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS,
                cfg.getInitialCamerasMarkValidTriangulatedPoints());

        // set new value
        assertSame(cfg, cfg.setInitialCamerasMarkValidTriangulatedPoints(false));

        // check correctness
        assertFalse(cfg.getInitialCamerasMarkValidTriangulatedPoints());
    }

    @Test
    public void testGetSetInitialIntrinsic1() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertNull(cfg.getInitialIntrinsic1());

        // set new value
        final PinholeCameraIntrinsicParameters intrinsic = new PinholeCameraIntrinsicParameters();
        assertSame(cfg, cfg.setInitialIntrinsic1(intrinsic));

        // check correctness
        assertSame(intrinsic, cfg.getInitialIntrinsic1());
    }

    @Test
    public void testGetSetInitialIntrinsic2() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertNull(cfg.getInitialIntrinsic2());

        // set new value
        final PinholeCameraIntrinsicParameters intrinsic = new PinholeCameraIntrinsicParameters();
        assertSame(cfg, cfg.setInitialIntrinsic2(intrinsic));

        // check correctness
        assertSame(intrinsic, cfg.getInitialIntrinsic2());
    }

    @Test
    public void testIsSetGeneralSceneAllowed() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE,
                cfg.isGeneralSceneAllowed());

        // set new value
        assertSame(cfg, cfg.setGeneralSceneAllowed(
                !TwoViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE));

        // check correctness
        assertEquals(!TwoViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_GENERAL_SCENE,
                cfg.isGeneralSceneAllowed());
    }

    @Test
    public void testIsSetPlanarSceneAllowed() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE,
                cfg.isPlanarSceneAllowed());

        // set new value
        assertSame(cfg, cfg.setPlanarSceneAllowed(
                !TwoViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE));

        // check correctness
        assertEquals(!TwoViewsSparseReconstructorConfiguration.DEFAULT_ALLOW_PLANAR_SCENE,
                cfg.isPlanarSceneAllowed());
    }

    @Test
    public void testGetSetRobustPlanarHomographyEstimatorMethod() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD,
                cfg.getRobustPlanarHomographyEstimatorMethod());

        // set new value
        assertSame(cfg, cfg.setRobustPlanarHomographyEstimatorMethod(RobustEstimatorMethod.RANSAC));

        // check correctness
        assertEquals(RobustEstimatorMethod.RANSAC, cfg.getRobustPlanarHomographyEstimatorMethod());
    }

    @Test
    public void testIsSetPlanarHomographyRefined() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY,
                cfg.isPlanarHomographyRefined());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyRefined(
                !TwoViewsSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY));

        // check correctness
        assertEquals(!TwoViewsSparseReconstructorConfiguration.DEFAULT_REFINE_PLANAR_HOMOGRAPHY,
                cfg.isPlanarHomographyRefined());
    }

    @Test
    public void testIsSetPlanarHomographyCovarianceKept() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE,
                cfg.isPlanarHomographyCovarianceKept());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyCovarianceKept(
                !TwoViewsSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE));

        // check correctness
        assertEquals(!TwoViewsSparseReconstructorConfiguration.DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE,
                cfg.isPlanarHomographyCovarianceKept());
    }

    @Test
    public void testGetSetPlanarHomographyConfidence() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE,
                cfg.getPlanarHomographyConfidence(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyConfidence(0.5));

        // check correctness
        assertEquals(0.5, cfg.getPlanarHomographyConfidence(), 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyMaxIterations() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS,
                cfg.getPlanarHomographyMaxIterations());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyMaxIterations(100));

        // check correctness
        assertEquals(100, cfg.getPlanarHomographyMaxIterations());
    }

    @Test
    public void testGetSetPlanarHomographyThreshold() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD,
                cfg.getPlanarHomographyThreshold(), 0.0);

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyThreshold(0.5));

        // check correctness
        assertEquals(0.5, cfg.getPlanarHomographyThreshold(), 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepInliers() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyComputeAndKeepInliers(
                !TwoViewsSparseReconstructorConfiguration.DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS));

        // check correctness
        assertEquals(!TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS,
                cfg.getPlanarHomographyComputeAndKeepInliers());
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepResiduals() {
        final TwoViewsSparseReconstructorConfiguration cfg = new TwoViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                cfg.getPlanarHomographyComputeAndKeepResiduals());

        // set new value
        assertSame(cfg, cfg.setPlanarHomographyComputeAndKeepResiduals(
                !TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS));

        // check correctness
        assertEquals(TwoViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS,
                !cfg.getPlanarHomographyComputeAndKeepResiduals());
    }

    @Test
    public void testSerializeDeserialize() throws IOException,
            ClassNotFoundException {
        final TwoViewsSparseReconstructorConfiguration cfg1 = new TwoViewsSparseReconstructorConfiguration();

        // set new values
        cfg1.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        cfg1.setRobustFundamentalMatrixEstimatorMethod(RobustEstimatorMethod.RANSAC);
        cfg1.setFundamentalMatrixRefined(false);
        cfg1.setFundamentalMatrixCovarianceKept(true);
        cfg1.setFundamentalMatrixConfidence(0.9);
        cfg1.setFundamentalMatrixMaxIterations(500);
        cfg1.setFundamentalMatrixThreshold(0.8);
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
        cfg1.setRobustPlanarHomographyEstimatorMethod(RobustEstimatorMethod.MSAC);
        cfg1.setPlanarHomographyRefined(false);
        cfg1.setPlanarHomographyCovarianceKept(true);
        cfg1.setPlanarHomographyConfidence(0.85);
        cfg1.setPlanarHomographyMaxIterations(100);
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
        assertEquals(0.8, cfg1.getFundamentalMatrixThreshold(), 0.0);
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepInliers());
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC,
                cfg1.getInitialCamerasEstimatorMethod());
        assertFalse(cfg1.getDaqUseHomogeneousPointTriangulator());
        assertEquals(0.99, cfg1.getInitialCamerasAspectRatio(), 0.0);
        assertEquals(1e-3, cfg1.getPrincipalPointX(), 0.0);
        assertEquals(-1e-3, cfg1.getPrincipalPointY(), 0.0);
        assertEquals(CorrectorType.GOLD_STANDARD, cfg1.getInitialCamerasCorrectorType());
        assertFalse(cfg1.getInitialCamerasMarkValidTriangulatedPoints());
        assertSame(intrinsic1, cfg1.getInitialIntrinsic1());
        assertSame(intrinsic2, cfg1.getInitialIntrinsic2());
        assertFalse(cfg1.isGeneralSceneAllowed());
        assertFalse(cfg1.isPlanarSceneAllowed());
        assertEquals(RobustEstimatorMethod.MSAC, cfg1.getRobustPlanarHomographyEstimatorMethod());
        assertFalse(cfg1.isPlanarHomographyRefined());
        assertTrue(cfg1.isPlanarHomographyCovarianceKept());
        assertEquals(0.85, cfg1.getPlanarHomographyConfidence(), 0.0);
        assertEquals(100, cfg1.getPlanarHomographyMaxIterations());
        assertEquals(1e-2, cfg1.getPlanarHomographyThreshold(), 0.0);
        assertFalse(cfg1.getPlanarHomographyComputeAndKeepInliers());
        assertFalse(cfg1.getPlanarHomographyComputeAndKeepResiduals());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(cfg1);
        final TwoViewsSparseReconstructorConfiguration cfg2 = SerializationHelper.deserialize(bytes);

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
        assertEquals(cfg1.getInitialCamerasEstimatorMethod(), cfg2.getInitialCamerasEstimatorMethod());
        assertEquals(cfg1.getDaqUseHomogeneousPointTriangulator(), cfg2.getDaqUseHomogeneousPointTriangulator());
        assertEquals(cfg1.getInitialCamerasAspectRatio(), cfg2.getInitialCamerasAspectRatio(), 0.0);
        assertEquals(cfg1.getPrincipalPointX(), cfg2.getPrincipalPointX(), 0.0);
        assertEquals(cfg1.getPrincipalPointY(), cfg2.getPrincipalPointY(), 0.0);
        //noinspection EqualsWithItself
        assertEquals(cfg1.getInitialCamerasCorrectorType(), cfg1.getInitialCamerasCorrectorType());
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
        assertEquals(cfg1.getPlanarHomographyConfidence(), cfg2.getPlanarHomographyConfidence(), 0.0);
        assertEquals(cfg1.getPlanarHomographyMaxIterations(), cfg2.getPlanarHomographyMaxIterations());
        assertEquals(cfg1.getPlanarHomographyThreshold(), cfg2.getPlanarHomographyThreshold(), 0.0);
        assertEquals(cfg1.getPlanarHomographyComputeAndKeepInliers(),
                cfg2.getPlanarHomographyComputeAndKeepInliers());
        assertEquals(cfg1.getPlanarHomographyComputeAndKeepResiduals(),
                cfg2.getPlanarHomographyComputeAndKeepResiduals());
    }
}
