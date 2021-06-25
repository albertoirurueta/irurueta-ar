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
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.*;

public class PairedViewsSparseReconstructorConfigurationTest {

    @Test
    public void testConstructor() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getPairedCamerasAspectRatio(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getPrincipalPointX(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_X, 0.0);
        assertEquals(cfg.getPrincipalPointY(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_Y, 0.0);
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getPairedCamerasMarkValidTriangulatedPoints(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertEquals(cfg.areIntrinsicParametersKnown(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);
        assertEquals(cfg.isGeneralSceneAllowed(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testMake() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                PairedViewsSparseReconstructorConfiguration.make();

        // check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getPairedCamerasAspectRatio(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getPrincipalPointX(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_X, 0.0);
        assertEquals(cfg.getPrincipalPointY(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_Y, 0.0);
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getPairedCamerasMarkValidTriangulatedPoints(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertEquals(cfg.areIntrinsicParametersKnown(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);
        assertEquals(cfg.isGeneralSceneAllowed(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testGetSetNonRobustFundamentalMatrixEstimatorMethod() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                PairedViewsSparseReconstructorConfiguration.
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
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                PairedViewsSparseReconstructorConfiguration.
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
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isFundamentalMatrixRefined(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);

        // set new value
        assertSame(cfg.setFundamentalMatrixRefined(false), cfg);

        // check correctness
        assertFalse(cfg.isFundamentalMatrixRefined());
    }

    @Test
    public void testIsSetFundamentalMatrixCovarianceKept() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);

        // set new value
        assertSame(cfg.setFundamentalMatrixCovarianceKept(true), cfg);

        // check correctness
        assertTrue(cfg.isFundamentalMatrixCovarianceKept());
    }

    @Test
    public void testGetSetFundamentalMatrixConfidence() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);

        // set new value
        assertSame(cfg.setFundamentalMatrixConfidence(0.7), cfg);

        // check correctness
        assertEquals(cfg.getFundamentalMatrixConfidence(), 0.7, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixMaxIterations() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);

        // set new value
        assertSame(cfg.setFundamentalMatrixMaxIterations(10), cfg);

        // check correctness
        assertEquals(cfg.getFundamentalMatrixMaxIterations(), 10);
    }

    @Test
    public void testGetSetFundamentalMatrixThreshold() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);

        // set new value
        assertSame(cfg.setFundamentalMatrixThreshold(2.0), cfg);

        // check correctness
        assertEquals(cfg.getFundamentalMatrixThreshold(), 2.0, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepInliers() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);

        // set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepInliers(false), cfg);

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepInliers());
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepResiduals() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);

        // set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepResiduals(false), cfg);

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetPairedCamerasEstimatorMethod() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD);

        // set new value
        assertSame(cfg.setPairedCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC),
                cfg);

        // check correctness
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);
    }

    @Test
    public void testGetSetDaqUseHomogeneousPointTriangulator() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);

        // set new value
        assertSame(cfg.setDaqUseHomogeneousPointTriangulator(false), cfg);

        // check correctness
        assertFalse(cfg.getDaqUseHomogeneousPointTriangulator());
    }

    @Test
    public void testGetSetPairedCamerasAspectRatio() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPairedCamerasAspectRatio(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, 0.0);

        // set new value
        assertSame(cfg.setPairedCamerasAspectRatio(0.5), cfg);

        // check correctness
        assertEquals(cfg.getPairedCamerasAspectRatio(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointX() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPrincipalPointX(), 0.0, 0.0);

        // set new value
        assertSame(cfg.setPrincipalPointX(10.0), cfg);

        // check correctness
        assertEquals(cfg.getPrincipalPointX(), 10.0, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointY() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPrincipalPointY(), 0.0, 0.0);

        // set new value
        assertSame(cfg.setPrincipalPointY(10.0), cfg);

        // check correctness
        assertEquals(cfg.getPrincipalPointY(), 10.0, 0.0);
    }

    @Test
    public void testGetSetPairedCamerasCorrectorType() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE);

        // set new value
        assertSame(cfg.setPairedCamerasCorrectorType(
                CorrectorType.GOLD_STANDARD), cfg);

        // check correctness
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                CorrectorType.GOLD_STANDARD);
    }

    @Test
    public void testGetSetPairedCamerasMarkValidTriangulatedPoints() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPairedCamerasMarkValidTriangulatedPoints(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);

        // set new value
        assertSame(cfg.setPairedCamerasMarkValidTriangulatedPoints(false),
                cfg);

        // check correctness
        assertFalse(cfg.getPairedCamerasMarkValidTriangulatedPoints());
    }

    @Test
    public void testAreSetIntrinsicParametersKnown() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.areIntrinsicParametersKnown(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);

        // set new value
        assertSame(cfg.setIntrinsicParametersKnown(
                !PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS), cfg);

        // check correctness
        assertEquals(cfg.areIntrinsicParametersKnown(),
                !PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);
    }

    @Test
    public void testIsSetGeneralSceneAllowed() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isGeneralSceneAllowed(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);

        // set new value
        assertSame(cfg.setGeneralSceneAllowed(
                !PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE), cfg);

        // check correctness
        assertEquals(cfg.isGeneralSceneAllowed(),
                !PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
    }

    @Test
    public void testIsSetPlanarSceneAllowed() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isPlanarSceneAllowed(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);

        // set new value
        assertSame(cfg.setPlanarSceneAllowed(
                !PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE), cfg);

        // check correctness
        assertEquals(cfg.isPlanarSceneAllowed(),
                !PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
    }

    @Test
    public void testGetSetRobustPlanarHomographyEstimatorMethod() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                PairedViewsSparseReconstructorConfiguration.
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
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isPlanarHomographyRefined(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);

        // set new value
        assertSame(cfg.setPlanarHomographyRefined(
                !PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY), cfg);

        // check correctness
        assertEquals(cfg.isPlanarHomographyRefined(),
                !PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
    }

    @Test
    public void testIsSetPlanarHomographyCovarianceKept() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);

        // set new value
        assertSame(cfg.setPlanarHomographyCovarianceKept(
                !PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE), cfg);

        // check correctness
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                !PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
    }

    @Test
    public void testGetSetPlanarHomographyConfidence() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyConfidence(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);

        // set new value
        assertSame(cfg.setPlanarHomographyConfidence(0.5), cfg);

        // check correctness
        assertEquals(cfg.getPlanarHomographyConfidence(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyMaxIterations() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);

        // set new value
        assertSame(cfg.setPlanarHomographyMaxIterations(100), cfg);

        // check correctness
        assertEquals(cfg.getPlanarHomographyMaxIterations(), 100);
    }

    @Test
    public void testGetSetPlanarHomographyThreshold() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyThreshold(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);

        // set new value
        assertSame(cfg.setPlanarHomographyThreshold(0.5), cfg);

        // check correctness
        assertEquals(cfg.getPlanarHomographyThreshold(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepInliers() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);

        // set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepInliers(
                !PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS), cfg);

        // check correctness
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                !PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepResiduals() {
        final PairedViewsSparseReconstructorConfiguration cfg =
                new PairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);

        // set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepResiduals(
                !PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS), cfg);

        // check correctness
        assertEquals(!cfg.getPlanarHomographyComputeAndKeepResiduals(),
                PairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final PairedViewsSparseReconstructorConfiguration cfg1 =
                new PairedViewsSparseReconstructorConfiguration();

        // set new value
        cfg1.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        cfg1.setRobustFundamentalMatrixEstimatorMethod(
                RobustEstimatorMethod.RANSAC);
        cfg1.setFundamentalMatrixRefined(false);
        cfg1.setFundamentalMatrixCovarianceKept(true);
        cfg1.setFundamentalMatrixConfidence(0.9);
        cfg1.setFundamentalMatrixMaxIterations(500);
        cfg1.setFundamentalMatrixThreshold(0.99);
        cfg1.setFundamentalMatrixComputeAndKeepInliers(false);
        cfg1.setFundamentalMatrixComputeAndKeepResiduals(false);
        cfg1.setPairedCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);
        cfg1.setDaqUseHomogeneousPointTriangulator(false);
        cfg1.setPairedCamerasAspectRatio(0.8);
        cfg1.setPrincipalPointX(1e-3);
        cfg1.setPrincipalPointY(-1e-3);
        cfg1.setPairedCamerasCorrectorType(CorrectorType.GOLD_STANDARD);
        cfg1.setPairedCamerasMarkValidTriangulatedPoints(false);
        cfg1.setIntrinsicParametersKnown(true);
        cfg1.setGeneralSceneAllowed(false);
        cfg1.setPlanarSceneAllowed(false);
        cfg1.setRobustPlanarHomographyEstimatorMethod(RobustEstimatorMethod.LMedS);
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
        assertEquals(RobustEstimatorMethod.RANSAC,
                cfg1.getRobustFundamentalMatrixEstimatorMethod());
        assertFalse(cfg1.isFundamentalMatrixRefined());
        assertTrue(cfg1.isFundamentalMatrixCovarianceKept());
        assertEquals(0.9, cfg1.getFundamentalMatrixConfidence(), 0.0);
        assertEquals(500, cfg1.getFundamentalMatrixMaxIterations());
        assertEquals(0.99, cfg1.getFundamentalMatrixThreshold(), 0.0);
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepInliers());
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC,
                cfg1.getPairedCamerasEstimatorMethod());
        assertFalse(cfg1.getDaqUseHomogeneousPointTriangulator());
        assertEquals(0.8, cfg1.getPairedCamerasAspectRatio(), 0.0);
        assertEquals(1e-3, cfg1.getPrincipalPointX(), 0.0);
        assertEquals(-1e-3, cfg1.getPrincipalPointY(), 0.0);
        assertEquals(CorrectorType.GOLD_STANDARD, cfg1.getPairedCamerasCorrectorType());
        assertFalse(cfg1.getPairedCamerasMarkValidTriangulatedPoints());
        assertTrue(cfg1.areIntrinsicParametersKnown());
        assertFalse(cfg1.isGeneralSceneAllowed());
        assertFalse(cfg1.isPlanarSceneAllowed());
        assertEquals(RobustEstimatorMethod.LMedS,
                cfg1.getRobustPlanarHomographyEstimatorMethod());
        assertFalse(cfg1.isPlanarHomographyRefined());
        assertTrue(cfg1.isPlanarHomographyCovarianceKept());
        assertEquals(0.7, cfg1.getPlanarHomographyConfidence(), 0.0);
        assertEquals(200, cfg1.getPlanarHomographyMaxIterations());
        assertEquals(1e-2, cfg1.getPlanarHomographyThreshold(), 0.0);
        assertFalse(cfg1.getPlanarHomographyComputeAndKeepInliers());
        assertFalse(cfg1.getPlanarHomographyComputeAndKeepResiduals());

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(cfg1);
        final PairedViewsSparseReconstructorConfiguration cfg2 =
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
        assertEquals(cfg1.getPairedCamerasEstimatorMethod(),
                cfg2.getPairedCamerasEstimatorMethod());
        assertEquals(cfg1.getDaqUseHomogeneousPointTriangulator(),
                cfg2.getDaqUseHomogeneousPointTriangulator());
        assertEquals(cfg1.getPairedCamerasAspectRatio(),
                cfg2.getPairedCamerasAspectRatio(), 0.0);
        assertEquals(cfg1.getPrincipalPointX(),
                cfg2.getPrincipalPointX(), 0.0);
        assertEquals(cfg1.getPrincipalPointY(),
                cfg2.getPrincipalPointY(), 0.0);
        assertEquals(cfg1.getPairedCamerasCorrectorType(),
                cfg2.getPairedCamerasCorrectorType());
        assertEquals(cfg1.getPairedCamerasMarkValidTriangulatedPoints(),
                cfg2.getPairedCamerasMarkValidTriangulatedPoints());
        assertEquals(cfg1.areIntrinsicParametersKnown(),
                cfg2.areIntrinsicParametersKnown());
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
    }
}
