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
import com.irurueta.ar.slam.SlamCalibrationData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.*;

public class SlamPairedViewsSparseReconstructorConfigurationTest {

    private static final int POINT_INHOM_COORDS = 3;
    private static final double CAMERA_POSITION_VARIANCE = 1e-6;

    @Test
    public void testConstructor() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getPairedCamerasAspectRatio(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getPrincipalPointX(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_X, 0.0);
        assertEquals(cfg.getPrincipalPointY(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_Y, 0.0);
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getPairedCamerasMarkValidTriangulatedPoints(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertEquals(cfg.areIntrinsicParametersKnown(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);
        assertEquals(cfg.isGeneralSceneAllowed(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
        assertNull(cfg.getCalibrationData());
        assertNotNull(cfg.getCameraPositionCovariance());
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);
    }

    @Test
    public void testMake() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                SlamPairedViewsSparseReconstructorConfiguration.make();

        // check default values
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD);
        assertEquals(cfg.isFundamentalMatrixRefined(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ESTIMATOR_METHOD);
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);
        assertEquals(cfg.getPairedCamerasAspectRatio(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, 0.0);
        assertEquals(cfg.getPrincipalPointX(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_X, 0.0);
        assertEquals(cfg.getPrincipalPointY(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_PRINCIPAL_POINT_Y, 0.0);
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_CORRECTOR_TYPE);
        assertEquals(cfg.getPairedCamerasMarkValidTriangulatedPoints(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);
        assertEquals(cfg.areIntrinsicParametersKnown(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);
        assertEquals(cfg.isGeneralSceneAllowed(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg.isPlanarSceneAllowed(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD);
        assertEquals(cfg.isPlanarHomographyRefined(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg.getPlanarHomographyConfidence(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);
        assertEquals(cfg.getPlanarHomographyThreshold(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
        assertNull(cfg.getCalibrationData());
        assertNotNull(cfg.getCameraPositionCovariance());
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);
    }

    @Test
    public void testGetSetNonRobustFundamentalMatrixEstimatorMethod() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getNonRobustFundamentalMatrixEstimatorMethod(),
                SlamPairedViewsSparseReconstructorConfiguration.
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
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getRobustFundamentalMatrixEstimatorMethod(),
                SlamPairedViewsSparseReconstructorConfiguration.
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
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isFundamentalMatrixRefined(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_FUNDAMENTAL_MATRIX);

        // set new value
        assertSame(cfg.setFundamentalMatrixRefined(false), cfg);

        // check correctness
        assertFalse(cfg.isFundamentalMatrixRefined());
    }

    @Test
    public void testIsSetFundamentalMatrixCovarianceKept() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isFundamentalMatrixCovarianceKept(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE);

        // set new value
        assertSame(cfg.setFundamentalMatrixCovarianceKept(true), cfg);

        // check correctness
        assertTrue(cfg.isFundamentalMatrixCovarianceKept());
    }

    @Test
    public void testGetSetFundamentalMatrixConfidence() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixConfidence(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE, 0.0);

        // set new value
        assertSame(cfg.setFundamentalMatrixConfidence(0.7), cfg);

        // check correctness
        assertEquals(cfg.getFundamentalMatrixConfidence(), 0.7, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixMaxIterations() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixMaxIterations(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS);

        // set new value
        assertSame(cfg.setFundamentalMatrixMaxIterations(10), cfg);

        // check correctness
        assertEquals(cfg.getFundamentalMatrixMaxIterations(), 10);
    }

    @Test
    public void testGetSetFundamentalMatrixThreshold() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixThreshold(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD, 0.0);

        // set new value
        assertSame(cfg.setFundamentalMatrixThreshold(2.0), cfg);

        // check correctness
        assertEquals(cfg.getFundamentalMatrixThreshold(), 2.0, 0.0);
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepInliers() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepInliers(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS);

        // set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepInliers(false), cfg);

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepInliers());
    }

    @Test
    public void testGetSetFundamentalMatrixComputeAndKeepResiduals() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getFundamentalMatrixComputeAndKeepResiduals(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS);

        // set new value
        assertSame(cfg.setFundamentalMatrixComputeAndKeepResiduals(false), cfg);

        // check correctness
        assertFalse(cfg.getFundamentalMatrixComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetPairedCamerasEstimatorMethod() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPairedCamerasEstimatorMethod(),
                SlamPairedViewsSparseReconstructorConfiguration.
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
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getDaqUseHomogeneousPointTriangulator(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR);

        // set new value
        assertSame(cfg.setDaqUseHomogeneousPointTriangulator(false), cfg);

        // check correctness
        assertFalse(cfg.getDaqUseHomogeneousPointTriangulator());
    }

    @Test
    public void testGetSetPairedCamerasAspectRatio() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPairedCamerasAspectRatio(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_ASPECT_RATIO, 0.0);

        // set new value
        assertSame(cfg.setPairedCamerasAspectRatio(0.5), cfg);

        // check correctness
        assertEquals(cfg.getPairedCamerasAspectRatio(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointX() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPrincipalPointX(), 0.0, 0.0);

        // set new value
        assertSame(cfg.setPrincipalPointX(10.0), cfg);

        // check correctness
        assertEquals(cfg.getPrincipalPointX(), 10.0, 0.0);
    }

    @Test
    public void testGetSetPrincipalPointY() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPrincipalPointY(), 0.0, 0.0);

        // set new value
        assertSame(cfg.setPrincipalPointY(10.0), cfg);

        // check correctness
        assertEquals(cfg.getPrincipalPointY(), 10.0, 0.0);
    }

    @Test
    public void testGetSetPairedCamerasCorrectorType() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPairedCamerasCorrectorType(),
                SlamPairedViewsSparseReconstructorConfiguration.
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
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPairedCamerasMarkValidTriangulatedPoints(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PAIRED_CAMERAS_MARK_VALID_TRIANGULATED_POINTS);

        // set new value
        assertSame(cfg.setPairedCamerasMarkValidTriangulatedPoints(false),
                cfg);

        // check correctness
        assertFalse(cfg.getPairedCamerasMarkValidTriangulatedPoints());
    }

    @Test
    public void testAreSetIntrinsicParametersKnown() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.areIntrinsicParametersKnown(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);

        // set new value
        assertSame(cfg.setIntrinsicParametersKnown(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS), cfg);

        // check correctness
        assertEquals(cfg.areIntrinsicParametersKnown(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);
    }

    @Test
    public void testIsSetGeneralSceneAllowed() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isGeneralSceneAllowed(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);

        // set new value
        assertSame(cfg.setGeneralSceneAllowed(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE), cfg);

        // check correctness
        assertEquals(cfg.isGeneralSceneAllowed(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
    }

    @Test
    public void testIsSetPlanarSceneAllowed() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isPlanarSceneAllowed(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);

        // set new value
        assertSame(cfg.setPlanarSceneAllowed(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE), cfg);

        // check correctness
        assertEquals(cfg.isPlanarSceneAllowed(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
    }

    @Test
    public void testGetSetRobustPlanarHomographyEstimatorMethod() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getRobustPlanarHomographyEstimatorMethod(),
                SlamPairedViewsSparseReconstructorConfiguration.
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
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isPlanarHomographyRefined(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);

        // set new value
        assertSame(cfg.setPlanarHomographyRefined(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY), cfg);

        // check correctness
        assertEquals(cfg.isPlanarHomographyRefined(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
    }

    @Test
    public void testIsSetPlanarHomographyCovarianceKept() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);

        // set new value
        assertSame(cfg.setPlanarHomographyCovarianceKept(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE), cfg);

        // check correctness
        assertEquals(cfg.isPlanarHomographyCovarianceKept(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
    }

    @Test
    public void testGetSetPlanarHomographyConfidence() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyConfidence(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE, 0.0);

        // set new value
        assertSame(cfg.setPlanarHomographyConfidence(0.5), cfg);

        // check correctness
        assertEquals(cfg.getPlanarHomographyConfidence(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyMaxIterations() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyMaxIterations(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS);

        // set new value
        assertSame(cfg.setPlanarHomographyMaxIterations(100), cfg);

        // check correctness
        assertEquals(cfg.getPlanarHomographyMaxIterations(), 100);
    }

    @Test
    public void testGetSetPlanarHomographyThreshold() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyThreshold(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD, 0.0);

        // set new value
        assertSame(cfg.setPlanarHomographyThreshold(0.5), cfg);

        // check correctness
        assertEquals(cfg.getPlanarHomographyThreshold(), 0.5, 0.0);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepInliers() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);

        // set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepInliers(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS), cfg);

        // check correctness
        assertEquals(cfg.getPlanarHomographyComputeAndKeepInliers(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
    }

    @Test
    public void testGetSetPlanarHomographyComputeAndKeepResiduals() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.getPlanarHomographyComputeAndKeepResiduals(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);

        // set new value
        assertSame(cfg.setPlanarHomographyComputeAndKeepResiduals(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS), cfg);

        // check correctness
        assertEquals(!cfg.getPlanarHomographyComputeAndKeepResiduals(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
    }

    @Test
    public void testGetSetCalibrationData() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertNull(cfg.getCalibrationData());

        // set new value
        final SlamCalibrationData calibrationData =
                new SlamCalibrationData();
        assertSame(cfg.setCalibrationData(calibrationData), cfg);

        // check correctness
        assertSame(cfg.getCalibrationData(), calibrationData);
    }

    @Test
    public void testGetSetCameraPositionCovarianceAndVariance() throws AlgebraException {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertNotNull(cfg.getCameraPositionCovariance());

        Matrix cov = Matrix.identity(POINT_INHOM_COORDS, POINT_INHOM_COORDS);
        cov.multiplyByScalar(CAMERA_POSITION_VARIANCE);
        assertEquals(cfg.getCameraPositionCovariance(), cov);

        // set new value
        cov = Matrix.identity(POINT_INHOM_COORDS, POINT_INHOM_COORDS);
        cov.multiplyByScalar(2.0);
        assertSame(cfg.setCameraPositionCovariance(cov), cfg);

        // check
        assertSame(cfg.getCameraPositionCovariance(), cov);

        // set variance
        assertSame(cfg.setCameraPositionVariance(5.0), cfg);

        // check
        cov = Matrix.identity(POINT_INHOM_COORDS, POINT_INHOM_COORDS);
        cov.multiplyByScalar(5.0);
        assertEquals(cfg.getCameraPositionCovariance(), cov);
    }

    @Test
    public void testIsSetNotifyAvailableSlamDataEnabled() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);

        // set new value
        assertSame(cfg.setNotifyAvailableSlamDataEnabled(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE), cfg);

        // check correctness
        assertEquals(cfg.isNotifyAvailableSlamDataEnabled(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
    }

    @Test
    public void testIsSetNotifyEstimatedSlamCameraEnabled() {
        final SlamPairedViewsSparseReconstructorConfiguration cfg =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // check default value
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);

        // set new value
        assertSame(cfg.setNotifyEstimatedSlamCameraEnabled(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA), cfg);

        // check correctness
        assertEquals(cfg.isNotifyEstimatedSlamCameraEnabled(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final SlamPairedViewsSparseReconstructorConfiguration cfg1 =
                new SlamPairedViewsSparseReconstructorConfiguration();

        // set new values
        assertSame(cfg1.setNonRobustFundamentalMatrixEstimatorMethod(
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM), cfg1);
        assertSame(cfg1.setRobustFundamentalMatrixEstimatorMethod(
                RobustEstimatorMethod.LMedS), cfg1);
        assertSame(cfg1.setFundamentalMatrixRefined(false), cfg1);
        assertSame(cfg1.setFundamentalMatrixCovarianceKept(true), cfg1);
        assertSame(cfg1.setFundamentalMatrixConfidence(0.7), cfg1);
        assertSame(cfg1.setFundamentalMatrixMaxIterations(10), cfg1);
        assertSame(cfg1.setFundamentalMatrixThreshold(2.0), cfg1);
        assertSame(cfg1.setFundamentalMatrixComputeAndKeepInliers(false), cfg1);
        assertSame(cfg1.setFundamentalMatrixComputeAndKeepResiduals(false), cfg1);
        assertSame(cfg1.setPairedCamerasEstimatorMethod(
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC),
                cfg1);
        assertSame(cfg1.setDaqUseHomogeneousPointTriangulator(false), cfg1);
        assertSame(cfg1.setPairedCamerasAspectRatio(0.5), cfg1);
        assertSame(cfg1.setPrincipalPointX(10.0), cfg1);
        assertSame(cfg1.setPrincipalPointY(10.0), cfg1);
        assertSame(cfg1.setPairedCamerasCorrectorType(
                CorrectorType.GOLD_STANDARD), cfg1);
        assertSame(cfg1.setPairedCamerasMarkValidTriangulatedPoints(false),
                cfg1);
        assertSame(cfg1.setIntrinsicParametersKnown(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS), cfg1);
        assertSame(cfg1.setGeneralSceneAllowed(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE), cfg1);
        assertSame(cfg1.setPlanarSceneAllowed(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE), cfg1);
        assertSame(cfg1.setRobustPlanarHomographyEstimatorMethod(
                RobustEstimatorMethod.RANSAC), cfg1);
        assertSame(cfg1.setPlanarHomographyRefined(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY), cfg1);
        assertSame(cfg1.setPlanarHomographyCovarianceKept(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE), cfg1);
        assertSame(cfg1.setPlanarHomographyConfidence(0.5), cfg1);
        assertSame(cfg1.setPlanarHomographyMaxIterations(100), cfg1);
        assertSame(cfg1.setPlanarHomographyThreshold(0.5), cfg1);
        assertSame(cfg1.setPlanarHomographyComputeAndKeepInliers(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS), cfg1);
        assertSame(cfg1.setPlanarHomographyComputeAndKeepResiduals(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS), cfg1);
        final SlamCalibrationData calibrationData =
                new SlamCalibrationData();
        assertSame(cfg1.setCalibrationData(calibrationData), cfg1);
        final Matrix cov = Matrix.identity(POINT_INHOM_COORDS, POINT_INHOM_COORDS);
        cov.multiplyByScalar(2.0);
        assertSame(cfg1.setCameraPositionCovariance(cov), cfg1);
        assertSame(cfg1.setNotifyAvailableSlamDataEnabled(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE), cfg1);
        assertSame(cfg1.setNotifyEstimatedSlamCameraEnabled(
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA), cfg1);

        // check
        assertEquals(cfg1.getNonRobustFundamentalMatrixEstimatorMethod(),
                FundamentalMatrixEstimatorMethod.EIGHT_POINTS_ALGORITHM);
        assertEquals(cfg1.getRobustFundamentalMatrixEstimatorMethod(),
                RobustEstimatorMethod.LMedS);
        assertFalse(cfg1.isFundamentalMatrixRefined());
        assertTrue(cfg1.isFundamentalMatrixCovarianceKept());
        assertEquals(cfg1.getFundamentalMatrixConfidence(), 0.7, 0.0);
        assertEquals(cfg1.getFundamentalMatrixMaxIterations(), 10);
        assertEquals(cfg1.getFundamentalMatrixThreshold(), 2.0, 0.0);
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepInliers());
        assertFalse(cfg1.getFundamentalMatrixComputeAndKeepResiduals());
        assertEquals(cfg1.getPairedCamerasEstimatorMethod(),
                InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC);
        assertFalse(cfg1.getDaqUseHomogeneousPointTriangulator());
        assertEquals(cfg1.getPairedCamerasAspectRatio(), 0.5, 0.0);
        assertEquals(cfg1.getPrincipalPointX(), 10.0, 0.0);
        assertEquals(cfg1.getPrincipalPointY(), 10.0, 0.0);
        assertEquals(cfg1.getPairedCamerasCorrectorType(),
                CorrectorType.GOLD_STANDARD);
        assertFalse(cfg1.getPairedCamerasMarkValidTriangulatedPoints());
        assertEquals(cfg1.areIntrinsicParametersKnown(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KNOWN_INTRINSIC_PARAMETERS);
        assertEquals(cfg1.isGeneralSceneAllowed(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_GENERAL_SCENE);
        assertEquals(cfg1.isPlanarSceneAllowed(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_ALLOW_PLANAR_SCENE);
        assertEquals(cfg1.getRobustPlanarHomographyEstimatorMethod(),
                RobustEstimatorMethod.RANSAC);
        assertEquals(cfg1.isPlanarHomographyRefined(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_REFINE_PLANAR_HOMOGRAPHY);
        assertEquals(cfg1.isPlanarHomographyCovarianceKept(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE);
        assertEquals(cfg1.getPlanarHomographyConfidence(), 0.5, 0.0);
        assertEquals(cfg1.getPlanarHomographyMaxIterations(), 100);
        assertEquals(cfg1.getPlanarHomographyThreshold(), 0.5, 0.0);
        assertEquals(cfg1.getPlanarHomographyComputeAndKeepInliers(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(!cfg1.getPlanarHomographyComputeAndKeepResiduals(),
                SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS);
        assertSame(cfg1.getCalibrationData(), calibrationData);
        assertSame(cfg1.getCameraPositionCovariance(), cov);
        assertEquals(cfg1.isNotifyAvailableSlamDataEnabled(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_SLAM_DATA_AVAILABLE);
        assertEquals(cfg1.isNotifyEstimatedSlamCameraEnabled(),
                !SlamPairedViewsSparseReconstructorConfiguration.
                        DEFAULT_NOTIFY_ESTIMATED_SLAM_CAMERA);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(cfg1);
        final SlamPairedViewsSparseReconstructorConfiguration cfg2 =
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
        assertNotSame(cfg1.getCalibrationData(), cfg2.getCalibrationData());
        assertEquals(cfg1.getCameraPositionCovariance(),
                cfg2.getCameraPositionCovariance());
        assertEquals(cfg1.isNotifyAvailableSlamDataEnabled(),
                cfg2.isNotifyAvailableSlamDataEnabled());
        assertEquals(cfg1.isNotifyEstimatedSlamCameraEnabled(),
                cfg2.isNotifyEstimatedSlamCameraEnabled());
    }
}
