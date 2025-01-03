/*
 * Copyright (C) 2016 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.ar.calibration.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.ar.calibration.DualAbsoluteQuadric;
import com.irurueta.ar.calibration.DualImageOfAbsoluteConic;
import com.irurueta.ar.calibration.InvalidTransformationException;
import com.irurueta.geometry.CameraException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

class MSACDualAbsoluteQuadricRobustEstimatorTest implements DualAbsoluteQuadricRobustEstimatorListener {

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    private static final int TIMES = 50;

    private static final int NUM_CAMS = 100;

    private static final int PERCENTAGE_OUTLIERS = 5;

    private static final double STD_ERROR = 1.0;

    @Test
    void testConstructor() {
        // test empty constructor
        var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, estimator.getDeterminantThreshold(),
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getCameras());
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertEquals(MSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);

        // test with listener
        estimator = new MSACDualAbsoluteQuadricRobustEstimator(this);

        // check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, estimator.getDeterminantThreshold(),
                0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertNull(estimator.getCameras());
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertEquals(MSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);

        // test with cameras
        final var cameras = new ArrayList<PinholeCamera>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        estimator = new MSACDualAbsoluteQuadricRobustEstimator(cameras);

        // check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, estimator.getDeterminantThreshold(),
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(cameras, estimator.getCameras());
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertEquals(MSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        final var empty = Collections.<PinholeCamera>emptyList();
        assertThrows(IllegalArgumentException.class,
                () -> new MSACDualAbsoluteQuadricRobustEstimator(empty));

        // test with cameras and listener
        estimator = new MSACDualAbsoluteQuadricRobustEstimator(cameras, this);

        // check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, estimator.getDeterminantThreshold(),
                0.0);
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());
        assertSame(cameras, estimator.getCameras());
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(RobustEstimatorMethod.MSAC, estimator.getMethod());
        assertEquals(MSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACDualAbsoluteQuadricRobustEstimator(empty, this));
    }

    @Test
    void testIsSetZeroSkewness() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertTrue(estimator.isZeroSkewness());

        // set new value
        estimator.setZeroSkewness(false);

        // check correctness
        assertFalse(estimator.isZeroSkewness());
    }

    @Test
    void testIsSetPrincipalPointAtOrigin() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertTrue(estimator.isPrincipalPointAtOrigin());

        // set new value
        estimator.setPrincipalPointAtOrigin(false);

        // check correctness
        assertFalse(estimator.isPrincipalPointAtOrigin());
    }

    @Test
    void testIsFocalDistanceAspectRatioKnown() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());

        // set new value
        estimator.setFocalDistanceAspectRatioKnown(false);

        // check correctness
        assertFalse(estimator.isFocalDistanceAspectRatioKnown());
    }

    @Test
    void testGetSetFocalDistanceAspectRatio() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);

        // set new value
        estimator.setFocalDistanceAspectRatio(0.5);

        // check correctness
        assertEquals(0.5, estimator.getFocalDistanceAspectRatio(), 0.0);
    }

    @Test
    void testIsSetSingularityEnforced() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertTrue(estimator.isSingularityEnforced());

        // set new value
        estimator.setSingularityEnforced(false);

        // check correctness
        assertFalse(estimator.isSingularityEnforced());
    }

    @Test
    void testIsSetEnforcedSingularityValidated() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertTrue(estimator.isEnforcedSingularityValidated());

        // set new value
        estimator.setEnforcedSingularityValidated(false);

        // check correctness
        assertFalse(estimator.isEnforcedSingularityValidated());
    }

    @Test
    void testGetSetDeterminantThreshold() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertEquals(DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD, estimator.getDeterminantThreshold(),
                0.0);

        // set new value
        estimator.setDeterminantThreshold(1e-3);

        // check correctness
        assertEquals(1e-3, estimator.getDeterminantThreshold(), 0.0);
    }

    @Test
    void testGetSetListenerAndIsListenerAvailable() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, estimator.getProgressDelta(),
                0.0);

        // set new value
        estimator.setProgressDelta(0.1f);

        // check correctness
        assertEquals(0.1f, estimator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> estimator.setProgressDelta(2.0f));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, estimator.getConfidence(), 0.0);

        // set new value
        estimator.setConfidence(0.8);

        // check correctness
        assertEquals(0.8, estimator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertEquals(DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS, estimator.getMaxIterations());

        // set new value
        estimator.setMaxIterations(100);

        // check correctness
        assertEquals(100, estimator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setMaxIterations(0));
    }

    @Test
    void testGetSetCameras() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // initial value
        assertNull(estimator.getCameras());

        // set new value
        final var cameras = new ArrayList<PinholeCamera>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        estimator.setCameras(cameras);

        // check correctness
        assertSame(cameras, estimator.getCameras());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setCameras(null));
        final var empty = Collections.<PinholeCamera>emptyList();
        assertThrows(IllegalArgumentException.class, () -> estimator.setCameras(empty));
    }

    @Test
    void testGetMinNumberOfRequiredCameras() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertEquals(2, estimator.getMinNumberOfRequiredCameras());

        // disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);

        // check correctness
        assertEquals(-1, estimator.getMinNumberOfRequiredCameras());

        // disable zero skewness
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setZeroSkewness(false);

        // check correctness
        assertEquals(4, estimator.getMinNumberOfRequiredCameras());

        // disable focal distance aspect ratio known
        estimator.setZeroSkewness(true);
        estimator.setFocalDistanceAspectRatioKnown(false);

        // check correctness
        assertEquals(3, estimator.getMinNumberOfRequiredCameras());

        // disable zero skewness and singularity enforcement
        estimator.setZeroSkewness(false);
        estimator.setSingularityEnforced(false);

        // check correctness
        assertEquals(5, estimator.getMinNumberOfRequiredCameras());

        // disable focal distance aspect ratio known and singularity enforcement
        estimator.setZeroSkewness(true);

        // check correctness
        assertEquals(3, estimator.getMinNumberOfRequiredCameras());
    }

    @Test
    void testAreValidConstraints() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertTrue(estimator.areValidConstraints());

        // disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);

        // check correctness
        assertFalse(estimator.areValidConstraints());

        // disable zero skewness
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setZeroSkewness(false);

        // check correctness
        assertTrue(estimator.areValidConstraints());

        // disable focal distance aspect ratio known
        estimator.setZeroSkewness(true);
        estimator.setFocalDistanceAspectRatioKnown(false);

        // check correctness
        assertFalse(estimator.areValidConstraints());

        // disable singularity enforcement
        estimator.setSingularityEnforced(false);

        // check correctness
        assertTrue(estimator.areValidConstraints());
    }

    @Test
    void testIsReady() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertNull(estimator.getCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());

        final var cameras = new ArrayList<PinholeCamera>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        estimator.setCameras(cameras);
        estimator.setQualityScores(new double[cameras.size()]);

        // check correctness
        assertTrue(estimator.isReady());

        // disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);

        // check correctness
        assertFalse(estimator.areValidConstraints());
        assertFalse(estimator.isReady());

        // enable principal point at origin
        estimator.setPrincipalPointAtOrigin(true);

        // check correctness
        assertTrue(estimator.isReady());

        // clear cameras
        cameras.clear();

        // check correctness
        assertFalse(estimator.isReady());
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();

        // check correctness
        assertEquals(MSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD, estimator.getThreshold(), 0.0);

        // set new value
        estimator.setThreshold(1e-3);

        // check correctness
        assertEquals(1e-3, estimator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setThreshold(0.0));
    }

    @Test
    void testEstimate() throws AlgebraException, InvalidTransformationException, LockedException, NotReadyException,
            CameraException, NotAvailableException {

        var numSucceeded = 0;
        for (var times = 0; times < TIMES; times++) {
            final var randomizer = new UniformRandomizer();

            // create ground truth intrinsic parameters
            final var aspectRatio = 1.0;
            var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            var verticalFocalLength = aspectRatio * horizontalFocalLength;
            var skewness = 0.0;
            var horizontalPrincipalPoint = 0.0;
            var verticalPrincipalPoint = 0.0;

            final var metricIntrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var metricDiac = new DualImageOfAbsoluteConic(metricIntrinsic);
            metricDiac.normalize();
            var metricDiacMatrix = metricDiac.asMatrix();

            // generate random projective transformation to transform ground
            // truth cameras
            var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            // ensure last element is not zero
            t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                    ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);

            final var transformation = new ProjectiveTransformation3D(t);

            transformation.normalize();

            final var projectiveDaq = new DualAbsoluteQuadric(transformation);
            projectiveDaq.normalize();

            final var projectiveDaqMatrix = projectiveDaq.asMatrix();

            double roll;
            double pitch;
            double yaw;
            double x;
            double y;
            double z;
            Quaternion q;
            InhomogeneousPoint3D cameraCenter;
            PinholeCamera metricCamera;
            PinholeCamera projectiveCamera;
            final var metricCameras = new ArrayList<PinholeCamera>();
            final var projectiveCameras = new ArrayList<PinholeCamera>();

            final var estimator = new MSACDualAbsoluteQuadricRobustEstimator();
            estimator.setListener(this);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);
            estimator.setSingularityEnforced(false);

            final var errorRandomizer = new GaussianRandomizer(0.0, STD_ERROR);
            final var inliers = new boolean[NUM_CAMS];
            for (var i = 0; i < NUM_CAMS; i++) {
                roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
                pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

                q = new Quaternion(roll, pitch, yaw);

                x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                cameraCenter = new InhomogeneousPoint3D(x, y, z);

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier (add error to metric intrinsics)
                    inliers[i] = false;
                    final var errorHorizontalFocalLength = errorRandomizer.nextDouble();
                    final var errorAspectRatio = errorRandomizer.nextDouble();
                    final var errorSkewness = errorRandomizer.nextDouble();
                    final var errorHorizontalPrincipalPoint = errorRandomizer.nextDouble();
                    final var errorVerticalPrincipalPoint = errorRandomizer.nextDouble();

                    final var outlierHorizontalFocalLength = horizontalFocalLength + errorHorizontalFocalLength;
                    final var outlierAspectRatio = aspectRatio + errorAspectRatio;
                    final var outlierVerticalFocalLength = outlierAspectRatio * outlierHorizontalFocalLength;
                    final var outlierSkewness = skewness + errorSkewness;
                    final var outlierHorizontalPrincipalPoint =
                            horizontalPrincipalPoint + errorHorizontalPrincipalPoint;
                    final var outlierVerticalPrincipalPoint = verticalPrincipalPoint + errorVerticalPrincipalPoint;

                    final var outlierMetricIntrinsic = new PinholeCameraIntrinsicParameters(
                            outlierHorizontalFocalLength, outlierVerticalFocalLength, outlierHorizontalPrincipalPoint,
                            outlierVerticalPrincipalPoint, outlierSkewness);

                    metricCamera = new PinholeCamera(outlierMetricIntrinsic, q, cameraCenter);
                } else {
                    // inlier
                    inliers[i] = true;
                    metricCamera = new PinholeCamera(metricIntrinsic, q, cameraCenter);
                }
                metricCamera.normalize();
                metricCameras.add(metricCamera);

                // transform camera
                projectiveCamera = transformation.transformAndReturnNew(metricCamera);

                projectiveCameras.add(projectiveCamera);
            }

            estimator.setCameras(projectiveCameras);

            try {
                final var estimatedDaq = estimator.estimate();
                estimatedDaq.normalize();
                final var estimatedDaqMatrix = estimatedDaq.asMatrix();

                // check that DAQ has rank 3 (zero determinant)
                if (Math.abs(Utils.det(estimatedDaqMatrix)) > ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(0.0, Utils.det(estimatedDaqMatrix), ABSOLUTE_ERROR);

                if (!projectiveDaqMatrix.equals(estimatedDaqMatrix, ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(projectiveDaqMatrix.equals(estimatedDaqMatrix, ABSOLUTE_ERROR));

                final var estimatedTransformation = estimatedDaq.getMetricToProjectiveTransformation();
                estimatedTransformation.normalize();
                final var invEstimatedTransformation =
                        (ProjectiveTransformation3D) estimatedTransformation.inverseAndReturnNew();

                // projected estimated DAQ using projective cameras to obtain
                // DIAC and check that DIAC in projective stratum is equal to
                // DIAC in metric stratum (for inlier cameras only)
                Point3D previousEstimatedMetricCenter = null;
                Point3D previousMetricCenter = null;
                Point3D estimatedMetricCenter, metricCenter;
                Rotation3D previousEstimatedMetricRotation = null;
                Rotation3D previousMetricRotation = null;
                Rotation3D estimatedMetricRotation;
                Rotation3D metricRotation;
                double distanceEstimatedCenter;
                double distanceCenter;
                var previousScale = 1.0;
                var scale = 1.0;
                PinholeCamera estimatedMetricCamera;
                var anyFailed = false;
                var count = 0;
                for (var i = 0; i < NUM_CAMS; i++) {
                    if (!inliers[i]) {
                        continue;
                    }

                    projectiveCamera = projectiveCameras.get(i);

                    final var projectedProjectiveDiac = new DualImageOfAbsoluteConic(projectiveCamera, estimatedDaq);
                    projectedProjectiveDiac.normalize();

                    final var projectedProjectiveDiacMatrix = projectedProjectiveDiac.asMatrix();

                    if (!metricDiacMatrix.equals(projectedProjectiveDiacMatrix, ABSOLUTE_ERROR)) {
                        anyFailed = true;
                        continue;
                    }
                    assertTrue(metricDiacMatrix.equals(projectedProjectiveDiacMatrix, ABSOLUTE_ERROR));

                    estimatedMetricCamera = invEstimatedTransformation.transformAndReturnNew(projectiveCamera);

                    estimatedMetricCamera.decompose();
                    final var estimatedIntrinsic = estimatedMetricCamera.getIntrinsicParameters();

                    assertEquals(horizontalFocalLength, estimatedIntrinsic.getHorizontalFocalLength(),
                            5 * LARGE_ABSOLUTE_ERROR);
                    assertEquals(verticalFocalLength, estimatedIntrinsic.getVerticalFocalLength(),
                            5 * LARGE_ABSOLUTE_ERROR);
                    assertEquals(skewness, estimatedIntrinsic.getSkewness(), 5 * LARGE_ABSOLUTE_ERROR);
                    assertEquals(horizontalPrincipalPoint, estimatedIntrinsic.getHorizontalPrincipalPoint(),
                            5 * LARGE_ABSOLUTE_ERROR);
                    assertEquals(verticalPrincipalPoint, estimatedIntrinsic.getVerticalPrincipalPoint(),
                            5 * LARGE_ABSOLUTE_ERROR);

                    if (anyFailed) {
                        continue;
                    }

                    // check that when DAQ is successfully estimated, estimated
                    // metric cameras are in the metric stratum up to an
                    // arbitrary scale

                    metricCamera = metricCameras.get(i);
                    metricCamera.decompose();

                    estimatedMetricCenter = estimatedMetricCamera.getCameraCenter();
                    metricCenter = metricCamera.getCameraCenter();
                    estimatedMetricRotation = estimatedMetricCamera.getCameraRotation();
                    metricRotation = metricCamera.getCameraRotation();

                    if (count > 0) {
                        distanceEstimatedCenter = previousEstimatedMetricCenter.distanceTo(estimatedMetricCenter);
                        distanceCenter = previousMetricCenter.distanceTo(metricCenter);
                        scale = distanceEstimatedCenter / distanceCenter;

                        final var diffEstimatedRotation = estimatedMetricRotation.combineAndReturnNew(
                                previousEstimatedMetricRotation.inverseRotationAndReturnNew());
                        final var diffRotation = metricRotation.combineAndReturnNew(
                                previousMetricRotation.inverseRotationAndReturnNew());

                        final var rot1 = diffEstimatedRotation.asInhomogeneousMatrix();
                        final var rot2 = diffRotation.asInhomogeneousMatrix();
                        assertTrue(rot1.equals(rot2, LARGE_ABSOLUTE_ERROR));
                    }

                    if (count > 1) {
                        assertEquals(scale, previousScale, 5 * LARGE_ABSOLUTE_ERROR);
                    }

                    previousEstimatedMetricCenter = estimatedMetricCenter;
                    previousMetricCenter = metricCenter;
                    previousScale = scale;

                    previousEstimatedMetricRotation = estimatedMetricRotation;
                    previousMetricRotation = metricRotation;

                    count++;
                }

                numSucceeded++;
                break;
            } catch (final RobustEstimatorException ignore) {
                // no action needed
            }
        }

        // sometimes if cameras are in degenerate configurations, DAQ estimation
        // can fail, for that reason we check that algorithm at least works one
        // if we retry multiple times.
        assertTrue(numSucceeded > 0);
    }


    @Override
    public void onEstimateStart(final DualAbsoluteQuadricRobustEstimator estimator) {
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(final DualAbsoluteQuadricRobustEstimator estimator) {
        checkLocked(estimator);
    }

    @Override
    public void onEstimateNextIteration(final DualAbsoluteQuadricRobustEstimator estimator, final int iteration) {
        checkLocked(estimator);
    }

    @Override
    public void onEstimateProgressChange(final DualAbsoluteQuadricRobustEstimator estimator, final float progress) {
        checkLocked(estimator);
    }

    private static void checkLocked(final DualAbsoluteQuadricRobustEstimator estimator) {
        assertTrue(estimator.isLocked());

        final var msacEstimator = (MSACDualAbsoluteQuadricRobustEstimator) estimator;
        assertThrows(LockedException.class, () -> msacEstimator.setThreshold(1.0));
        assertThrows(LockedException.class, () -> msacEstimator.setZeroSkewness(true));
        assertThrows(LockedException.class, () -> msacEstimator.setPrincipalPointAtOrigin(true));
        assertThrows(LockedException.class, () -> msacEstimator.setFocalDistanceAspectRatioKnown(true));
        assertThrows(LockedException.class, () -> msacEstimator.setFocalDistanceAspectRatio(2.0));
        assertThrows(LockedException.class, () -> msacEstimator.setSingularityEnforced(true));
        assertThrows(LockedException.class, () -> msacEstimator.setEnforcedSingularityValidated(true));
        assertThrows(LockedException.class, () -> msacEstimator.setDeterminantThreshold(1e-3));
        assertThrows(LockedException.class, () -> msacEstimator.setListener(null));
        assertThrows(LockedException.class, () -> msacEstimator.setProgressDelta(0.1f));
        assertThrows(LockedException.class, () -> msacEstimator.setConfidence(0.8));
        assertThrows(LockedException.class, () -> msacEstimator.setMaxIterations(100));
        assertThrows(LockedException.class, () -> msacEstimator.setCameras(null));
        assertThrows(LockedException.class, msacEstimator::estimate);
    }
}
