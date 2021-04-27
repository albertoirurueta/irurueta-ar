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
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class WeightedDualAbsoluteQuadricEstimatorTest implements
        DualAbsoluteQuadricEstimatorListener {

    private static final double MIN_ASPECT_RATIO = 0.5;
    private static final double MAX_ASPECT_RATIO = 2.0;

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_SKEWNESS = -1e-3;
    private static final double MAX_SKEWNESS = 1e-3;

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    private static final double MIN_WEIGHT = 0.5;
    private static final double MAX_WEIGHT = 1.0;

    private static final int TIMES = 1000;

    @Test
    public void testConstructor() {
        // empty constructor
        WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // check default values
        assertNull(estimator.getCameras());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertEquals(estimator.getType(), DualAbsoluteQuadricEstimatorType.
                WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertNull(estimator.getWeights());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxCameras(),
                WeightedDualAbsoluteQuadricEstimator.DEFAULT_MAX_CAMERAS);
        assertTrue(estimator.isSortWeightsEnabled());

        // constructor with listener
        estimator = new WeightedDualAbsoluteQuadricEstimator(this);

        // check default values
        assertNull(estimator.getCameras());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getType(), DualAbsoluteQuadricEstimatorType.
                WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertNull(estimator.getWeights());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxCameras(),
                WeightedDualAbsoluteQuadricEstimator.DEFAULT_MAX_CAMERAS);
        assertTrue(estimator.isSortWeightsEnabled());

        // constructor with cameras
        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        estimator = new WeightedDualAbsoluteQuadricEstimator(cameras);

        // check default values
        assertSame(estimator.getCameras(), cameras);
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertNull(estimator.getListener());
        assertEquals(estimator.getType(), DualAbsoluteQuadricEstimatorType.
                WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertNull(estimator.getWeights());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxCameras(),
                WeightedDualAbsoluteQuadricEstimator.DEFAULT_MAX_CAMERAS);
        assertTrue(estimator.isSortWeightsEnabled());

        // constructor with cameras and listener
        estimator = new WeightedDualAbsoluteQuadricEstimator(cameras, this);

        // check default values
        assertSame(estimator.getCameras(), cameras);
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getType(), DualAbsoluteQuadricEstimatorType.
                WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertNull(estimator.getWeights());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxCameras(),
                WeightedDualAbsoluteQuadricEstimator.DEFAULT_MAX_CAMERAS);
        assertTrue(estimator.isSortWeightsEnabled());

        // constructor with cameras and weights
        final double[] weights = new double[2];
        estimator = new WeightedDualAbsoluteQuadricEstimator(cameras, weights);

        // check default values
        assertSame(estimator.getCameras(), cameras);
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertNull(estimator.getListener());
        assertEquals(estimator.getType(), DualAbsoluteQuadricEstimatorType.
                WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertSame(weights, estimator.getWeights());
        assertTrue(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxCameras(),
                WeightedDualAbsoluteQuadricEstimator.DEFAULT_MAX_CAMERAS);
        assertTrue(estimator.isSortWeightsEnabled());

        // constructor with cameras, weights and listener
        estimator = new WeightedDualAbsoluteQuadricEstimator(cameras, weights, this);

        // check default values
        assertSame(estimator.getCameras(), cameras);
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getType(), DualAbsoluteQuadricEstimatorType.
                WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR);
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertSame(weights, estimator.getWeights());
        assertTrue(estimator.areWeightsAvailable());
        assertEquals(estimator.getMaxCameras(),
                WeightedDualAbsoluteQuadricEstimator.DEFAULT_MAX_CAMERAS);
        assertTrue(estimator.isSortWeightsEnabled());
    }

    @Test
    public void testAreValidCamerasAndWeights() {
        // test valid
        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        final double[] weights = new double[cameras.size()];

        assertTrue(WeightedDualAbsoluteQuadricEstimator.
                areValidCamerasAndWeights(cameras, weights));

        // test not valid
        //noinspection ConstantConditions
        assertFalse(WeightedDualAbsoluteQuadricEstimator.
                areValidCamerasAndWeights(null, weights));
        //noinspection ConstantConditions
        assertFalse(WeightedDualAbsoluteQuadricEstimator.
                areValidCamerasAndWeights(cameras, null));
        assertFalse(WeightedDualAbsoluteQuadricEstimator.
                areValidCamerasAndWeights(cameras, new double[1]));
    }

    @Test
    public void testGetSetWeights() throws LockedException {
        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator(cameras);

        // initial value
        assertNull(estimator.getWeights());

        // set new value
        final double[] weights = new double[cameras.size()];
        estimator.setWeights(weights);

        // check correctness
        assertSame(estimator.getWeights(), weights);

        // Force IllegalArgumentException
        try {
            estimator.setWeights(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSetCamerasAndWeights() throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // initial values
        assertNull(estimator.getCameras());
        assertNull(estimator.getWeights());
        assertFalse(estimator.areWeightsAvailable());

        // set new values
        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        final double[] weights = new double[cameras.size()];

        estimator.setCamerasAndWeights(cameras, weights);

        // check correctness
        assertSame(estimator.getCameras(), cameras);
        assertSame(estimator.getWeights(), weights);

        // Force IllegalArgumentException
        try {
            estimator.setCamerasAndWeights(cameras, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxCameras() throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // initial value
        assertEquals(estimator.getMaxCameras(),
                WeightedDualAbsoluteQuadricEstimator.DEFAULT_MAX_CAMERAS);

        // set new value
        estimator.setMaxCameras(100);

        // check correctness
        assertEquals(estimator.getMaxCameras(), 100);
    }

    @Test
    public void tetIsSetsortWeightsEnabled() throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // initial value
        assertTrue(estimator.isSortWeightsEnabled());

        // set new value
        estimator.setSortWeightsEnabled(false);

        // check correctness
        assertFalse(estimator.isSortWeightsEnabled());
    }

    @Test
    public void testIsSetZeroSkewness() throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // check default value
        assertTrue(estimator.isZeroSkewness());

        // set new value
        estimator.setZeroSkewness(false);

        // check correctness
        assertFalse(estimator.isZeroSkewness());
    }

    @Test
    public void testIsSetPrincipalPointAtOrigin() throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // check default value
        assertTrue(estimator.isPrincipalPointAtOrigin());

        // check default value
        estimator.setPrincipalPointAtOrigin(false);

        // check correctness
        assertFalse(estimator.isPrincipalPointAtOrigin());
    }

    @Test
    public void testIsSetFocalDistanceAspectRatioKnown()
            throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // check default value
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());

        // set new value
        estimator.setFocalDistanceAspectRatioKnown(false);

        // check correctness
        assertFalse(estimator.isFocalDistanceAspectRatioKnown());
    }

    @Test
    public void testGetSetfocalDistanceAspectRatio() throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(),
                DualAbsoluteQuadricEstimator.
                        DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, 0.0);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double aspectRatio = randomizer.nextDouble(MIN_ASPECT_RATIO,
                MAX_ASPECT_RATIO);
        estimator.setFocalDistanceAspectRatio(aspectRatio);

        // check correctness
        assertEquals(estimator.getFocalDistanceAspectRatio(), aspectRatio, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setFocalDistanceAspectRatio(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetSingularityEnforced() throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // check default value
        assertTrue(estimator.isSingularityEnforced());

        // set new value
        estimator.setSingularityEnforced(false);

        // check correctness
        assertFalse(estimator.isSingularityEnforced());
    }

    @Test
    public void testIsSetEnforcedSingularityValidated() throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // check default value
        assertTrue(estimator.isEnforcedSingularityValidated());

        // set new value
        estimator.setEnforcedSingularityValidated(false);

        // check correctness
        assertFalse(estimator.isEnforcedSingularityValidated());
    }

    @Test
    public void testGetSetDeterminantThreshold() throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // check default value
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);

        // set new value
        estimator.setDeterminantThreshold(1e-3);

        // check correctness
        assertEquals(estimator.getDeterminantThreshold(), 1e-3, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setDeterminantThreshold(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetCamera() throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // check default value
        assertNull(estimator.getCameras());

        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        estimator.setCameras(cameras);

        // check correctness
        assertSame(estimator.getCameras(), cameras);

        // Force IllegalArgumentException
        cameras.clear();

        try {
            estimator.setCameras(cameras);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetMinNumberOfRequiredCameras() throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // check default value
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);

        // disable principal point at origin
        estimator.setPrincipalPointAtOrigin(false);

        // check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), -1);

        // disable zero skewness
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setZeroSkewness(false);

        // check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 4);

        // disable focal distance aspect ratio known
        estimator.setZeroSkewness(true);
        estimator.setFocalDistanceAspectRatioKnown(false);

        // check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 3);

        // disable zero skewness and singularity enforcement
        estimator.setZeroSkewness(false);
        estimator.setSingularityEnforced(false);

        // check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 5);

        // disable focal distance aspect ratio known and singularity enforcement
        estimator.setZeroSkewness(true);

        // check correctness
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 3);
    }

    @Test
    public void testAreValidConstraints() throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

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
        assertFalse(estimator.areValidConstraints());

        // disable focal distance aspect ratio known
        estimator.setZeroSkewness(true);
        estimator.setFocalDistanceAspectRatioKnown(false);

        // check correctness
        assertFalse(estimator.areValidConstraints());

        // disable singular enforcement
        estimator.setSingularityEnforced(false);

        // check correctness
        assertFalse(estimator.areValidConstraints());
    }

    @Test
    public void testIsReady() throws LockedException {
        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();

        // check default value
        assertNull(estimator.getCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());

        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        final double[] weights = new double[cameras.size()];

        estimator.setCamerasAndWeights(cameras, weights);

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
    public void testProject() throws InvalidPinholeCameraIntrinsicParametersException,
            AlgebraException, NonSymmetricMatrixException,
            InvalidTransformationException {

        int numSucceeded = 0;
        for (int times = 0; times < TIMES; times++) {
            // projecting DAQ with cameras results in the same DIAC for all
            // cameras
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            final double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final PinholeCameraIntrinsicParameters metricIntrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final DualAbsoluteQuadric metricDaq = new DualAbsoluteQuadric();
            final DualImageOfAbsoluteConic metricDiac = new DualImageOfAbsoluteConic(
                    metricIntrinsic);
            metricDiac.normalize();

            final Matrix metricDiacMatrix = metricDiac.asMatrix();

            PinholeCamera metricCamera;
            final List<PinholeCamera> metricCameras = new ArrayList<>();

            double roll;
            double pitch;
            double yaw;
            double x;
            double y;
            double z;
            Quaternion q;
            InhomogeneousPoint3D cameraCenter;
            for (int i = 0; i < 12; i++) {
                roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                        2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                        2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

                q = new Quaternion(roll, pitch, yaw);

                x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                cameraCenter = new InhomogeneousPoint3D(x, y, z);

                metricCamera = new PinholeCamera(metricIntrinsic, q,
                        cameraCenter);
                metricCamera.normalize();

                final DualImageOfAbsoluteConic projectedMetricDiac =
                        new DualImageOfAbsoluteConic(metricCamera, metricDaq);
                projectedMetricDiac.normalize();

                final Matrix projectedMetricDiacMatrix = projectedMetricDiac.asMatrix();

                if (!metricDiacMatrix.equals(projectedMetricDiacMatrix,
                        ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(metricDiacMatrix.equals(projectedMetricDiacMatrix,
                        ABSOLUTE_ERROR));

                metricCameras.add(metricCamera);

                final PinholeCameraIntrinsicParameters projectedMetricIntrinsic =
                        projectedMetricDiac.getIntrinsicParameters();

                assertEquals(horizontalFocalLength,
                        projectedMetricIntrinsic.getHorizontalFocalLength(),
                        ABSOLUTE_ERROR);
                assertEquals(verticalFocalLength,
                        projectedMetricIntrinsic.getVerticalFocalLength(),
                        ABSOLUTE_ERROR);
                assertEquals(skewness, projectedMetricIntrinsic.getSkewness(),
                        ABSOLUTE_ERROR);
                assertEquals(horizontalPrincipalPoint,
                        projectedMetricIntrinsic.getHorizontalPrincipalPoint(),
                        ABSOLUTE_ERROR);
                assertEquals(verticalPrincipalPoint,
                        projectedMetricIntrinsic.getVerticalPrincipalPoint(),
                        ABSOLUTE_ERROR);
            }

            // test in a projective stratum, and check that in any arbitrary
            // stratum projection does not change

            // generate random projective transformation to transform ground
            // truth cameras
            final Matrix t = Matrix.createWithUniformRandomValues(
                    ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            // ensure last element is not zero
            t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                    ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);
            final ProjectiveTransformation3D transformation =
                    new ProjectiveTransformation3D(t);

            transformation.normalize();

            final Matrix invTransT = Utils.inverse(t).transposeAndReturnNew();
            final ProjectiveTransformation3D invTransTransformation =
                    new ProjectiveTransformation3D(invTransT);

            final DualAbsoluteQuadric projectiveDaq = new DualAbsoluteQuadric();
            invTransTransformation.transform(metricDaq, projectiveDaq);

            final DualAbsoluteQuadric projectiveDaq2 = new DualAbsoluteQuadric(
                    transformation);

            PinholeCamera projectiveCamera;
            for (final PinholeCamera c : metricCameras) {
                projectiveCamera = transformation.transformAndReturnNew(c);
                projectiveCamera.normalize();

                final DualImageOfAbsoluteConic projectedProjectiveDiac =
                        new DualImageOfAbsoluteConic(projectiveCamera,
                                projectiveDaq);
                projectedProjectiveDiac.normalize();

                final DualImageOfAbsoluteConic projectedProjectiveDiac2 =
                        new DualImageOfAbsoluteConic(projectiveCamera,
                                projectiveDaq2);
                projectedProjectiveDiac2.normalize();

                final Matrix projectedProjectiveDiacMatrix =
                        projectedProjectiveDiac.asMatrix();
                final Matrix projectedProjectiveDiacMatrix2 =
                        projectedProjectiveDiac2.asMatrix();

                if (!metricDiacMatrix.equals(projectedProjectiveDiacMatrix,
                        ABSOLUTE_ERROR)) {
                    continue;
                }
                if (!metricDiacMatrix.equals(projectedProjectiveDiacMatrix2,
                        ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(metricDiacMatrix.equals(
                        projectedProjectiveDiacMatrix, ABSOLUTE_ERROR));
                assertTrue(metricDiacMatrix.equals(
                        projectedProjectiveDiacMatrix2, ABSOLUTE_ERROR));

                final PinholeCameraIntrinsicParameters projectedProjectiveIntrinsic =
                        projectedProjectiveDiac.getIntrinsicParameters();

                assertEquals(horizontalFocalLength,
                        projectedProjectiveIntrinsic.getHorizontalFocalLength(),
                        5 * LARGE_ABSOLUTE_ERROR);
                assertEquals(verticalFocalLength,
                        projectedProjectiveIntrinsic.getVerticalFocalLength(),
                        5 * LARGE_ABSOLUTE_ERROR);
                assertEquals(skewness,
                        projectedProjectiveIntrinsic.getSkewness(),
                        5 * LARGE_ABSOLUTE_ERROR);
                assertEquals(horizontalPrincipalPoint,
                        projectedProjectiveIntrinsic.
                                getHorizontalPrincipalPoint(),
                        5 * LARGE_ABSOLUTE_ERROR);
                assertEquals(verticalPrincipalPoint,
                        projectedProjectiveIntrinsic.
                                getVerticalPrincipalPoint(),
                        5 * LARGE_ABSOLUTE_ERROR);
            }

            numSucceeded++;
        }

        // sometimes if cameras are in degenerate configurations, DAQ estimation
        // can fail, for that reason we check that algorithm at least workes once
        // if we retry multiple times
        assertTrue(numSucceeded > 0);
    }

    // zero skewness
    // principal point at origin
    // focal distance aspect ratio known (1.0)
    // singularity not enforced
    @Test
    public void testEstimate1() throws LockedException,
            AlgebraException, NotReadyException,
            InvalidTransformationException, NotAvailableException,
            CameraException {

        int numSucceeded = 0;
        for (int times = 0; times < TIMES; times++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // create ground truth intrinsic parameters
            final double aspectRatio = 1.0;
            final double horizontalFocalLength =
                    randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = aspectRatio * horizontalFocalLength;
            final double skewness = 0.0;
            final double horizontalPrincipalPoint = 0.0;
            final double verticalPrincipalPoint = 0.0;

            final PinholeCameraIntrinsicParameters metricIntrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final DualImageOfAbsoluteConic metricDiac = new DualImageOfAbsoluteConic(
                    metricIntrinsic);
            metricDiac.normalize();
            final Matrix metricDiacMatrix = metricDiac.asMatrix();

            // generate random projective transformation to transform ground
            // truth cameras
            Matrix t = Matrix.createWithUniformRandomValues(
                    ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            // ensure last element is not zero
            t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                    ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);

            final ProjectiveTransformation3D transformation =
                    new ProjectiveTransformation3D(t);

            transformation.normalize();

            final DualAbsoluteQuadric projectiveDaq = new DualAbsoluteQuadric(
                    transformation);
            projectiveDaq.normalize();

            final Matrix projectiveDaqMatrix = projectiveDaq.asMatrix();

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
            final List<PinholeCamera> metricCameras = new ArrayList<>();
            final List<PinholeCamera> projectiveCameras =
                    new ArrayList<>();

            final WeightedDualAbsoluteQuadricEstimator estimator =
                    new WeightedDualAbsoluteQuadricEstimator();
            estimator.setListener(this);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);
            estimator.setSingularityEnforced(false);

            final int numCams = estimator.getMinNumberOfRequiredCameras();
            final double[] weights = new double[numCams];
            for (int i = 0; i < numCams; i++) {
                roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                        2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                        2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

                q = new Quaternion(roll, pitch, yaw);

                x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                cameraCenter = new InhomogeneousPoint3D(x, y, z);

                metricCamera = new PinholeCamera(metricIntrinsic, q,
                        cameraCenter);
                metricCamera.normalize();
                metricCameras.add(metricCamera);

                // transform camera
                projectiveCamera = transformation.transformAndReturnNew(
                        metricCamera);

                projectiveCameras.add(projectiveCamera);
                weights[i] = randomizer.nextDouble(MIN_WEIGHT, MAX_WEIGHT);
            }

            estimator.setCamerasAndWeights(projectiveCameras, weights);

            try {
                final DualAbsoluteQuadric estimatedDaq = estimator.estimate();
                estimatedDaq.normalize();
                final Matrix estimatedDaqMatrix = estimatedDaq.asMatrix();

                final DualAbsoluteQuadric estimatedDaq2 = new DualAbsoluteQuadric();
                estimator.estimate(estimatedDaq2);
                estimatedDaq2.normalize();

                assertTrue(estimatedDaqMatrix.equals(estimatedDaq2.asMatrix(),
                        ABSOLUTE_ERROR));

                // check that DAQ has rank 3 (zero determinant)
                if (Math.abs(Utils.det(estimatedDaqMatrix)) > ABSOLUTE_ERROR) {
                    continue;
                }

                assertEquals(Utils.det(estimatedDaqMatrix), 0.0,
                        ABSOLUTE_ERROR);

                if (!projectiveDaqMatrix.equals(estimatedDaqMatrix,
                        ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(projectiveDaqMatrix.equals(estimatedDaqMatrix,
                        ABSOLUTE_ERROR));

                final ProjectiveTransformation3D estimatedTransformation =
                        estimatedDaq.getMetricToProjectiveTransformation();
                estimatedTransformation.normalize();
                final ProjectiveTransformation3D invEstimatedTransformation =
                        (ProjectiveTransformation3D) estimatedTransformation.
                                inverseAndReturnNew();

                // project estimated DAQ using projective cameras to obtain DIAC
                // and check that DIAC in projective stratum is equal to DIAC in
                // metric stratum
                Point3D previousEstimatedMetricCenter = null;
                Point3D previousMetricCenter = null;
                Point3D estimatedMetricCenter, metricCenter;
                Rotation3D previousEstimatedMetricRotation = null;
                Rotation3D previousMetricRotation = null;
                Rotation3D estimatedMetricRotation, metricRotation;
                double distanceEstimatedCenter;
                double distanceCenter;
                double previousScale = 1.0;
                double scale = 1.0;
                PinholeCamera estimatedMetricCamera;
                boolean anyFailed = false;
                for (int i = 0; i < numCams; i++) {
                    projectiveCamera = projectiveCameras.get(i);

                    final DualImageOfAbsoluteConic projectedProjectiveDiac =
                            new DualImageOfAbsoluteConic(projectiveCamera,
                                    estimatedDaq);
                    projectedProjectiveDiac.normalize();

                    final Matrix projectedProjectiveDiacMatrix =
                            projectedProjectiveDiac.asMatrix();

                    if (!metricDiacMatrix.equals(
                            projectedProjectiveDiacMatrix, ABSOLUTE_ERROR)) {
                        anyFailed = true;
                        continue;
                    }
                    assertTrue(metricDiacMatrix.equals(
                            projectedProjectiveDiacMatrix, ABSOLUTE_ERROR));

                    estimatedMetricCamera = invEstimatedTransformation.
                            transformAndReturnNew(projectiveCamera);

                    estimatedMetricCamera.decompose();
                    final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                            estimatedMetricCamera.getIntrinsicParameters();

                    if (Math.abs(horizontalFocalLength - estimatedIntrinsic.getHorizontalFocalLength()) > 5 * LARGE_ABSOLUTE_ERROR) {
                        continue;
                    }
                    assertEquals(horizontalFocalLength,
                            estimatedIntrinsic.getHorizontalFocalLength(),
                            5 * LARGE_ABSOLUTE_ERROR);
                    if (Math.abs(verticalFocalLength - estimatedIntrinsic.getVerticalFocalLength()) > 5 * LARGE_ABSOLUTE_ERROR) {
                        continue;
                    }
                    assertEquals(verticalFocalLength,
                            estimatedIntrinsic.getVerticalFocalLength(),
                            5 * LARGE_ABSOLUTE_ERROR);
                    if (Math.abs(skewness - estimatedIntrinsic.getSkewness()) > 5 * LARGE_ABSOLUTE_ERROR) {
                        continue;
                    }
                    assertEquals(skewness, estimatedIntrinsic.getSkewness(),
                            5 * LARGE_ABSOLUTE_ERROR);
                    if (Math.abs(horizontalPrincipalPoint - estimatedIntrinsic.getHorizontalPrincipalPoint()) > 5 * LARGE_ABSOLUTE_ERROR) {
                        continue;
                    }
                    assertEquals(horizontalPrincipalPoint,
                            estimatedIntrinsic.getHorizontalPrincipalPoint(),
                            5 * LARGE_ABSOLUTE_ERROR);
                    if (Math.abs(verticalPrincipalPoint - estimatedIntrinsic.getVerticalPrincipalPoint()) > 5 * LARGE_ABSOLUTE_ERROR) {
                        continue;
                    }
                    assertEquals(verticalPrincipalPoint,
                            estimatedIntrinsic.getVerticalPrincipalPoint(),
                            5 * LARGE_ABSOLUTE_ERROR);

                    if (anyFailed) {
                        continue;
                    }

                    // check that when DAQ is successfully estimated, estimated
                    // metric cameras are in the metric stratum up to an
                    // arbitrary scale

                    metricCamera = metricCameras.get(i);
                    metricCamera.decompose();

                    estimatedMetricCenter = estimatedMetricCamera.
                            getCameraCenter();
                    metricCenter = metricCamera.getCameraCenter();
                    estimatedMetricRotation = estimatedMetricCamera.
                            getCameraRotation();
                    metricRotation = metricCamera.getCameraRotation();

                    if (i > 0 && previousEstimatedMetricCenter != null) {
                        distanceEstimatedCenter = previousEstimatedMetricCenter.
                                distanceTo(estimatedMetricCenter);
                        distanceCenter = previousMetricCenter.distanceTo(
                                metricCenter);
                        scale = distanceEstimatedCenter / distanceCenter;

                        final Rotation3D diffEstimatedRotation =
                                estimatedMetricRotation.combineAndReturnNew(
                                        previousEstimatedMetricRotation.
                                                inverseRotationAndReturnNew());
                        final Rotation3D diffRotation = metricRotation.
                                combineAndReturnNew(previousMetricRotation.
                                        inverseRotationAndReturnNew());

                        final Matrix rot1 = diffEstimatedRotation.
                                asInhomogeneousMatrix();
                        final Matrix rot2 = diffRotation.asInhomogeneousMatrix();
                        assertTrue(rot1.equals(rot2, LARGE_ABSOLUTE_ERROR));
                    }

                    if (i > 1) {
                        assertEquals(scale, previousScale, 5 * LARGE_ABSOLUTE_ERROR);
                    }

                    previousEstimatedMetricCenter = estimatedMetricCenter;
                    previousMetricCenter = metricCenter;
                    previousScale = scale;

                    previousEstimatedMetricRotation = estimatedMetricRotation;
                    previousMetricRotation = metricRotation;
                }

                numSucceeded++;
                break;
            } catch (final DualAbsoluteQuadricEstimatorException ignore) {
            }
        }

        // sometimes if cameras are in degenerate configurations, DAQ estimation
        // can fail, for that reason we check that algorithm at least workes once
        // if we retry multiple times
        assertTrue(numSucceeded > 0);
    }

    // zero skewness
    // principal point at origin
    // focal distance aspect ratio known (1.0)
    // singularity enforced
    @Test
    public void testEstimate2() throws LockedException,
            AlgebraException, NotReadyException,
            InvalidTransformationException, NotAvailableException,
            CameraException {

        int numSucceeded = 0;
        for (int times = 0; times < TIMES; times++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // create ground truth intrinsic parameters
            final double aspectRatio = 1.0;
            final double horizontalFocalLength =
                    randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final double verticalFocalLength = aspectRatio * horizontalFocalLength;
            final double skewness = 0.0;
            final double horizontalPrincipalPoint = 0.0;
            final double verticalPrincipalPoint = 0.0;

            final PinholeCameraIntrinsicParameters metricIntrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                            verticalFocalLength, horizontalPrincipalPoint,
                            verticalPrincipalPoint, skewness);

            final DualImageOfAbsoluteConic metricDiac = new DualImageOfAbsoluteConic(
                    metricIntrinsic);
            metricDiac.normalize();
            final Matrix metricDiacMatrix = metricDiac.asMatrix();

            // generate random projective transformation to transform ground
            // truth cameras
            final Matrix t = Matrix.createWithUniformRandomValues(
                    ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            // ensure last element is not zero
            t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                    ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);

            final ProjectiveTransformation3D transformation =
                    new ProjectiveTransformation3D(t);

            transformation.normalize();

            final DualAbsoluteQuadric projectiveDaq = new DualAbsoluteQuadric(
                    transformation);
            projectiveDaq.normalize();

            final Matrix projectiveDaqMatrix = projectiveDaq.asMatrix();

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
            final List<PinholeCamera> metricCameras = new ArrayList<>();
            final List<PinholeCamera> projectiveCameras =
                    new ArrayList<>();

            final WeightedDualAbsoluteQuadricEstimator estimator =
                    new WeightedDualAbsoluteQuadricEstimator();
            estimator.setListener(this);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);
            estimator.setSingularityEnforced(true);

            final int numCams = estimator.getMinNumberOfRequiredCameras();
            double[] weights = new double[numCams];
            for (int i = 0; i < numCams; i++) {
                roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                        2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                        2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

                q = new Quaternion(roll, pitch, yaw);

                x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                cameraCenter = new InhomogeneousPoint3D(x, y, z);

                metricCamera = new PinholeCamera(metricIntrinsic, q,
                        cameraCenter);
                metricCamera.normalize();
                metricCameras.add(metricCamera);

                // transform camera
                projectiveCamera = transformation.transformAndReturnNew(
                        metricCamera);

                projectiveCameras.add(projectiveCamera);
                weights[i] = randomizer.nextDouble(MIN_WEIGHT, MAX_WEIGHT);
            }

            estimator.setCamerasAndWeights(projectiveCameras, weights);

            try {
                final DualAbsoluteQuadric estimatedDaq = estimator.estimate();
                estimatedDaq.normalize();
                final Matrix estimatedDaqMatrix = estimatedDaq.asMatrix();

                final DualAbsoluteQuadric estimatedDaq2 = new DualAbsoluteQuadric();
                estimator.estimate(estimatedDaq2);
                estimatedDaq2.normalize();

                if (!estimatedDaqMatrix.equals(estimatedDaq2.asMatrix(), ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(estimatedDaqMatrix.equals(estimatedDaq2.asMatrix(),
                        ABSOLUTE_ERROR));

                // check that DAQ has rank 3 (zero determinant)
                if (Math.abs(Utils.det(estimatedDaqMatrix)) > ABSOLUTE_ERROR) {
                    continue;
                }

                assertEquals(Utils.det(estimatedDaqMatrix), 0.0,
                        ABSOLUTE_ERROR);

                if (!projectiveDaqMatrix.equals(estimatedDaqMatrix,
                        ABSOLUTE_ERROR)) {
                    continue;
                }
                assertTrue(projectiveDaqMatrix.equals(estimatedDaqMatrix,
                        ABSOLUTE_ERROR));

                final ProjectiveTransformation3D estimatedTransformation =
                        estimatedDaq.getMetricToProjectiveTransformation();
                estimatedTransformation.normalize();
                final ProjectiveTransformation3D invEstimatedTransformation =
                        (ProjectiveTransformation3D) estimatedTransformation.
                                inverseAndReturnNew();

                // project estimated DAQ using projective cameras to obtain DIAC
                // and check that DIAC in projective stratum is equal to DIAC in
                // metric stratum
                Point3D previousEstimatedMetricCenter = null;
                Point3D previousMetricCenter = null;
                Point3D estimatedMetricCenter;
                Point3D metricCenter;
                Rotation3D previousEstimatedMetricRotation = null;
                Rotation3D previousMetricRotation = null;
                Rotation3D estimatedMetricRotation;
                Rotation3D metricRotation;
                double distanceEstimatedCenter;
                double distanceCenter;
                double previousScale = 1.0;
                double scale = 1.0;
                PinholeCamera estimatedMetricCamera;
                boolean anyFailed = false;
                for (int i = 0; i < numCams; i++) {
                    projectiveCamera = projectiveCameras.get(i);

                    final DualImageOfAbsoluteConic projectedProjectiveDiac =
                            new DualImageOfAbsoluteConic(projectiveCamera,
                                    estimatedDaq);
                    projectedProjectiveDiac.normalize();

                    final Matrix projectedProjectiveDiacMatrix =
                            projectedProjectiveDiac.asMatrix();

                    if (!metricDiacMatrix.equals(
                            projectedProjectiveDiacMatrix, ABSOLUTE_ERROR)) {
                        anyFailed = true;
                        continue;
                    }
                    assertTrue(metricDiacMatrix.equals(
                            projectedProjectiveDiacMatrix, ABSOLUTE_ERROR));

                    estimatedMetricCamera = invEstimatedTransformation.
                            transformAndReturnNew(projectiveCamera);

                    estimatedMetricCamera.decompose();
                    final PinholeCameraIntrinsicParameters estimatedIntrinsic =
                            estimatedMetricCamera.getIntrinsicParameters();

                    assertEquals(horizontalFocalLength,
                            estimatedIntrinsic.getHorizontalFocalLength(),
                            5 * LARGE_ABSOLUTE_ERROR);
                    assertEquals(verticalFocalLength,
                            estimatedIntrinsic.getVerticalFocalLength(),
                            5 * LARGE_ABSOLUTE_ERROR);
                    assertEquals(skewness, estimatedIntrinsic.getSkewness(),
                            5 * LARGE_ABSOLUTE_ERROR);
                    assertEquals(horizontalPrincipalPoint,
                            estimatedIntrinsic.getHorizontalPrincipalPoint(),
                            5 * LARGE_ABSOLUTE_ERROR);
                    assertEquals(verticalPrincipalPoint,
                            estimatedIntrinsic.getVerticalPrincipalPoint(),
                            5 * LARGE_ABSOLUTE_ERROR);

                    if (anyFailed) {
                        continue;
                    }

                    // check that when DAQ is successfully estimated, estimated
                    // metric cameras are in the metric stratum up to an
                    // arbitrary scale

                    metricCamera = metricCameras.get(i);
                    metricCamera.decompose();

                    estimatedMetricCenter = estimatedMetricCamera.
                            getCameraCenter();
                    metricCenter = metricCamera.getCameraCenter();
                    estimatedMetricRotation = estimatedMetricCamera.
                            getCameraRotation();
                    metricRotation = metricCamera.getCameraRotation();

                    if (i > 0) {
                        distanceEstimatedCenter = previousEstimatedMetricCenter.
                                distanceTo(estimatedMetricCenter);
                        distanceCenter = previousMetricCenter.distanceTo(
                                metricCenter);
                        scale = distanceEstimatedCenter / distanceCenter;

                        final Rotation3D diffEstimatedRotation =
                                estimatedMetricRotation.combineAndReturnNew(
                                        previousEstimatedMetricRotation.
                                                inverseRotationAndReturnNew());
                        final Rotation3D diffRotation = metricRotation.
                                combineAndReturnNew(previousMetricRotation.
                                        inverseRotationAndReturnNew());

                        final Matrix rot1 = diffEstimatedRotation.
                                asInhomogeneousMatrix();
                        final Matrix rot2 = diffRotation.asInhomogeneousMatrix();
                        assertTrue(rot1.equals(rot2, LARGE_ABSOLUTE_ERROR));
                    }

                    if (i > 1) {
                        assertEquals(scale, previousScale, 5 * LARGE_ABSOLUTE_ERROR);
                    }

                    previousEstimatedMetricCenter = estimatedMetricCenter;
                    previousMetricCenter = metricCenter;
                    previousScale = scale;

                    previousEstimatedMetricRotation = estimatedMetricRotation;
                    previousMetricRotation = metricRotation;
                }

                numSucceeded++;
                break;
            } catch (final DualAbsoluteQuadricEstimatorException ignore) {
            }
        }

        // sometimes if cameras are in degenerate configurations, DAQ estimation
        // can fail, for that reason we check that algorithm at least workes once
        // if we retry multiple times
        assertTrue(numSucceeded > 0);
    }

    // arbitrary skewness
    // principal point at origin
    // focal distance aspect ratio known (1.0)
    // singularity not enforced
    @Test
    public void testEstimate3() throws LockedException,
            AlgebraException, InvalidTransformationException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // create ground truth intrinsic parameters
        final double aspectRatio = 1.0;
        final double horizontalFocalLength =
                randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength = aspectRatio * horizontalFocalLength;
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final double horizontalPrincipalPoint = 0.0;
        final double verticalPrincipalPoint = 0.0;

        final PinholeCameraIntrinsicParameters metricIntrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final DualImageOfAbsoluteConic metricDiac = new DualImageOfAbsoluteConic(
                metricIntrinsic);
        metricDiac.normalize();

        // generate random projective transformation to transform ground
        // truth cameras
        final Matrix t = Matrix.createWithUniformRandomValues(
                ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        // ensure last element is not zero
        t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);

        final ProjectiveTransformation3D transformation =
                new ProjectiveTransformation3D(t);

        transformation.normalize();

        final DualAbsoluteQuadric projectiveDaq = new DualAbsoluteQuadric(
                transformation);
        projectiveDaq.normalize();

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
        final List<PinholeCamera> projectiveCameras =
                new ArrayList<>();

        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();
        estimator.setListener(this);
        estimator.setZeroSkewness(false);
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setFocalDistanceAspectRatioKnown(true);
        estimator.setFocalDistanceAspectRatio(1.0);
        estimator.setSingularityEnforced(false);

        final int numCams = estimator.getMinNumberOfRequiredCameras();
        final double[] weights = new double[numCams];
        for (int i = 0; i < numCams; i++) {
            roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            q = new Quaternion(roll, pitch, yaw);

            x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            cameraCenter = new InhomogeneousPoint3D(x, y, z);

            metricCamera = new PinholeCamera(metricIntrinsic, q,
                    cameraCenter);
            metricCamera.normalize();

            // transform camera
            projectiveCamera = transformation.transformAndReturnNew(
                    metricCamera);

            projectiveCameras.add(projectiveCamera);
            weights[i] = randomizer.nextDouble(MIN_WEIGHT, MAX_WEIGHT);
        }

        estimator.setCamerasAndWeights(projectiveCameras, weights);

        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        } catch (final DualAbsoluteQuadricEstimatorException ex) {
            fail("NotReadyException expected but not thrown");
        }
    }

    // arbitrary skewness
    // principal point at origin
    // focal distance aspect ratio known (1.0)
    // singularity enforced
    @Test
    public void testEstimate4() throws LockedException,
            AlgebraException, InvalidTransformationException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // create ground truth intrinsic parameters
        final double aspectRatio = 1.0;
        final double horizontalFocalLength =
                randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength = aspectRatio * horizontalFocalLength;
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final double horizontalPrincipalPoint = 0.0;
        final double verticalPrincipalPoint = 0.0;

        final PinholeCameraIntrinsicParameters metricIntrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final DualImageOfAbsoluteConic metricDiac = new DualImageOfAbsoluteConic(
                metricIntrinsic);
        metricDiac.normalize();

        // generate random projective transformation to transform ground
        // truth cameras
        final Matrix t = Matrix.createWithUniformRandomValues(
                ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        // ensure last element is not zero
        t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);

        final ProjectiveTransformation3D transformation =
                new ProjectiveTransformation3D(t);

        transformation.normalize();

        final DualAbsoluteQuadric projectiveDaq = new DualAbsoluteQuadric(
                transformation);
        projectiveDaq.normalize();

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
        final List<PinholeCamera> projectiveCameras =
                new ArrayList<>();

        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();
        estimator.setListener(this);
        estimator.setZeroSkewness(false);
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setFocalDistanceAspectRatioKnown(true);
        estimator.setFocalDistanceAspectRatio(1.0);
        estimator.setSingularityEnforced(true);

        final int numCams = estimator.getMinNumberOfRequiredCameras();
        final double[] weights = new double[numCams];
        for (int i = 0; i < numCams; i++) {
            roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            q = new Quaternion(roll, pitch, yaw);

            x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            cameraCenter = new InhomogeneousPoint3D(x, y, z);

            metricCamera = new PinholeCamera(metricIntrinsic, q,
                    cameraCenter);
            metricCamera.normalize();

            // transform camera
            projectiveCamera = transformation.transformAndReturnNew(
                    metricCamera);

            projectiveCameras.add(projectiveCamera);
            weights[i] = randomizer.nextDouble(MIN_WEIGHT, MAX_WEIGHT);
        }

        estimator.setCamerasAndWeights(projectiveCameras, weights);

        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        } catch (final DualAbsoluteQuadricEstimatorException ex) {
            fail("NotReadyException expected but not thrown");
        }
    }

    // zero skewness
    // principal point at origin
    // arbitrary focal distance aspect ratio
    // singularity not enforced
    @Test
    public void testEstimate5() throws LockedException,
            AlgebraException, InvalidTransformationException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // create ground truth intrinsic parameters
        final double aspectRatio = randomizer.nextDouble(MIN_ASPECT_RATIO,
                MAX_ASPECT_RATIO);
        final double horizontalFocalLength =
                randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength = aspectRatio * horizontalFocalLength;
        final double skewness = 0.0;
        final double horizontalPrincipalPoint = 0.0;
        final double verticalPrincipalPoint = 0.0;

        final PinholeCameraIntrinsicParameters metricIntrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final DualImageOfAbsoluteConic metricDiac = new DualImageOfAbsoluteConic(
                metricIntrinsic);
        metricDiac.normalize();

        // generate random projective transformation to transform ground
        // truth cameras
        final Matrix t = Matrix.createWithUniformRandomValues(
                ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        // ensure last element is not zero
        t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);

        final ProjectiveTransformation3D transformation =
                new ProjectiveTransformation3D(t);

        transformation.normalize();

        final DualAbsoluteQuadric projectiveDaq = new DualAbsoluteQuadric(
                transformation);
        projectiveDaq.normalize();

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
        final List<PinholeCamera> projectiveCameras =
                new ArrayList<>();

        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();
        estimator.setListener(this);
        estimator.setZeroSkewness(true);
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setFocalDistanceAspectRatioKnown(false);
        estimator.setFocalDistanceAspectRatio(aspectRatio);
        estimator.setSingularityEnforced(false);

        final int numCams = estimator.getMinNumberOfRequiredCameras();
        final double[] weights = new double[numCams];
        for (int i = 0; i < numCams; i++) {
            roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            q = new Quaternion(roll, pitch, yaw);

            x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            cameraCenter = new InhomogeneousPoint3D(x, y, z);

            metricCamera = new PinholeCamera(metricIntrinsic, q,
                    cameraCenter);
            metricCamera.normalize();

            // transform camera
            projectiveCamera = transformation.transformAndReturnNew(
                    metricCamera);

            projectiveCameras.add(projectiveCamera);
            weights[i] = randomizer.nextDouble(MIN_WEIGHT, MAX_WEIGHT);
        }

        estimator.setCamerasAndWeights(projectiveCameras, weights);

        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        } catch (final DualAbsoluteQuadricEstimatorException ex) {
            fail("NotReadyException expected but not thrown");
        }
    }

    // zero skewness
    // principal point at origin
    // arbitrary focal distance aspect ratio
    // singularity enforced
    @Test
    public void testEstimate6() throws LockedException,
            AlgebraException, InvalidTransformationException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // create ground truth intrinsic parameters
        final double aspectRatio = randomizer.nextDouble(MIN_ASPECT_RATIO,
                MAX_ASPECT_RATIO);
        final double horizontalFocalLength =
                randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength = aspectRatio * horizontalFocalLength;
        final double skewness = 0.0;
        final double horizontalPrincipalPoint = 0.0;
        final double verticalPrincipalPoint = 0.0;

        final PinholeCameraIntrinsicParameters metricIntrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final DualImageOfAbsoluteConic metricDiac = new DualImageOfAbsoluteConic(
                metricIntrinsic);
        metricDiac.normalize();

        // generate random projective transformation to transform ground
        // truth cameras
        final Matrix t = Matrix.createWithUniformRandomValues(
                ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        // ensure last element is not zero
        t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);

        final ProjectiveTransformation3D transformation =
                new ProjectiveTransformation3D(t);

        transformation.normalize();

        final DualAbsoluteQuadric projectiveDaq = new DualAbsoluteQuadric(
                transformation);
        projectiveDaq.normalize();

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
        final List<PinholeCamera> projectiveCameras =
                new ArrayList<>();

        final WeightedDualAbsoluteQuadricEstimator estimator =
                new WeightedDualAbsoluteQuadricEstimator();
        estimator.setListener(this);
        estimator.setZeroSkewness(true);
        estimator.setPrincipalPointAtOrigin(true);
        estimator.setFocalDistanceAspectRatioKnown(false);
        estimator.setFocalDistanceAspectRatio(aspectRatio);
        estimator.setSingularityEnforced(true);

        final int numCams = estimator.getMinNumberOfRequiredCameras();
        final double[] weights = new double[numCams];
        for (int i = 0; i < numCams; i++) {
            roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            q = new Quaternion(roll, pitch, yaw);

            x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            cameraCenter = new InhomogeneousPoint3D(x, y, z);

            metricCamera = new PinholeCamera(metricIntrinsic, q,
                    cameraCenter);
            metricCamera.normalize();

            // transform camera
            projectiveCamera = transformation.transformAndReturnNew(
                    metricCamera);

            projectiveCameras.add(projectiveCamera);
            weights[i] = randomizer.nextDouble(MIN_WEIGHT, MAX_WEIGHT);
        }

        estimator.setCamerasAndWeights(projectiveCameras, weights);

        try {
            estimator.estimate();
            fail("NotReadyException expected but not thrown");
        } catch (final NotReadyException ignore) {
        } catch (final DualAbsoluteQuadricEstimatorException ex) {
            fail("NotReadyException expected but not thrown");
        }
    }

    @Override
    public void onEstimateStart(final DualAbsoluteQuadricEstimator estimator) {
        assertTrue(estimator.isLocked());
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(final DualAbsoluteQuadricEstimator estimator) {
        assertTrue(estimator.isLocked());
        checkLocked(estimator);
    }

    @Override
    public void onEstimationProgressChange(
            final DualAbsoluteQuadricEstimator estimator, final float progress) {
        checkLocked(estimator);
    }

    private void checkLocked(final DualAbsoluteQuadricEstimator estimator) {
        final WeightedDualAbsoluteQuadricEstimator wEstimator =
                (WeightedDualAbsoluteQuadricEstimator) estimator;
        final double[] weights = new double[wEstimator.getCameras().size()];
        try {
            wEstimator.setWeights(weights);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            wEstimator.setCamerasAndWeights(wEstimator.getCameras(), weights);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            wEstimator.setMaxCameras(1);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            wEstimator.setSortWeightsEnabled(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            wEstimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception ignore) {
            fail("LockedException expected but not thrown");
        }
        try {
            wEstimator.setZeroSkewness(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            wEstimator.setPrincipalPointAtOrigin(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            wEstimator.setFocalDistanceAspectRatioKnown(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            wEstimator.setFocalDistanceAspectRatio(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            wEstimator.setSingularityEnforced(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            wEstimator.setEnforcedSingularityValidated(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            wEstimator.setDeterminantThreshold(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            wEstimator.setCameras(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }
}
