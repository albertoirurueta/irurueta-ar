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
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PROSACDualAbsoluteQuadricRobustEstimatorTest implements
        DualAbsoluteQuadricRobustEstimatorListener {

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    private static final int TIMES = 1000;

    private static final int NUM_CAMS = 100;

    private static final int PERCENTAGE_OUTLIERS = 10;

    private static final double STD_ERROR = 1.0;

    @Test
    public void testConstructor() {
        // test empty constructor
        PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(estimator.getThreshold(),
                PROSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD,
                0.0);

        // test with listener
        estimator = new PROSACDualAbsoluteQuadricRobustEstimator(this);

        // check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(estimator.getThreshold(),
                PROSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD,
                0.0);

        // test with cameras
        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        estimator = new PROSACDualAbsoluteQuadricRobustEstimator(cameras);

        // check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(estimator.getThreshold(),
                PROSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD,
                0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACDualAbsoluteQuadricRobustEstimator(
                    new ArrayList<PinholeCamera>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with cameras and listener
        estimator = new PROSACDualAbsoluteQuadricRobustEstimator(cameras, this);

        // check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertNull(estimator.getQualityScores());
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(estimator.getThreshold(),
                PROSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD,
                0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACDualAbsoluteQuadricRobustEstimator(
                    new ArrayList<PinholeCamera>(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with quality scores
        final double[] qualityScores = new double[2];
        estimator = new PROSACDualAbsoluteQuadricRobustEstimator(qualityScores);

        // check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(estimator.getThreshold(),
                PROSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD,
                0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACDualAbsoluteQuadricRobustEstimator(
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with quality scores and listener
        estimator = new PROSACDualAbsoluteQuadricRobustEstimator(qualityScores,
                this);

        // check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertNull(estimator.getCameras());
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(estimator.getThreshold(),
                PROSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD,
                0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACDualAbsoluteQuadricRobustEstimator(
                    new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with quality scores and cameras
        estimator = new PROSACDualAbsoluteQuadricRobustEstimator(cameras,
                qualityScores);

        // check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(estimator.getThreshold(),
                PROSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD,
                0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACDualAbsoluteQuadricRobustEstimator(
                    new ArrayList<PinholeCamera>(), qualityScores);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACDualAbsoluteQuadricRobustEstimator(
                    cameras, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test with quality scores, cameras and listener
        estimator = new PROSACDualAbsoluteQuadricRobustEstimator(cameras,
                qualityScores, this);

        // check correctness
        assertTrue(estimator.isZeroSkewness());
        assertTrue(estimator.isPrincipalPointAtOrigin());
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);
        assertTrue(estimator.isSingularityEnforced());
        assertTrue(estimator.isEnforcedSingularityValidated());
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getProgressDelta(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);
        assertSame(estimator.getCameras(), cameras);
        assertEquals(estimator.getMinNumberOfRequiredCameras(), 2);
        assertTrue(estimator.areValidConstraints());
        assertTrue(estimator.isReady());
        assertSame(estimator.getQualityScores(), qualityScores);
        assertEquals(estimator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(estimator.getThreshold(),
                PROSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD,
                0.0);

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new PROSACDualAbsoluteQuadricRobustEstimator(
                    new ArrayList<PinholeCamera>(), qualityScores, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new PROSACDualAbsoluteQuadricRobustEstimator(
                    cameras, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertEquals(estimator.getThreshold(),
                PROSACDualAbsoluteQuadricRobustEstimator.DEFAULT_THRESHOLD,
                0.0);

        // set new value
        estimator.setThreshold(0.5);

        // check correctness
        assertEquals(estimator.getThreshold(), 0.5, 0.0);
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertNull(estimator.getQualityScores());

        // set new value
        final double[] qualityScores = new double[2];
        estimator.setQualityScores(qualityScores);

        // check correctness
        assertSame(estimator.getQualityScores(), qualityScores);
    }

    @Test
    public void testIsSetZeroSkewness() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertTrue(estimator.isZeroSkewness());

        // set new value
        estimator.setZeroSkewness(false);

        // check correctness
        assertFalse(estimator.isZeroSkewness());
    }

    @Test
    public void testIsSetPrincipalPointAtOrigin() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertTrue(estimator.isPrincipalPointAtOrigin());

        // set new value
        estimator.setPrincipalPointAtOrigin(false);

        // check correctness
        assertFalse(estimator.isPrincipalPointAtOrigin());
    }

    @Test
    public void testIsFocalDistanceAspectRatioKnown() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());

        // set new value
        estimator.setFocalDistanceAspectRatioKnown(false);

        // check correctness
        assertFalse(estimator.isFocalDistanceAspectRatioKnown());
    }

    @Test
    public void testGetSetFocalDistanceAspectRatio() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertEquals(estimator.getFocalDistanceAspectRatio(), 1.0, 0.0);

        // set new value
        estimator.setFocalDistanceAspectRatio(0.5);

        // check correctness
        assertEquals(estimator.getFocalDistanceAspectRatio(), 0.5, 0.0);
    }

    @Test
    public void testIsSetSingularityEnforced() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertTrue(estimator.isSingularityEnforced());

        // set new value
        estimator.setSingularityEnforced(false);

        // check correctness
        assertFalse(estimator.isSingularityEnforced());
    }

    @Test
    public void testIsSetEnforcedSingularityValidated() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertTrue(estimator.isEnforcedSingularityValidated());

        // set new value
        estimator.setEnforcedSingularityValidated(false);

        // check correctness
        assertFalse(estimator.isEnforcedSingularityValidated());
    }

    @Test
    public void testGetSetDeterminantThreshold() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertEquals(estimator.getDeterminantThreshold(),
                DualAbsoluteQuadricEstimator.DEFAULT_DETERMINANT_THRESHOLD,
                0.0);

        // set new value
        estimator.setDeterminantThreshold(1e-3);

        // check correctness
        assertEquals(estimator.getDeterminantThreshold(), 1e-3, 0.0);
    }

    @Test
    public void testGetSetListenerAndIsListenerAvailable()
            throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // check default value
        assertNull(estimator.getListener());
        assertFalse(estimator.isListenerAvailable());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(estimator.getListener(), this);
        assertTrue(estimator.isListenerAvailable());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertEquals(estimator.getProgressDelta(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_PROGRESS_DELTA, 0.0);

        // set new value
        estimator.setProgressDelta(0.1f);

        // check correctness
        assertEquals(estimator.getProgressDelta(), 0.1f, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertEquals(estimator.getConfidence(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_CONFIDENCE, 0.0);

        // set new value
        estimator.setConfidence(0.8);

        // check correctness
        assertEquals(estimator.getConfidence(), 0.8, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertEquals(estimator.getMaxIterations(),
                DualAbsoluteQuadricRobustEstimator.DEFAULT_MAX_ITERATIONS);

        // set new value
        estimator.setMaxIterations(100);

        // check correctness
        assertEquals(estimator.getMaxIterations(), 100);

        // Force IllegalArgumentException
        try {
            estimator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetCameras() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // initial value
        assertNull(estimator.getCameras());

        // set new value
        final List<PinholeCamera> cameras = new ArrayList<>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());
        estimator.setCameras(cameras);

        // check correctness
        assertSame(estimator.getCameras(), cameras);

        // Force IllegalArgumentException
        try {
            estimator.setCameras(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setCameras(new ArrayList<PinholeCamera>());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetMinNumberOfRequiredCameras() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

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
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

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
    public void testIsReady() throws LockedException {
        final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                new PROSACDualAbsoluteQuadricRobustEstimator();

        // check default value
        assertNull(estimator.getCameras());
        assertTrue(estimator.areValidConstraints());
        assertFalse(estimator.isReady());

        final List<PinholeCamera> cameras = new ArrayList<>();
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
    public void testEstimate() throws AlgebraException,
            InvalidTransformationException, LockedException, NotReadyException,
            CameraException, NotAvailableException {

        int numSucceeded = 0;
        for (int times = 0; times < TIMES; times++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            // create ground truth intrinsic parameters
            final double aspectRatio = 1.0;
            final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
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

            final PROSACDualAbsoluteQuadricRobustEstimator estimator =
                    new PROSACDualAbsoluteQuadricRobustEstimator();
            estimator.setListener(this);
            estimator.setZeroSkewness(true);
            estimator.setPrincipalPointAtOrigin(true);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(1.0);
            estimator.setSingularityEnforced(false);

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, STD_ERROR);
            final boolean[] inliers = new boolean[NUM_CAMS];
            final double[] qualityScores = new double[NUM_CAMS];
            for (int i = 0; i < NUM_CAMS; i++) {
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

                if (randomizer.nextInt(0, 100) < PERCENTAGE_OUTLIERS) {
                    // outlier (add error to metric intrinsics)
                    inliers[i] = false;
                    final double errorHorizontalFocalLength =
                            errorRandomizer.nextDouble();
                    final double errorAspectRatio = errorRandomizer.nextDouble();
                    final double errorSkewness = errorRandomizer.nextDouble();
                    final double errorHorizontalPrincipalPoint =
                            errorRandomizer.nextDouble();
                    final double errorVerticalPrincipalPoint =
                            errorRandomizer.nextDouble();

                    final double avgAbsError = (Math.abs(errorHorizontalFocalLength) +
                            Math.abs(errorAspectRatio) +
                            Math.abs(errorSkewness) +
                            Math.abs(errorHorizontalPrincipalPoint) +
                            Math.abs(errorVerticalPrincipalPoint)) / 5.0;
                    qualityScores[i] = 1.0 / (1.0 + avgAbsError);

                    final double outlierHorizontalFocalLength =
                            horizontalFocalLength + errorHorizontalFocalLength;
                    final double outlierAspectRatio = aspectRatio + errorAspectRatio;
                    final double outlierVerticalFocalLength =
                            outlierAspectRatio * outlierHorizontalFocalLength;
                    final double outlierSkewness = skewness + errorSkewness;
                    final double outlierHorizontalPrincipalPoint =
                            horizontalPrincipalPoint +
                                    errorHorizontalPrincipalPoint;
                    final double outlierVerticalPrincipalPoint =
                            verticalPrincipalPoint + errorVerticalPrincipalPoint;

                    final PinholeCameraIntrinsicParameters outlierMetricIntrinsic =
                            new PinholeCameraIntrinsicParameters(
                                    outlierHorizontalFocalLength,
                                    outlierVerticalFocalLength,
                                    outlierHorizontalPrincipalPoint,
                                    outlierVerticalPrincipalPoint, outlierSkewness);

                    metricCamera = new PinholeCamera(outlierMetricIntrinsic, q,
                            cameraCenter);
                } else {
                    // inlier
                    inliers[i] = true;
                    qualityScores[i] = 1.0;
                    metricCamera = new PinholeCamera(metricIntrinsic, q,
                            cameraCenter);
                }
                metricCamera.normalize();
                metricCameras.add(metricCamera);

                // transform camera
                projectiveCamera = transformation.transformAndReturnNew(
                        metricCamera);

                projectiveCameras.add(projectiveCamera);
            }

            estimator.setCameras(projectiveCameras);
            estimator.setQualityScores(qualityScores);

            try {
                final DualAbsoluteQuadric estimatedDaq = estimator.estimate();
                estimatedDaq.normalize();
                final Matrix estimatedDaqMatrix = estimatedDaq.asMatrix();

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
                double previousScale = 1.0;
                double scale = 1.0;
                boolean anyFailed = false;
                int count = 0;
                PinholeCamera estimatedMetricCamera;
                for (int i = 0; i < NUM_CAMS; i++) {
                    if (!inliers[i]) {
                        continue;
                    }

                    projectiveCamera = projectiveCameras.get(i);

                    final DualImageOfAbsoluteConic projectedProjectiveDiac =
                            new DualImageOfAbsoluteConic(projectiveCamera,
                                    estimatedDaq);
                    projectedProjectiveDiac.normalize();

                    final Matrix projectedProjectiveDiacMatrix =
                            projectedProjectiveDiac.asMatrix();

                    if (!metricDiacMatrix.equals(projectedProjectiveDiacMatrix,
                            ABSOLUTE_ERROR)) {
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

                    if (count > 0) {
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

                    if (count > 1) {
                        if (Math.abs(scale - previousScale) > 5 * LARGE_ABSOLUTE_ERROR) {
                            count++;
                            continue;
                        }
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
            }
        }

        // sometimes if cameras are in degenerate configurations, DAQ estimation
        // can fail, for that reason we check that algorithm at least works one
        // if we retry multiple times.
        assertTrue(numSucceeded > 0);
    }


    @Override
    public void onEstimateStart(final DualAbsoluteQuadricRobustEstimator estimator) {
        assertTrue(estimator.isLocked());
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(final DualAbsoluteQuadricRobustEstimator estimator) {
        assertTrue(estimator.isLocked());
        checkLocked(estimator);
    }

    @Override
    public void onEstimateNextIteration(
            final DualAbsoluteQuadricRobustEstimator estimator, final int iteration) {
        checkLocked(estimator);
    }

    @Override
    public void onEstimateProgressChange(
            final DualAbsoluteQuadricRobustEstimator estimator, final float progress) {
        checkLocked(estimator);
    }

    private void checkLocked(final DualAbsoluteQuadricRobustEstimator estimator) {
        final PROSACDualAbsoluteQuadricRobustEstimator prosacEstimator =
                (PROSACDualAbsoluteQuadricRobustEstimator) estimator;

        try {
            prosacEstimator.setThreshold(1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.setQualityScores(new double[2]);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.setZeroSkewness(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.setPrincipalPointAtOrigin(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.setFocalDistanceAspectRatioKnown(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.setFocalDistanceAspectRatio(2.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.setSingularityEnforced(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.setEnforcedSingularityValidated(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.setDeterminantThreshold(1e-3);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.setProgressDelta(0.1f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.setConfidence(0.8);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.setMaxIterations(100);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.setCameras(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            prosacEstimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}
