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

import com.irurueta.ar.calibration.DualImageOfAbsoluteConic;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidPinholeCameraIntrinsicParametersException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class KruppaDualImageOfAbsoluteConicEstimatorTest implements KruppaDualImageOfAbsoluteConicEstimatorListener {

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_ASPECT_RATIO = 0.5;
    private static final double MAX_ASPECT_RATIO = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 5.0;

    private static final int TIMES = 1000;

    private int estimateStart;
    private int estimateEnd;

    @Test
    void testConstructor() {
        // test empty constructor
        var estimator = new KruppaDualImageOfAbsoluteConicEstimator();

        // check default values
        assertEquals(0.0, estimator.getPrincipalPointX(), 0.0);
        assertEquals(0.0, estimator.getPrincipalPointY(), 0.0);
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertNull(estimator.getFundamentalMatrix());
        assertFalse(estimator.isReady());

        // test constructor with listener
        estimator = new KruppaDualImageOfAbsoluteConicEstimator(this);

        // check default values
        assertEquals(0.0, estimator.getPrincipalPointX(), 0.0);
        assertEquals(0.0, estimator.getPrincipalPointY(), 0.0);
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getFundamentalMatrix());
        assertFalse(estimator.isReady());

        // test constructor with fundamental matrix
        final FundamentalMatrix f = new FundamentalMatrix();
        estimator = new KruppaDualImageOfAbsoluteConicEstimator(f);

        // check default values
        assertEquals(0.0, estimator.getPrincipalPointX(), 0.0);
        assertEquals(0.0, estimator.getPrincipalPointY(), 0.0);
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getListener());
        assertSame(f, estimator.getFundamentalMatrix());
        assertTrue(estimator.isReady());

        // test constructor with fundamental matrix and listener
        estimator = new KruppaDualImageOfAbsoluteConicEstimator(f, this);

        // check default values
        assertEquals(0.0, estimator.getPrincipalPointX(), 0.0);
        assertEquals(0.0, estimator.getPrincipalPointY(), 0.0);
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);
        assertFalse(estimator.isLocked());
        assertSame(this, estimator.getListener());
        assertSame(f, estimator.getFundamentalMatrix());
        assertTrue(estimator.isReady());
    }

    @Test
    void testGetSetPrincipalPointX() throws LockedException {
        final var estimator = new KruppaDualImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(0.0, estimator.getPrincipalPointX(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var principalPointX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        estimator.setPrincipalPointX(principalPointX);

        // check correctness
        assertEquals(principalPointX, estimator.getPrincipalPointX(), 0.0);
    }

    @Test
    void testGetSetPrincipalPointY() throws LockedException {
        final var estimator = new KruppaDualImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(0.0, estimator.getPrincipalPointY(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var principalPointY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        estimator.setPrincipalPointY(principalPointY);

        // check correctness
        assertEquals(principalPointY, estimator.getPrincipalPointY(), 0.0);
    }

    @Test
    void testIsSetFocalDistanceAspectRatioKnown() throws LockedException {
        final var estimator = new KruppaDualImageOfAbsoluteConicEstimator();

        // check default value
        assertTrue(estimator.isFocalDistanceAspectRatioKnown());

        // set new value
        estimator.setFocalDistanceAspectRatioKnown(false);

        // check correctness
        assertFalse(estimator.isFocalDistanceAspectRatioKnown());
    }

    @Test
    void testGetSetFocalDistanceAspectRatio() throws LockedException {
        final var estimator = new KruppaDualImageOfAbsoluteConicEstimator();

        // check default value
        assertEquals(1.0, estimator.getFocalDistanceAspectRatio(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var aspectRatio = randomizer.nextDouble(MIN_ASPECT_RATIO, MAX_ASPECT_RATIO);

        estimator.setFocalDistanceAspectRatio(aspectRatio);

        // check correctness
        assertEquals(aspectRatio, estimator.getFocalDistanceAspectRatio(), 0.0);
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new KruppaDualImageOfAbsoluteConicEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetFundamentalMatrixAndIsReady() throws LockedException {
        final var estimator = new KruppaDualImageOfAbsoluteConicEstimator();

        // check default value
        assertNull(estimator.getFundamentalMatrix());
        assertFalse(estimator.isReady());

        // set new value
        final var f = new FundamentalMatrix();
        estimator.setFundamentalMatrix(f);

        // check correctness
        assertSame(f, estimator.getFundamentalMatrix());
        assertTrue(estimator.isReady());
    }

    @Test
    void testEstimateWithKnownAspectRatio() throws LockedException, NotReadyException,
            InvalidPinholeCameraIntrinsicParametersException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // create ground truth intrinsic parameters
            final var aspectRatio = 1.0;
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = aspectRatio * horizontalFocalLength;
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var rollLeft = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
            final var pitchLeft = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES));
            final var yawLeft = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
            final var xLeft = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var yLeft = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var zLeft = Math.toRadians(randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final var rollRight = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES));
            final var pitchRight = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yawRight = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES));
            final var xRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var yRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var zRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var qLeft = new Quaternion(rollLeft, pitchLeft, yawLeft);
            final var qRight = new Quaternion(rollRight, pitchRight, yawRight);
            final var leftCameraCenter = new InhomogeneousPoint3D(xLeft, yLeft, zLeft);
            final var rightCameraCenter = new InhomogeneousPoint3D(xRight, yRight, zRight);
            final var leftCamera = new PinholeCamera(intrinsic, qLeft, leftCameraCenter);
            final var rightCamera = new PinholeCamera(intrinsic, qRight, rightCameraCenter);

            final FundamentalMatrix fundamentalMatrix;
            try {
                fundamentalMatrix = new FundamentalMatrix(leftCamera, rightCamera);
            } catch (final InvalidPairOfCamerasException e) {
                continue;
            }

            final var estimator = new KruppaDualImageOfAbsoluteConicEstimator(fundamentalMatrix, this);
            estimator.setPrincipalPointX(horizontalPrincipalPoint);
            estimator.setPrincipalPointY(verticalPrincipalPoint);
            estimator.setFocalDistanceAspectRatioKnown(true);
            estimator.setFocalDistanceAspectRatio(aspectRatio);

            // check
            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertTrue(estimator.isReady());

            final DualImageOfAbsoluteConic diac;
            final var diac2 = new DualImageOfAbsoluteConic(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            try {
                diac = estimator.estimate();
                estimator.estimate(diac2);
            } catch (final KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }

            // check correctness
            assertEquals(2, estimateStart);
            assertEquals(2, estimateEnd);
            diac.normalize();
            diac2.normalize();

            assertTrue(diac.asMatrix().equals(diac2.asMatrix(), ABSOLUTE_ERROR));

            final var intrinsic2 = diac.getIntrinsicParameters();
            final var intrinsic3 = diac2.getIntrinsicParameters();

            if (Math.abs(intrinsic2.getHorizontalFocalLength() - horizontalFocalLength) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(horizontalFocalLength, intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength, intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2.getSkewness(), ABSOLUTE_ERROR);

            assertEquals(horizontalFocalLength, intrinsic3.getHorizontalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength, intrinsic3.getVerticalFocalLength(), ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, intrinsic3.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, intrinsic3.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic3.getSkewness(), ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateWithUnknownAspectRatio() throws LockedException, NotReadyException,
            InvalidPinholeCameraIntrinsicParametersException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            // create ground truth intrinsic parameters
            final var aspectRatio = randomizer.nextDouble(MIN_ASPECT_RATIO, MAX_ASPECT_RATIO);
            final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            final var verticalFocalLength = aspectRatio * horizontalFocalLength;
            final var skewness = 0.0;
            final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var verticalPrincipalPoint = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                    horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

            final var rollLeft = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES));
            final var pitchLeft = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yawLeft = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES));
            final var xLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var yLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var zLeft = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var rollRight = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES));
            final var pitchRight = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yawRight = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES));
            final var xRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var yRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var zRight = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var qLeft = new Quaternion(rollLeft, pitchLeft, yawLeft);
            final var qRight = new Quaternion(rollRight, pitchRight, yawRight);
            final var leftCameraCenter = new InhomogeneousPoint3D(xLeft, yLeft, zLeft);
            final var rightCameraCenter = new InhomogeneousPoint3D(xRight, yRight, zRight);
            final var leftCamera = new PinholeCamera(intrinsic, qLeft, leftCameraCenter);
            final var rightCamera = new PinholeCamera(intrinsic, qRight, rightCameraCenter);

            final FundamentalMatrix fundamentalMatrix;
            try {
                fundamentalMatrix = new FundamentalMatrix(leftCamera, rightCamera);
            } catch (final InvalidPairOfCamerasException e) {
                continue;
            }

            final var estimator = new KruppaDualImageOfAbsoluteConicEstimator(fundamentalMatrix, this);
            estimator.setPrincipalPointX(horizontalPrincipalPoint);
            estimator.setPrincipalPointY(verticalPrincipalPoint);
            estimator.setFocalDistanceAspectRatioKnown(false);

            // check
            reset();
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);
            assertTrue(estimator.isReady());

            final DualImageOfAbsoluteConic diac;
            final var diac2 = new DualImageOfAbsoluteConic(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            try {
                diac = estimator.estimate();
                estimator.estimate(diac2);
            } catch (final KruppaDualImageOfAbsoluteConicEstimatorException e) {
                continue;
            }

            // check correctness
            assertEquals(2, estimateStart);
            assertEquals(2, estimateEnd);
            diac.normalize();
            diac2.normalize();

            assertTrue(diac.asMatrix().equals(diac2.asMatrix(), ABSOLUTE_ERROR));

            final var intrinsic2 = diac.getIntrinsicParameters();
            final var intrinsic3 = diac2.getIntrinsicParameters();

            if (Math.abs(intrinsic2.getHorizontalFocalLength() - horizontalFocalLength) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(horizontalFocalLength, intrinsic2.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2.getVerticalFocalLength() - verticalFocalLength) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(verticalFocalLength, intrinsic2.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2.getHorizontalPrincipalPoint() - horizontalPrincipalPoint) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(horizontalPrincipalPoint, intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic2.getSkewness(), ABSOLUTE_ERROR);

            assertEquals(horizontalFocalLength, intrinsic3.getHorizontalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength, intrinsic3.getVerticalFocalLength(), LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint, intrinsic3.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint, intrinsic3.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
            assertEquals(0.0, intrinsic3.getSkewness(), ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    private void reset() {
        estimateStart = estimateEnd = 0;
    }

    @Override
    public void onEstimateStart(final KruppaDualImageOfAbsoluteConicEstimator estimator) {
        estimateStart++;
        checkLocked(estimator);
    }

    @Override
    public void onEstimateEnd(final KruppaDualImageOfAbsoluteConicEstimator estimator) {
        estimateEnd++;
        checkLocked(estimator);
    }

    private static void checkLocked(final KruppaDualImageOfAbsoluteConicEstimator estimator) {
        assertTrue(estimator.isLocked());

        assertThrows(LockedException.class, () -> estimator.setPrincipalPointX(0.0));
        assertThrows(LockedException.class, () -> estimator.setPrincipalPointY(0.0));
        assertThrows(LockedException.class, () -> estimator.setFocalDistanceAspectRatioKnown(true));
        assertThrows(LockedException.class, () -> estimator.setFocalDistanceAspectRatio(1.0));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setFundamentalMatrix(null));
        assertThrows(LockedException.class, estimator::estimate);
        assertThrows(LockedException.class, () -> estimator.estimate(null));
    }
}
