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
package com.irurueta.ar.slam;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.SerializationHelper;
import com.irurueta.ar.slam.BaseSlamEstimator.BaseSlamEstimatorListener;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.numerical.signal.processing.KalmanFilter;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Arrays;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class AbsoluteOrientationSlamEstimatorTest implements
        BaseSlamEstimatorListener<AbsoluteOrientationSlamCalibrationData> {

    private static final int TIMES = 50;
    private static final double ABSOLUTE_ERROR = 1e-8;

    // conversion from milliseconds to nanoseconds
    private static final int MILLIS_TO_NANOS = 1000000;

    // time between samples expressed in nanoseconds (a typical sensor in Android
    // delivers a sample every 20ms)
    private static final int DELTA_NANOS = 20000000; // 0.02 seconds
    private static final double DELTA_SECONDS = 0.02; // 0.2 seconds;

    // gain below is only valid when measure variance is 1e-8 and process noise
    // variance is 1e-6. This is the gain component of the Kalman filter for
    // those error covariances. This gain will change for other values
    private static final double VELOCITY_GAIN = 0.019991983014891204; //0.1923075055475187;

    private static final int REPEAT_TIMES = 5;
    private static final int N_PREDICTION_SAMPLES = 10000;

    private static final double ACCELERATION_NOISE_STANDARD_DEVIATION = 1e-4;
    private static final double ANGULAR_SPEED_NOISE_STANDARD_DEVIATION = 1e-4;
    private static final double ORIENTATION_NOISE_STANDARD_DEVIATION = 1e-4;

    private static final double REQUIRED_PREDICTION_SUCCESS_RATE = 0.5;
    private static final double REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE = 0.2;

    private static final float MIN_CALIBRATION_OFFSET = -1e-4f;
    private static final float MAX_CALIBRATION_OFFSET = 1e-4f;

    private static final int MAX_CALIBRATION_SAMPLES = 10000;

    private int fullSampleReceived;
    private int fullSampleProcessed;
    private int correctWithPositionMeasure;
    private int correctedWithPositionMeasure;

    @Test
    public void testConstructor() throws WrongSizeException {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        // check initial values
        assertNull(estimator.getListener());
        assertNull(estimator.getCalibrationData());
        assertEquals(0.0, estimator.getStatePositionX(), 0.0);
        assertEquals(0.0, estimator.getStatePositionY(), 0.0);
        assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getStatePosition(), 0.0);
        final double[] position = new double[3];
        estimator.getStatePosition(position);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, position, 0.0);

        assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getStateVelocity(), 0.0);
        final double[] velocity = new double[3];
        estimator.getStateVelocity(velocity);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, velocity, 0.0);

        assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
        assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
        assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getStateAcceleration(), 0.0);
        final double[] acceleration = new double[3];
        estimator.getStateAcceleration(acceleration);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, acceleration, 0.0);

        assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
        assertArrayEquals(new double[]{1.0, 0.0, 0.0, 0.0}, estimator.getStateQuaternionArray(), 0.0);
        final double[] quaternionArray = new double[4];
        estimator.getStateQuaternionArray(quaternionArray);
        assertArrayEquals(new double[]{1.0, 0.0, 0.0, 0.0}, quaternionArray, 0.0);

        Quaternion q = estimator.getStateQuaternion();
        assertEquals(1.0, q.getA(), 0.0);
        assertEquals(0.0, q.getB(), 0.0);
        assertEquals(0.0, q.getC(), 0.0);
        assertEquals(0.0, q.getD(), 0.0);

        q = new Quaternion();
        estimator.getStateQuaternion(q);
        assertEquals(1.0, q.getA(), 0.0);
        assertEquals(0.0, q.getB(), 0.0);
        assertEquals(0.0, q.getC(), 0.0);
        assertEquals(0.0, q.getD(), 0.0);

        assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
        assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
        assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getStateAngularSpeed(), 0.0);
        final double[] angularSpeed = new double[3];
        estimator.getStateAngularSpeed(angularSpeed);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, angularSpeed, 0.0);

        assertFalse(estimator.hasError());
        assertTrue(estimator.isAccumulationEnabled());
        assertEquals(-1, estimator.getAccelerometerTimestampNanos());
        assertEquals(-1, estimator.getGyroscopeTimestampNanos());
        assertEquals(0, estimator.getAccumulatedAccelerometerSamples());
        assertEquals(0, estimator.getAccumulatedGyroscopeSamples());
        assertFalse(estimator.isAccelerometerSampleReceived());
        assertFalse(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());

        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getAccumulatedAccelerationSample(), 0.0);
        final double[] accumAcc = new double[3];
        estimator.getAccumulatedAccelerationSample(accumAcc);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, accumAcc, 0.0);

        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getAccumulatedAngularSpeedSample(), 0.0);
        final double[] accumAngularSpeed = new double[3];
        estimator.getAccumulatedAngularSpeedSample(accumAngularSpeed);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, accumAngularSpeed, 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());

        assertEquals(estimator.getPositionCovarianceMatrix(),
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        KalmanFilter.DEFAULT_MEASUREMENT_NOISE_VARIANCE));

        assertEquals(-1, estimator.getOrientationTimestampNanos());
        assertTrue(estimator.getAccumulatedOrientation() instanceof Quaternion);
        Quaternion accumulatedOrientation = (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(1.0, accumulatedOrientation.getA(), ABSOLUTE_ERROR);
        assertEquals(0.0, accumulatedOrientation.getB(), ABSOLUTE_ERROR);
        assertEquals(0.0, accumulatedOrientation.getC(), ABSOLUTE_ERROR);
        assertEquals(0.0, accumulatedOrientation.getD(), ABSOLUTE_ERROR);
        accumulatedOrientation = new Quaternion();
        estimator.getAccumulatedOrientation(accumulatedOrientation);
        assertEquals(1.0, accumulatedOrientation.getA(), ABSOLUTE_ERROR);
        assertEquals(0.0, accumulatedOrientation.getB(), ABSOLUTE_ERROR);
        assertEquals(0.0, accumulatedOrientation.getC(), ABSOLUTE_ERROR);
        assertEquals(0.0, accumulatedOrientation.getD(), ABSOLUTE_ERROR);

        assertEquals(0, estimator.getAccumulatedOrientationSamples());
        assertFalse(estimator.isOrientationSampleReceived());
        assertNotNull(estimator.getStateCovariance());
    }

    @Test
    public void testGetSetListener() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        // initial value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testGetSetCalibrationData() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        // initial value
        assertNull(estimator.getCalibrationData());

        // set new value
        final AbsoluteOrientationSlamCalibrationData data = new AbsoluteOrientationSlamCalibrationData();
        estimator.setCalibrationData(data);

        // check correctness
        assertSame(data, estimator.getCalibrationData());
    }

    @Test
    public void testResetPosition() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        estimator.resetPosition();

        assertEquals(0.0, estimator.getStatePositionX(), 0.0);
        assertEquals(0.0, estimator.getStatePositionY(), 0.0);
        assertEquals(0.0, estimator.getStatePositionZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testResetVelocity() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        estimator.resetVelocity();

        assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testResetPositionAndVelocity() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        estimator.resetPositionAndVelocity();

        assertEquals(0.0, estimator.getStatePositionX(), 0.0);
        assertEquals(0.0, estimator.getStatePositionY(), 0.0);
        assertEquals(0.0, estimator.getStatePositionZ(), 0.0);

        assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testResetAcceleration() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        estimator.resetAcceleration();

        assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
        assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
        assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testResetOrientation() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        estimator.resetOrientation();

        assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testResetAngularSpeed() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        estimator.resetAngularSpeed();

        assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
        assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
        assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testReset() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        estimator.reset();

        assertEquals(0.0, estimator.getStatePositionX(), 0.0);
        assertEquals(0.0, estimator.getStatePositionY(), 0.0);
        assertEquals(0.0, estimator.getStatePositionZ(), 0.0);

        assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);

        assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
        assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
        assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);

        assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);

        assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
        assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
        assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testGetStatePositionX() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(0.0, estimator.getStatePositionX(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStatePositionX = value;

        // check correctness
        assertEquals(value, estimator.getStatePositionX(), 0.0);
    }

    @Test
    public void testGetStatePositionY() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(0.0, estimator.getStatePositionY(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStatePositionY = value;

        // check correctness
        assertEquals(value, estimator.getStatePositionY(), 0.0);
    }

    @Test
    public void testGetStatePositionZ() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(0.0, estimator.getStatePositionZ(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStatePositionZ = value;

        // check correctness
        assertEquals(value, estimator.getStatePositionZ(), 0.0);
    }

    @Test
    public void testGetStatePosition() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        estimator.mStatePositionX = valueX;
        estimator.mStatePositionY = valueY;
        estimator.mStatePositionZ = valueZ;

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, estimator.getStatePosition(), 0.0);

        final double[] position = new double[3];
        estimator.getStatePosition(position);

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, position, 0.0);

        // Force IllegalArgumentException
        final double[] wrong = new double[4];
        try {
            estimator.getStatePosition(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetStateVelocityX() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(0.0, estimator.getStateVelocityX(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStateVelocityX = value;

        // check correctness
        assertEquals(value, estimator.getStateVelocityX(), 0.0);
    }

    @Test
    public void testGetStateVelocityY() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(0.0, estimator.getStateVelocityY(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStateVelocityY = value;

        // check correctness
        assertEquals(value, estimator.getStateVelocityY(), 0.0);
    }

    @Test
    public void testGetStateVelocityZ() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStateVelocityZ = value;

        // check correctness
        assertEquals(value, estimator.getStateVelocityZ(), 0.0);
    }

    @Test
    public void testGetStateVelocity() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        estimator.mStateVelocityX = valueX;
        estimator.mStateVelocityY = valueY;
        estimator.mStateVelocityZ = valueZ;

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, estimator.getStateVelocity(), 0.0);

        final double[] velocity = new double[3];
        estimator.getStateVelocity(velocity);

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, velocity, 0.0);

        // Force IllegalArgumentException
        final double[] wrong = new double[4];
        try {
            estimator.getStateVelocity(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetStateAccelerationX() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStateAccelerationX = value;

        // check correctness
        assertEquals(value, estimator.getStateAccelerationX(), 0.0);
    }

    @Test
    public void testGetStateAccelerationY() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStateAccelerationY = value;

        // check correctness
        assertEquals(value, estimator.getStateAccelerationY(), 0.0);
    }

    @Test
    public void testGetStateAccelerationZ() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStateAccelerationZ = value;

        // check correctness
        assertEquals(value, estimator.getStateAccelerationZ(), 0.0);
    }

    @Test
    public void testGetStateAcceleration() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        estimator.mStateAccelerationX = valueX;
        estimator.mStateAccelerationY = valueY;
        estimator.mStateAccelerationZ = valueZ;

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, estimator.getStateAcceleration(), 0.0);

        final double[] acceleration = new double[3];
        estimator.getStateAcceleration(acceleration);

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, acceleration, 0.0);

        // Force IllegalArgumentException
        final double[] wrong = new double[4];
        try {
            estimator.getStateAcceleration(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetStateQuaternionA() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateQuaternionA = value;

        // check correctness
        assertEquals(value, estimator.getStateQuaternionA(), 0.0);
    }

    @Test
    public void testGetStateQuaternionB() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateQuaternionB = value;

        // check correctness
        assertEquals(value, estimator.getStateQuaternionB(), 0.0);
    }

    @Test
    public void testGetStateQuaternionC() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateQuaternionC = value;

        // check correctness
        assertEquals(value, estimator.getStateQuaternionC(), 0.0);
    }

    @Test
    public void testGetStateQuaternionD() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateQuaternionD = value;

        // check correctness
        assertEquals(value, estimator.getStateQuaternionD(), 0.0);
    }

    @Test
    public void testGetStateQuaternionArray() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertArrayEquals(new double[]{1.0, 0.0, 0.0, 0.0}, estimator.getStateQuaternionArray(), 0.0);

        // set new values
        final double valueA = randomizer.nextDouble();
        final double valueB = randomizer.nextDouble();
        final double valueC = randomizer.nextDouble();
        final double valueD = randomizer.nextDouble();
        estimator.mStateQuaternionA = valueA;
        estimator.mStateQuaternionB = valueB;
        estimator.mStateQuaternionC = valueC;
        estimator.mStateQuaternionD = valueD;

        // check correctness
        assertArrayEquals(new double[]{valueA, valueB, valueC, valueD},
                estimator.getStateQuaternionArray(), 0.0);

        final double[] quaternion = new double[4];
        estimator.getStateQuaternionArray(quaternion);

        assertArrayEquals(new double[]{valueA, valueB, valueC, valueD}, quaternion, 0.0);

        // Force IllegalArgumentException
        final double[] wrong = new double[5];
        try {
            estimator.getStateQuaternionArray(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetStateQuaternion() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        Quaternion q = estimator.getStateQuaternion();

        assertEquals(1.0, q.getA(), 0.0);
        assertEquals(0.0, q.getB(), 0.0);
        assertEquals(0.0, q.getC(), 0.0);
        assertEquals(0.0, q.getD(), 0.0);

        // set new values
        final double valueA = randomizer.nextDouble();
        final double valueB = randomizer.nextDouble();
        final double valueC = randomizer.nextDouble();
        final double valueD = randomizer.nextDouble();
        estimator.mStateQuaternionA = valueA;
        estimator.mStateQuaternionB = valueB;
        estimator.mStateQuaternionC = valueC;
        estimator.mStateQuaternionD = valueD;

        // check correctness
        q = estimator.getStateQuaternion();

        assertEquals(valueA, q.getA(), 0.0);
        assertEquals(valueB, q.getB(), 0.0);
        assertEquals(valueC, q.getC(), 0.0);
        assertEquals(valueD, q.getD(), 0.0);

        q = new Quaternion();
        estimator.getStateQuaternion(q);

        assertEquals(valueA, q.getA(), 0.0);
        assertEquals(valueB, q.getB(), 0.0);
        assertEquals(valueC, q.getC(), 0.0);
        assertEquals(valueD, q.getD(), 0.0);
    }

    @Test
    public void testGetStateAngularSpeedX() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateAngularSpeedX = value;

        // check correctness
        assertEquals(value, estimator.getStateAngularSpeedX(), 0.0);
    }

    @Test
    public void testGetStateAngularSpeedY() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateAngularSpeedY = value;

        // check correctness
        assertEquals(value, estimator.getStateAngularSpeedY(), 0.0);
    }

    @Test
    public void testGetStateAngularSpeedZ() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateAngularSpeedZ = value;

        // check correctness
        assertEquals(value, estimator.getStateAngularSpeedZ(), 0.0);
    }

    @Test
    public void testGetStateAngularSpeed() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getStateAngularSpeed(), 0.0);

        // set new value
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        estimator.mStateAngularSpeedX = valueX;
        estimator.mStateAngularSpeedY = valueY;
        estimator.mStateAngularSpeedZ = valueZ;

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, estimator.getStateAngularSpeed(), 0.0);

        final double[] angularSpeed = new double[3];
        estimator.getStateAngularSpeed(angularSpeed);

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, angularSpeed, 0.0);

        // Force IllegalArgumentException
        final double[] wrong = new double[4];
        try {
            estimator.getStateAngularSpeed(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testHasError() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        assertFalse(estimator.hasError());

        estimator.mError = true;

        assertTrue(estimator.hasError());
    }

    @Test
    public void testIsAccumulationEnabled() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        assertTrue(estimator.isAccumulationEnabled());

        estimator.setAccumulationEnabled(false);

        assertFalse(estimator.isAccumulationEnabled());
    }

    @Test
    public void testGetAccelerometerTimestampNanos() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(-1, estimator.getAccelerometerTimestampNanos());

        // set new value
        estimator.mAccelerometerTimestampNanos = 1000;

        // check correctness
        assertEquals(1000, estimator.getAccelerometerTimestampNanos());
    }

    @Test
    public void testGetGyroscopeTimestampNanos() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(-1, estimator.getGyroscopeTimestampNanos());

        // set new value
        estimator.mGyroscopeTimestampNanos = 2000;

        // check correctness
        assertEquals(2000, estimator.getGyroscopeTimestampNanos());
    }

    @Test
    public void testGetOrientationTimestampNanos() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(-1, estimator.getOrientationTimestampNanos());

        // set new value
        estimator.mOrientationTimestampNanos = 3000;

        // check correctness
        assertEquals(3000, estimator.getOrientationTimestampNanos());
    }

    @Test
    public void testGetAccumulatedAccelerometerSamplesAndIsAccelerometerSampleReceived() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(0, estimator.getAccumulatedAccelerometerSamples());
        assertFalse(estimator.isAccelerometerSampleReceived());

        // set new value
        estimator.mAccumulatedAccelerometerSamples = 10;

        // check correctness
        assertEquals(10, estimator.getAccumulatedAccelerometerSamples());
        assertTrue(estimator.isAccelerometerSampleReceived());
    }

    @Test
    public void testGetAccumulatedGyroscopeSamplesAndIsGyroscopeSampleReceived() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(0, estimator.getAccumulatedGyroscopeSamples());
        assertFalse(estimator.isGyroscopeSampleReceived());

        // set new value
        estimator.mAccumulatedGyroscopeSamples = 20;

        // check correctness
        assertEquals(20, estimator.getAccumulatedGyroscopeSamples());
        assertTrue(estimator.isGyroscopeSampleReceived());
    }

    @Test
    public void testGetAccumulatedOrientationAndIsOrientationSampleReceived() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(0, estimator.getAccumulatedOrientationSamples());
        assertFalse(estimator.isOrientationSampleReceived());

        // set new value
        final Quaternion q = new Quaternion(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        estimator.mAccumulatedOrientation = q;

        // check correctness
        final Quaternion accumulatedOrientation = (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(q.getA(), accumulatedOrientation.getA(), ABSOLUTE_ERROR);
        assertEquals(q.getB(), accumulatedOrientation.getB(), ABSOLUTE_ERROR);
        assertEquals(q.getC(), accumulatedOrientation.getC(), ABSOLUTE_ERROR);
        assertEquals(q.getD(), accumulatedOrientation.getD(), ABSOLUTE_ERROR);
    }

    @Test
    public void testIsFullSampleAvailable() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(0, estimator.getAccumulatedAccelerometerSamples());
        assertEquals(0, estimator.getAccumulatedGyroscopeSamples());
        assertEquals(0, estimator.getAccumulatedOrientationSamples());
        assertFalse(estimator.isAccelerometerSampleReceived());
        assertFalse(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.mAccumulatedAccelerometerSamples = 1;

        assertEquals(1, estimator.getAccumulatedAccelerometerSamples());
        assertEquals(0, estimator.getAccumulatedGyroscopeSamples());
        assertEquals(0, estimator.getAccumulatedOrientationSamples());
        assertTrue(estimator.isAccelerometerSampleReceived());
        assertFalse(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.mAccumulatedGyroscopeSamples = 2;

        assertEquals(1, estimator.getAccumulatedAccelerometerSamples());
        assertEquals(2, estimator.getAccumulatedGyroscopeSamples());
        assertEquals(0, estimator.getAccumulatedOrientationSamples());
        assertTrue(estimator.isAccelerometerSampleReceived());
        assertTrue(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.mAccumulatedOrientationSamples = 3;

        assertEquals(1, estimator.getAccumulatedAccelerometerSamples());
        assertEquals(2, estimator.getAccumulatedGyroscopeSamples());
        assertEquals(3, estimator.getAccumulatedOrientationSamples());
        assertTrue(estimator.isAccelerometerSampleReceived());
        assertTrue(estimator.isGyroscopeSampleReceived());
        assertTrue(estimator.isFullSampleAvailable());
    }

    @Test
    public void testGetAccumulatedAccelerationSampleX() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleX(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleX = value;

        // check correctness
        assertEquals(value, estimator.getAccumulatedAccelerationSampleX(), 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSampleY() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleY(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleY = value;

        // check correctness
        assertEquals(value, estimator.getAccumulatedAccelerationSampleY(), 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSampleZ() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleZ(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleZ = value;

        // check correctness
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(), value, 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSample() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getAccumulatedAccelerationSample(), 0.0);

        // set new values
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleX = valueX;
        estimator.mAccumulatedAccelerationSampleY = valueY;
        estimator.mAccumulatedAccelerationSampleZ = valueZ;

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, estimator.getAccumulatedAccelerationSample(),
                0.0);

        final double[] acceleration = new double[3];
        estimator.getAccumulatedAccelerationSample(acceleration);

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, acceleration, 0.0);

        // Force IllegalArgumentException
        final double[] wrong = new double[4];
        try {
            estimator.getAccumulatedAccelerationSample(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetAccumulatedAngularSpeedSampleX() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleX(), 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAngularSpeedSampleX = value;

        // check correctness
        assertEquals(value, estimator.getAccumulatedAngularSpeedSampleX(), 0.0);
    }

    @Test
    public void testGetAccumulatedAngularSpeedSampleY() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleY(), 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAngularSpeedSampleY = value;

        // check correctness
        assertEquals(value, estimator.getAccumulatedAngularSpeedSampleY(), 0.0);
    }

    @Test
    public void testGetAccumulatedAngularSpeedSampleZ() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleZ(), 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAngularSpeedSampleZ = value;

        // check correctness
        assertEquals(value, estimator.getAccumulatedAngularSpeedSampleZ(), 0.0);
    }

    @Test
    public void testGetAccumulatedAngularSpeedSample() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getAccumulatedAngularSpeedSample(), 0.0);

        // set new value
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        estimator.mAccumulatedAngularSpeedSampleX = valueX;
        estimator.mAccumulatedAngularSpeedSampleY = valueY;
        estimator.mAccumulatedAngularSpeedSampleZ = valueZ;

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, estimator.getAccumulatedAngularSpeedSample(),
                0.0);

        final double[] speed = new double[3];
        estimator.getAccumulatedAngularSpeedSample(speed);

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, speed, 0.0);

        // Force IllegalArgumentException
        final double[] wrong = new double[4];
        try {
            estimator.getAccumulatedAngularSpeedSample(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testUpdateAccelerometerSampleWithAccumulationDisabled() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(false);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        long timestamp = System.currentTimeMillis();
        final float accelerationX = randomizer.nextFloat();
        final float accelerationY = randomizer.nextFloat();
        final float accelerationZ = randomizer.nextFloat();

        // check initial values
        assertEquals(-1, estimator.getAccelerometerTimestampNanos());
        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(0, estimator.getAccumulatedAccelerometerSamples());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);

        // check correctness
        assertEquals(estimator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(accelerationX, estimator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(accelerationY, estimator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(accelerationZ, estimator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(1, estimator.getAccumulatedAccelerometerSamples());
        assertFalse(estimator.isFullSampleAvailable());

        // test again but using an array
        final float[] acceleration = new float[3];
        randomizer.fill(acceleration);
        timestamp += 1000;

        estimator.updateAccelerometerSample(timestamp, acceleration);

        // check correctness
        assertEquals(timestamp, estimator.getAccelerometerTimestampNanos());
        assertEquals(acceleration[0], estimator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(acceleration[1], estimator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(acceleration[2], estimator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(2, estimator.getAccumulatedAccelerometerSamples());
        assertFalse(estimator.isFullSampleAvailable());

        // Force IllegalArgumentException
        final float[] wrong = new float[4];
        try {
            estimator.updateAccelerometerSample(timestamp, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testUpdateAccelerometerSampleWithAccumulationEnabled() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(true);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(-1, estimator.getAccelerometerTimestampNanos());
        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(0, estimator.getAccumulatedAccelerometerSamples());
        assertFalse(estimator.isFullSampleAvailable());

        // update with several samples
        long timestamp = System.currentTimeMillis();
        float accelerationX;
        float accelerationY;
        float accelerationZ;
        double avgAccelerationX = 0.0;
        double avgAccelerationY = 0.0;
        double avgAccelerationZ = 0.0;
        for (int i = 0; i < TIMES; i++) {
            timestamp += 1000;
            accelerationX = randomizer.nextFloat();
            accelerationY = randomizer.nextFloat();
            accelerationZ = randomizer.nextFloat();

            avgAccelerationX += accelerationX / TIMES;
            avgAccelerationY += accelerationY / TIMES;
            avgAccelerationZ += accelerationZ / TIMES;

            estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
        }

        // check correctness
        assertEquals(estimator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(avgAccelerationX, estimator.getAccumulatedAccelerationSampleX(), ABSOLUTE_ERROR);
        assertEquals(avgAccelerationY, estimator.getAccumulatedAccelerationSampleY(), ABSOLUTE_ERROR);
        assertEquals(avgAccelerationZ, estimator.getAccumulatedAccelerationSampleZ(), ABSOLUTE_ERROR);
        assertEquals(TIMES, estimator.getAccumulatedAccelerometerSamples());
        assertFalse(estimator.isFullSampleAvailable());
    }

    @Test
    public void testUpdateGyroscopeSampleWithAccumulationDisabled() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(false);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        long timestamp = System.currentTimeMillis();
        final float angularSpeedX = randomizer.nextFloat();
        final float angularSpeedY = randomizer.nextFloat();
        final float angularSpeedZ = randomizer.nextFloat();

        // check initial values
        assertEquals(-1, estimator.getGyroscopeTimestampNanos());
        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertEquals(0, estimator.getAccumulatedGyroscopeSamples());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

        // check correctness
        assertEquals(timestamp, estimator.getGyroscopeTimestampNanos());
        assertEquals(angularSpeedX, estimator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(angularSpeedY, estimator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(angularSpeedZ, estimator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertEquals(1, estimator.getAccumulatedGyroscopeSamples());
        assertFalse(estimator.isFullSampleAvailable());

        // test again but using an array
        final float[] angularSpeed = new float[3];
        randomizer.fill(angularSpeed);
        timestamp += 100;

        estimator.updateGyroscopeSample(timestamp, angularSpeed);

        // check correctness
        assertEquals(timestamp, estimator.getGyroscopeTimestampNanos());
        assertEquals(angularSpeed[0], estimator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(angularSpeed[1], estimator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(angularSpeed[2], estimator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertEquals(2, estimator.getAccumulatedGyroscopeSamples());
        assertFalse(estimator.isFullSampleAvailable());

        // Force IllegalArgumentException
        final float[] wrong = new float[4];
        try {
            estimator.updateGyroscopeSample(timestamp, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testUpdateGyroscopeSampleWithAccumulationEnabled() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(true);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(-1, estimator.getGyroscopeTimestampNanos());
        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertEquals(0, estimator.getAccumulatedGyroscopeSamples());
        assertFalse(estimator.isFullSampleAvailable());

        // update with several samples
        long timestamp = System.currentTimeMillis();
        float angularSpeedX;
        float angularSpeedY;
        float angularSpeedZ;
        double avgAngularSpeedX = 0.0;
        double avgAngularSpeedY = 0.0;
        double avgAngularSpeedZ = 0.0;
        for (int i = 0; i < TIMES; i++) {
            timestamp += 1000;
            angularSpeedX = randomizer.nextFloat();
            angularSpeedY = randomizer.nextFloat();
            angularSpeedZ = randomizer.nextFloat();

            avgAngularSpeedX += angularSpeedX / TIMES;
            avgAngularSpeedY += angularSpeedY / TIMES;
            avgAngularSpeedZ += angularSpeedZ / TIMES;

            estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
        }

        // check correctness
        assertEquals(timestamp, estimator.getGyroscopeTimestampNanos());
        assertEquals(avgAngularSpeedX, estimator.getAccumulatedAngularSpeedSampleX(), ABSOLUTE_ERROR);
        assertEquals(avgAngularSpeedY, estimator.getAccumulatedAngularSpeedSampleY(), ABSOLUTE_ERROR);
        assertEquals(avgAngularSpeedZ, estimator.getAccumulatedAngularSpeedSampleZ(), ABSOLUTE_ERROR);
        assertEquals(TIMES, estimator.getAccumulatedGyroscopeSamples());
        assertFalse(estimator.isFullSampleAvailable());
    }

    @Test
    public void testUpdateOrientationSampleWithAccumulationDisabled() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(false);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final long timestamp = System.currentTimeMillis();
        final double orientationA = randomizer.nextDouble();
        final double orientationB = randomizer.nextDouble();
        final double orientationC = randomizer.nextDouble();
        final double orientationD = randomizer.nextDouble();
        final Quaternion orientation = new Quaternion(orientationA, orientationB, orientationC, orientationD);

        // check initial values
        assertEquals(-1, estimator.getOrientationTimestampNanos());
        Quaternion accumulatedOrientation = (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(1.0, accumulatedOrientation.getA(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getB(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getC(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getD(), 0.0);
        assertEquals(0, estimator.getAccumulatedOrientationSamples());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.updateOrientationSample(timestamp, orientation);

        // check correctness
        assertEquals(timestamp, estimator.getOrientationTimestampNanos());
        accumulatedOrientation = (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(orientationA, accumulatedOrientation.getA(), 0.0);
        assertEquals(orientationB, accumulatedOrientation.getB(), 0.0);
        assertEquals(orientationC, accumulatedOrientation.getC(), 0.0);
        assertEquals(orientationD, accumulatedOrientation.getD(), 0.0);
    }

    @Test
    public void testUpdateOrientationSampleWithAccumulationEnabled() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(true);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(-1, estimator.getOrientationTimestampNanos());
        Quaternion accumulatedOrientation = (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(1.0, accumulatedOrientation.getA(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getB(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getC(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getD(), 0.0);
        assertEquals(0, estimator.getAccumulatedOrientationSamples());
        assertFalse(estimator.isFullSampleAvailable());

        // update with several samples
        long timestamp = System.currentTimeMillis();
        double orientationA;
        double orientationB;
        double orientationC;
        double orientationD;
        double avgOrientationA = 0.0, avgOrientationB = 0.0, avgOrientationC = 0.0, avgOrientationD = 0.0;
        Quaternion orientation = new Quaternion();
        double norm;
        for (int i = 0; i < TIMES; i++) {
            timestamp += 1000;
            orientationA = randomizer.nextDouble();
            orientationB = randomizer.nextDouble();
            orientationC = randomizer.nextDouble();
            orientationD = randomizer.nextDouble();
            norm = Math.sqrt(orientationA * orientationA + orientationB * orientationB +
                    orientationC * orientationC + orientationD * orientationD);
            orientationA /= norm;
            orientationB /= norm;
            orientationC /= norm;
            orientationD /= norm;

            avgOrientationA += orientationA / TIMES;
            avgOrientationB += orientationB / TIMES;
            avgOrientationC += orientationC / TIMES;
            avgOrientationD += orientationD / TIMES;

            orientation.setA(orientationA);
            orientation.setB(orientationB);
            orientation.setC(orientationC);
            orientation.setD(orientationD);

            estimator.updateOrientationSample(timestamp, orientation);
        }

        // check correctness
        assertEquals(estimator.getOrientationTimestampNanos(), timestamp);
        accumulatedOrientation = (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(avgOrientationA, accumulatedOrientation.getA(), ABSOLUTE_ERROR);
        assertEquals(avgOrientationB, accumulatedOrientation.getB(), ABSOLUTE_ERROR);
        assertEquals(avgOrientationC, accumulatedOrientation.getC(), ABSOLUTE_ERROR);
        assertEquals(avgOrientationD, accumulatedOrientation.getD(), ABSOLUTE_ERROR);
    }

    @Test
    public void testGetMostRecentTimestampNanos() {
        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        final long timestamp = System.currentTimeMillis();
        estimator.mAccelerometerTimestampNanos = timestamp;
        estimator.mGyroscopeTimestampNanos = timestamp + 1000;

        assertEquals(timestamp + 1000, estimator.getMostRecentTimestampNanos());

        estimator.mAccelerometerTimestampNanos = timestamp + 2000;

        assertEquals(timestamp + 2000, estimator.getMostRecentTimestampNanos());

        estimator.mOrientationTimestampNanos = timestamp + 3000;

        assertEquals(timestamp + 3000, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testCorrectWithPositionMeasureCoordinatesAndCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double positionX = randomizer.nextDouble();
            final double positionY = randomizer.nextDouble();
            final double positionZ = randomizer.nextDouble();

            final Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);

            // check initial value
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            final Quaternion orientation = new Quaternion();
            estimator.updateOrientationSample(timestamp, orientation);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(1, fullSampleReceived);
            assertEquals(1, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // correct has no effect until we predict again
            estimator.correctWithPositionMeasure(positionX, positionY, positionZ, positionCovariance);

            // check correctness
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, fullSampleReceived);
            assertEquals(1, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp,
                    0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);
            estimator.updateOrientationSample(timestamp, orientation);

            // check correctness
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(2, fullSampleReceived);
            assertEquals(2, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(positionX, positionY, positionZ, positionCovariance);

            // expected velocities
            final double vx = VELOCITY_GAIN * positionX;
            final double vy = VELOCITY_GAIN * positionY;
            final double vz = VELOCITY_GAIN * positionZ;

            // check correctness
            assertEquals(positionX, estimator.getStatePositionX(), ABSOLUTE_ERROR);
            assertEquals(positionY, estimator.getStatePositionY(), ABSOLUTE_ERROR);
            assertEquals(positionZ, estimator.getStatePositionZ(), ABSOLUTE_ERROR);
            assertEquals(vx, estimator.getStateVelocityX(), ABSOLUTE_ERROR);
            assertEquals(vy, estimator.getStateVelocityY(), ABSOLUTE_ERROR);
            assertEquals(vz, estimator.getStateVelocityZ(), ABSOLUTE_ERROR);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            final Quaternion stateQuaternion = estimator.getStateQuaternion();
            stateQuaternion.normalize();
            assertEquals(1.0, stateQuaternion.getA(), 0.0);
            assertEquals(0.0, stateQuaternion.getB(), 0.0);
            assertEquals(0.0, stateQuaternion.getC(), 0.0);
            assertEquals(0.0, stateQuaternion.getD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, correctWithPositionMeasure);
            assertEquals(1, correctedWithPositionMeasure);

            // Force IllegalArgumentException (wrong covariance matrix size)
            final Matrix wrong = new Matrix(4, 4);
            try {
                estimator.correctWithPositionMeasure(positionX, positionY, positionZ, wrong);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
        }
    }

    @Test
    public void testCorrectWithPositionMeasureArrayAndCovariance() throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double positionX = randomizer.nextDouble();
            final double positionY = randomizer.nextDouble();
            final double positionZ = randomizer.nextDouble();
            final double[] position = new double[]{positionX, positionY, positionZ};

            final Matrix positionCovariance = Matrix.identity(3, 3)
                    .multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);

            // check initial value
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            final Quaternion orientation = new Quaternion();
            estimator.updateOrientationSample(timestamp, orientation);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(1, fullSampleReceived);
            assertEquals(1, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // correct has no effect until we predict again
            estimator.correctWithPositionMeasure(position, positionCovariance);

            // check correctness
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, fullSampleReceived);
            assertEquals(1, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp,
                    0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);
            estimator.updateOrientationSample(timestamp, orientation);

            // check correctness
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(2, fullSampleReceived);
            assertEquals(2, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(position, positionCovariance);

            // expected velocities
            final double vx = VELOCITY_GAIN * positionX;
            final double vy = VELOCITY_GAIN * positionY;
            final double vz = VELOCITY_GAIN * positionZ;

            // check correctness
            assertEquals(positionX, estimator.getStatePositionX(), ABSOLUTE_ERROR);
            assertEquals(positionY, estimator.getStatePositionY(), ABSOLUTE_ERROR);
            assertEquals(positionZ, estimator.getStatePositionZ(), ABSOLUTE_ERROR);
            assertEquals(vx, estimator.getStateVelocityX(), ABSOLUTE_ERROR);
            assertEquals(vy, estimator.getStateVelocityY(), ABSOLUTE_ERROR);
            assertEquals(vz, estimator.getStateVelocityZ(), ABSOLUTE_ERROR);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            Quaternion stateQuaternion = estimator.getStateQuaternion();
            stateQuaternion.normalize();
            assertEquals(1.0, stateQuaternion.getA(), 0.0);
            assertEquals(0.0, stateQuaternion.getB(), 0.0);
            assertEquals(0.0, stateQuaternion.getC(), 0.0);
            assertEquals(0.0, stateQuaternion.getD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, correctWithPositionMeasure);
            assertEquals(1, correctedWithPositionMeasure);

            // Force IllegalArgumentException (wrong covariance matrix size)
            final Matrix wrongCov = new Matrix(4, 4);
            try {
                estimator.correctWithPositionMeasure(position, wrongCov);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }

            // wrong position length
            final double[] wrongPos = new double[4];
            try {
                estimator.correctWithPositionMeasure(wrongPos, positionCovariance);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
        }
    }

    @Test
    public void testCorrectWithPositionMeasurePointAndCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double positionX = randomizer.nextDouble();
            final double positionY = randomizer.nextDouble();
            final double positionZ = randomizer.nextDouble();
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(positionX, positionY, positionZ);

            final Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);

            // check initial value
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            final Quaternion orientation = new Quaternion();
            estimator.updateOrientationSample(timestamp, orientation);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(1, fullSampleReceived);
            assertEquals(1, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // correct has no effect until we predict again
            estimator.correctWithPositionMeasure(position, positionCovariance);

            // check correctness
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, fullSampleReceived);
            assertEquals(1, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp,
                    0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);
            estimator.updateOrientationSample(timestamp, orientation);

            // check correctness
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(2, fullSampleReceived);
            assertEquals(2, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(position, positionCovariance);

            // expected velocities
            final double vx = VELOCITY_GAIN * positionX;
            final double vy = VELOCITY_GAIN * positionY;
            final double vz = VELOCITY_GAIN * positionZ;

            // check correctness
            assertEquals(positionX, estimator.getStatePositionX(), ABSOLUTE_ERROR);
            assertEquals(positionY, estimator.getStatePositionY(), ABSOLUTE_ERROR);
            assertEquals(positionZ, estimator.getStatePositionZ(), ABSOLUTE_ERROR);
            assertEquals(vx, estimator.getStateVelocityX(), ABSOLUTE_ERROR);
            assertEquals(vy, estimator.getStateVelocityY(), ABSOLUTE_ERROR);
            assertEquals(vz, estimator.getStateVelocityZ(), ABSOLUTE_ERROR);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            final Quaternion stateQuaternion = estimator.getStateQuaternion();
            stateQuaternion.normalize();
            assertEquals(1.0, stateQuaternion.getA(), 0.0);
            assertEquals(0.0, stateQuaternion.getB(), 0.0);
            assertEquals(0.0, stateQuaternion.getC(), 0.0);
            assertEquals(0.0, stateQuaternion.getD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, correctWithPositionMeasure);
            assertEquals(1, correctedWithPositionMeasure);

            // Force IllegalArgumentException (wrong covariance matrix size)
            final Matrix wrong = new Matrix(4, 4);
            try {
                estimator.correctWithPositionMeasure(position, wrong);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
        }
    }

    @Test
    public void testGetSetPositionCovarianceMatrix() throws AlgebraException {

        final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

        // check initial value
        assertEquals(Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                KalmanFilter.DEFAULT_MEASUREMENT_NOISE_VARIANCE), estimator.getPositionCovarianceMatrix());

        // set new value
        final Matrix positionCovariance = Matrix.identity(3, 3).
                multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);

        estimator.setPositionCovarianceMatrix(positionCovariance);

        // check correctness
        assertEquals(positionCovariance, estimator.getPositionCovarianceMatrix());

        // Force IllegalArgumentException
        final Matrix wrong = new Matrix(4, 4);
        try {
            estimator.setPositionCovarianceMatrix(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testCorrectWithPositionMeasureCoordinatesWithoutCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double positionX = randomizer.nextDouble();
            final double positionY = randomizer.nextDouble();
            final double positionZ = randomizer.nextDouble();

            final Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);
            estimator.setPositionCovarianceMatrix(positionCovariance);

            // check initial value
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            final Quaternion orientation = new Quaternion();
            estimator.updateOrientationSample(timestamp, orientation);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(1, fullSampleReceived);
            assertEquals(1, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // correct has no effect until we predict again
            estimator.correctWithPositionMeasure(positionX, positionY, positionZ);

            // check correctness
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, fullSampleReceived);
            assertEquals(1, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp,
                    0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);
            estimator.updateOrientationSample(timestamp, orientation);

            // check correctness
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(2, fullSampleReceived);
            assertEquals(2, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(positionX, positionY, positionZ);

            // expected velocities
            final double vx = VELOCITY_GAIN * positionX;
            final double vy = VELOCITY_GAIN * positionY;
            final double vz = VELOCITY_GAIN * positionZ;

            // check correctness
            assertEquals(positionX, estimator.getStatePositionX(), ABSOLUTE_ERROR);
            assertEquals(positionY, estimator.getStatePositionY(), ABSOLUTE_ERROR);
            assertEquals(positionZ, estimator.getStatePositionZ(), ABSOLUTE_ERROR);
            assertEquals(vx, estimator.getStateVelocityX(), ABSOLUTE_ERROR);
            assertEquals(vy, estimator.getStateVelocityY(), ABSOLUTE_ERROR);
            assertEquals(vz, estimator.getStateVelocityZ(), ABSOLUTE_ERROR);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            Quaternion stateQuaternion = estimator.getStateQuaternion();
            stateQuaternion.normalize();
            assertEquals(1.0, stateQuaternion.getA(), 0.0);
            assertEquals(0.0, stateQuaternion.getB(), 0.0);
            assertEquals(0.0, stateQuaternion.getC(), 0.0);
            assertEquals(0.0, stateQuaternion.getD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, correctWithPositionMeasure);
            assertEquals(1, correctedWithPositionMeasure);
        }
    }

    @Test
    public void testCorrectWithPositionMeasureArrayWithoutCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double positionX = randomizer.nextDouble();
            final double positionY = randomizer.nextDouble();
            final double positionZ = randomizer.nextDouble();
            final double[] position = new double[]{positionX, positionY, positionZ};

            final Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);
            estimator.setPositionCovarianceMatrix(positionCovariance);

            // check initial value
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            final Quaternion orientation = new Quaternion();
            estimator.updateOrientationSample(timestamp, orientation);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(1, fullSampleReceived);
            assertEquals(1, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // correct has no effect until we predict again
            estimator.correctWithPositionMeasure(position);

            // check correctness
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, fullSampleReceived);
            assertEquals(1, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp,
                    0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);
            estimator.updateOrientationSample(timestamp, orientation);

            // check correctness
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(2, fullSampleReceived);
            assertEquals(2, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(position);

            // expected velocities
            final double vx = VELOCITY_GAIN * positionX;
            final double vy = VELOCITY_GAIN * positionY;
            final double vz = VELOCITY_GAIN * positionZ;

            // check correctness
            assertEquals(positionX, estimator.getStatePositionX(), ABSOLUTE_ERROR);
            assertEquals(positionY, estimator.getStatePositionY(), ABSOLUTE_ERROR);
            assertEquals(positionZ, estimator.getStatePositionZ(), ABSOLUTE_ERROR);
            assertEquals(vx, estimator.getStateVelocityX(), ABSOLUTE_ERROR);
            assertEquals(vy, estimator.getStateVelocityY(), ABSOLUTE_ERROR);
            assertEquals(vz, estimator.getStateVelocityZ(), ABSOLUTE_ERROR);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            final Quaternion stateQuaternion = estimator.getStateQuaternion();
            stateQuaternion.normalize();
            assertEquals(1.0, stateQuaternion.getA(), 0.0);
            assertEquals(0.0, stateQuaternion.getB(), 0.0);
            assertEquals(0.0, stateQuaternion.getC(), 0.0);
            assertEquals(0.0, stateQuaternion.getD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, correctWithPositionMeasure);
            assertEquals(1, correctedWithPositionMeasure);

            // Force IllegalArgumentException
            final double[] wrong = new double[4];
            try {
                estimator.correctWithPositionMeasure(wrong);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
        }
    }

    @Test
    public void testCorrectWithPositionMeasurePointWithoutCovariance() throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double positionX = randomizer.nextDouble();
            final double positionY = randomizer.nextDouble();
            final double positionZ = randomizer.nextDouble();
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(positionX, positionY, positionZ);

            final Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);
            estimator.setPositionCovarianceMatrix(positionCovariance);

            // check initial value
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            final Quaternion orientation = new Quaternion();
            estimator.updateOrientationSample(timestamp, orientation);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(1, fullSampleReceived);
            assertEquals(1, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // correct has no effect until we predict again
            estimator.correctWithPositionMeasure(position);

            // check correctness
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, fullSampleReceived);
            assertEquals(1, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp,
                    0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);
            estimator.updateOrientationSample(timestamp, orientation);

            // check correctness
            assertEquals(0.0, estimator.getStatePositionX(), 0.0);
            assertEquals(0.0, estimator.getStatePositionY(), 0.0);
            assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
            assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(2, fullSampleReceived);
            assertEquals(2, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            // correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(position);

            // expected velocities
            final double vx = VELOCITY_GAIN * positionX;
            final double vy = VELOCITY_GAIN * positionY;
            final double vz = VELOCITY_GAIN * positionZ;

            // check correctness
            assertEquals(positionX, estimator.getStatePositionX(), ABSOLUTE_ERROR);
            assertEquals(positionY, estimator.getStatePositionY(), ABSOLUTE_ERROR);
            assertEquals(positionZ, estimator.getStatePositionZ(), ABSOLUTE_ERROR);
            assertEquals(vx, estimator.getStateVelocityX(), ABSOLUTE_ERROR);
            assertEquals(vy, estimator.getStateVelocityY(), ABSOLUTE_ERROR);
            assertEquals(vz, estimator.getStateVelocityZ(), ABSOLUTE_ERROR);
            assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
            assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
            final Quaternion stateQuaternion = estimator.getStateQuaternion();
            stateQuaternion.normalize();
            assertEquals(1.0, stateQuaternion.getA(), 0.0);
            assertEquals(0.0, stateQuaternion.getB(), 0.0);
            assertEquals(0.0, stateQuaternion.getC(), 0.0);
            assertEquals(0.0, stateQuaternion.getD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, correctWithPositionMeasure);
            assertEquals(1, correctedWithPositionMeasure);

            // Force IllegalArgumentException (wrong covariance matrix size)
            final Matrix wrong = new Matrix(4, 4);
            try {
                estimator.correctWithPositionMeasure(
                        positionX, positionY, positionZ, wrong);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
        }
    }

    @Test
    public void testCreateCalibrator() {
        //noinspection ConstantValue
        assertTrue(AbsoluteOrientationSlamEstimator.createCalibrator()
                instanceof AbsoluteOrientationSlamCalibrator);
    }

    @Test
    public void testPredictionNoMotionWithNoise() {
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0;
            final double gtPositionY = 0.0;
            final double gtPositionZ = 0.0;
            final double gtSpeedX = 0.0;
            final double gtSpeedY = 0.0;
            final double gtSpeedZ = 0.0;
            final float gtAccelerationX = 0.0f;
            final float gtAccelerationY = 0.0f;
            final float gtAccelerationZ = 0.0f;
            final float gtAngularSpeedX = 0.0f;
            final float gtAngularSpeedY = 0.0f;
            final float gtAngularSpeedZ = 0.0f;
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(),
                    estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionConstantSpeedWithNoise() {
        int numSuccess = 0;
        for (int t = 0; t < 5 * REPEAT_TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0;
            final double gtPositionY = 0.0;
            final double gtPositionZ = 0.0;
            final double gtSpeedX = randomizer.nextDouble();
            final double gtSpeedY = randomizer.nextDouble();
            final double gtSpeedZ = randomizer.nextDouble();
            final float gtAccelerationX = 0.0f;
            final float gtAccelerationY = 0.0f;
            final float gtAccelerationZ = 0.0f;
            final float gtAngularSpeedX = 0.0f;
            final float gtAngularSpeedY = 0.0f;
            final float gtAngularSpeedZ = 0.0f;
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionConstantAccelerationWithNoise() {
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0;
            final double gtPositionY = 0.0;
            final double gtPositionZ = 0.0;
            final double gtSpeedX = 0.0;
            final double gtSpeedY = 0.0;
            final double gtSpeedZ = 0.0;
            final float gtAccelerationX = randomizer.nextFloat();
            final float gtAccelerationY = randomizer.nextFloat();
            final float gtAccelerationZ = randomizer.nextFloat();
            final float gtAngularSpeedX = 0.0f;
            final float gtAngularSpeedY = 0.0f;
            final float gtAngularSpeedZ = 0.0f;
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) <
                            predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionRotationOnlyWithNoise() {
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0;
            final double gtPositionY = 0.0;
            final double gtPositionZ = 0.0;
            final double gtSpeedX = 0.0;
            final double gtSpeedY = 0.0;
            final double gtSpeedZ = 0.0;
            final float gtAccelerationX = randomizer.nextFloat();
            final float gtAccelerationY = randomizer.nextFloat();
            final float gtAccelerationZ = randomizer.nextFloat();
            final float gtAngularSpeedX = 0.0f;
            final float gtAngularSpeedY = 0.0f;
            final float gtAngularSpeedZ = 0.0f;
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved = filteredPos.distanceTo(groundTruthPos) <
                    predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionVariableAccelerationWithNoise() {
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0;
            final double gtPositionY = 0.0;
            final double gtPositionZ = 0.0;
            final double gtSpeedX = randomizer.nextDouble();
            final double gtSpeedY = randomizer.nextDouble();
            final double gtSpeedZ = randomizer.nextDouble();
            final int period = N_PREDICTION_SAMPLES / 2;
            final int offsetAccelerationX = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final int offsetAccelerationY = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final int offsetAccelerationZ = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final float amplitudeAccelerationX = randomizer.nextFloat();
            final float amplitudeAccelerationY = randomizer.nextFloat();
            final float amplitudeAccelerationZ = randomizer.nextFloat();
            float gtAccelerationX = (float) (amplitudeAccelerationX * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAccelerationX)));
            float gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAccelerationY)));
            float gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAccelerationZ)));
            final float gtAngularSpeedX = 0.0f;
            final float gtAngularSpeedY = 0.0f;
            final float gtAngularSpeedZ = 0.0f;
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                gtAccelerationX = (float) (amplitudeAccelerationX * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAccelerationX)));
                gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAccelerationY)));
                gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAccelerationZ)));

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved = filteredPos.distanceTo(groundTruthPos) <
                    predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionVariableAngularSpeedWithNoise() {
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0;
            final double gtPositionY = 0.0;
            final double gtPositionZ = 0.0;
            final double gtSpeedX = 0.0;
            final double gtSpeedY = 0.0;
            final double gtSpeedZ = 0.0;
            final float gtAccelerationX = 0.0f;
            final float gtAccelerationY = 0.0f;
            final float gtAccelerationZ = 0.0f;
            final int period = N_PREDICTION_SAMPLES / 2;
            final int offsetAngularSpeedX = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final int offsetAngularSpeedY = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final int offsetAngularSpeedZ = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final float amplitudeAngularSpeedX = randomizer.nextFloat();
            final float amplitudeAngularSpeedY = randomizer.nextFloat();
            final float amplitudeAngularSpeedZ = randomizer.nextFloat();
            float gtAngularSpeedX = (float) (amplitudeAngularSpeedX * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAngularSpeedX)));
            float gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAngularSpeedY)));
            float gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAngularSpeedZ)));
            final float gtQuaternionA = 1.0f, gtQuaternionB = 0.0f,
                    gtQuaternionC = 0.0f, gtQuaternionD = 0.0f;

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                gtAngularSpeedX = (float) (amplitudeAngularSpeedX * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAngularSpeedX)));
                gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAngularSpeedY)));
                gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAngularSpeedZ)));

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved || positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionWithNoiseAndInitialState() {
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = randomizer.nextDouble();
            final double gtPositionY = randomizer.nextDouble();
            final double gtPositionZ = randomizer.nextDouble();
            final double gtSpeedX = randomizer.nextDouble();
            final double gtSpeedY = randomizer.nextDouble();
            final double gtSpeedZ = randomizer.nextDouble();
            final float gtAccelerationX = randomizer.nextFloat();
            final float gtAccelerationY = randomizer.nextFloat();
            final float gtAccelerationZ = randomizer.nextFloat();
            final float gtAngularSpeedX = randomizer.nextFloat();
            final float gtAngularSpeedY = randomizer.nextFloat();
            final float gtAngularSpeedZ = randomizer.nextFloat();
            final Quaternion gtQuaternion = new Quaternion(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());
            final float gtQuaternionA = (float) gtQuaternion.getA();
            final float gtQuaternionB = (float) gtQuaternion.getB();
            final float gtQuaternionC = (float) gtQuaternion.getC();
            final float gtQuaternionD = (float) gtQuaternion.getD();

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved || positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionVariableAccelerationWithNoiseAndInitialState() {
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = randomizer.nextDouble();
            final double gtPositionY = randomizer.nextDouble();
            final double gtPositionZ = randomizer.nextDouble();
            final double gtSpeedX = randomizer.nextDouble();
            final double gtSpeedY = randomizer.nextDouble();
            final double gtSpeedZ = randomizer.nextDouble();
            final int period = N_PREDICTION_SAMPLES / 2;
            final int offsetAccelerationX = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final int offsetAccelerationY = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final int offsetAccelerationZ = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final float amplitudeAccelerationX = randomizer.nextFloat();
            final float amplitudeAccelerationY = randomizer.nextFloat();
            final float amplitudeAccelerationZ = randomizer.nextFloat();
            float gtAccelerationX = (float) (amplitudeAccelerationX * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAccelerationX)));
            float gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAccelerationY)));
            float gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAccelerationZ)));
            final float gtAngularSpeedX = randomizer.nextFloat();
            final float gtAngularSpeedY = randomizer.nextFloat();
            final float gtAngularSpeedZ = randomizer.nextFloat();
            final Quaternion gtQuaternion = new Quaternion(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());
            final float gtQuaternionA = (float) gtQuaternion.getA();
            final float gtQuaternionB = (float) gtQuaternion.getB();
            final float gtQuaternionC = (float) gtQuaternion.getC();
            final float gtQuaternionD = (float) gtQuaternion.getD();

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                gtAccelerationX = (float) (amplitudeAccelerationX * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAccelerationX)));
                gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAccelerationY)));
                gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAccelerationZ)));

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved || positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionVariableAngularSpeedWithNoiseAndInitialState() {
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = randomizer.nextDouble();
            final double gtPositionY = randomizer.nextDouble();
            final double gtPositionZ = randomizer.nextDouble();
            final double gtSpeedX = randomizer.nextDouble();
            final double gtSpeedY = randomizer.nextDouble();
            final double gtSpeedZ = randomizer.nextDouble();
            final float gtAccelerationX = randomizer.nextFloat();
            final float gtAccelerationY = randomizer.nextFloat();
            final float gtAccelerationZ = randomizer.nextFloat();
            final int period = N_PREDICTION_SAMPLES / 2;
            final int offsetAngularSpeedX = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final int offsetAngularSpeedY = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final int offsetAngularSpeedZ = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final float amplitudeAngularSpeedX = randomizer.nextFloat();
            final float amplitudeAngularSpeedY = randomizer.nextFloat();
            final float amplitudeAngularSpeedZ = randomizer.nextFloat();
            float gtAngularSpeedX = (float) (amplitudeAngularSpeedX * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAngularSpeedX)));
            float gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAngularSpeedY)));
            float gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAngularSpeedZ)));
            final Quaternion gtQuaternion = new Quaternion(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());
            final float gtQuaternionA = (float) gtQuaternion.getA();
            final float gtQuaternionB = (float) gtQuaternion.getB();
            final float gtQuaternionC = (float) gtQuaternion.getC();
            final float gtQuaternionD = (float) gtQuaternion.getD();

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                gtAngularSpeedX = (float) (amplitudeAngularSpeedX * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAngularSpeedX)));
                gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAngularSpeedY)));
                gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAngularSpeedZ)));

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved || positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionRandomAccelerationAndAngularSpeedWithNoise() {
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = randomizer.nextDouble();
            final double gtPositionY = randomizer.nextDouble();
            final double gtPositionZ = randomizer.nextDouble();
            final double gtSpeedX = randomizer.nextDouble();
            final double gtSpeedY = randomizer.nextDouble();
            final double gtSpeedZ = randomizer.nextDouble();
            float gtAccelerationX = randomizer.nextFloat();
            float gtAccelerationY = randomizer.nextFloat();
            float gtAccelerationZ = randomizer.nextFloat();
            float gtAngularSpeedX = randomizer.nextFloat();
            float gtAngularSpeedY = randomizer.nextFloat();
            float gtAngularSpeedZ = randomizer.nextFloat();
            final Quaternion gtQuaternion = new Quaternion(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());
            final float gtQuaternionA = (float) gtQuaternion.getA();
            float gtQuaternionB = (float) gtQuaternion.getB();
            float gtQuaternionC = (float) gtQuaternion.getC();
            float gtQuaternionD = (float) gtQuaternion.getD();

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                gtAccelerationX = randomizer.nextFloat();
                gtAccelerationY = randomizer.nextFloat();
                gtAccelerationZ = randomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                gtAngularSpeedX = randomizer.nextFloat();
                gtAngularSpeedY = randomizer.nextFloat();
                gtAngularSpeedZ = randomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertFalse(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionNoMotionWithNoiseAndCalibration() {
        int numSuccess = 0;
        for (int t = 0; t < 5 * REPEAT_TIMES; t++) {
            final UniformRandomizer offsetRandomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final AbsoluteOrientationSlamCalibrator calibrator =
                    createFinishedCalibrator(accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                            angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final AbsoluteOrientationSlamCalibrationData calibration = calibrator.getCalibrationData();

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0;
            final double gtPositionY = 0.0;
            final double gtPositionZ = 0.0;
            final double gtSpeedX = 0.0;
            final double gtSpeedY = 0.0;
            final double gtSpeedZ = 0.0;
            final float gtAccelerationX = 0.0f;
            final float gtAccelerationY = 0.0f;
            final float gtAccelerationZ = 0.0f;
            final float gtAngularSpeedX = 0.0f;
            final float gtAngularSpeedY = 0.0f;
            final float gtAngularSpeedZ = 0.0f;
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final AbsoluteOrientationSlamEstimator estimatorWithCalibration =
                    new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            float accelerationWithOffsetX;
            float accelerationWithOffsetY;
            float accelerationWithOffsetZ;
            float angularSpeedWithOffsetX;
            float angularSpeedWithOffsetY;
            float angularSpeedWithOffsetZ;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastAccelerationWithOffsetX = 0.0;
            double lastAccelerationWithOffsetY = 0.0;
            double lastAccelerationWithOffsetZ = 0.0;
            double lastAngularSpeedWithOffsetX = 0.0;
            double lastAngularSpeedWithOffsetY = 0.0;
            double lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX;
            double deltaAccelerationWithOffsetY;
            double deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX;
            double deltaAngularSpeedWithOffsetY;
            double deltaAngularSpeedWithOffsetZ;

            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            final double[] uWithOffset = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            final double[] xWithOffset = Arrays.copyOf(x, x.length);

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);
            estimatorWithCalibration.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                accelerationWithOffsetX = accelerationX + accelerationOffsetX;
                accelerationWithOffsetY = accelerationY + accelerationOffsetY;
                accelerationWithOffsetZ = accelerationZ + accelerationOffsetZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                angularSpeedWithOffsetX = angularSpeedX + angularOffsetX;
                angularSpeedWithOffsetY = angularSpeedY + angularOffsetY;
                angularSpeedWithOffsetZ = angularSpeedZ + angularOffsetZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY, accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, angularSpeedWithOffsetZ);
                estimatorWithCalibration.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastAccelerationWithOffsetX = accelerationWithOffsetX;
                    lastAccelerationWithOffsetY = accelerationWithOffsetY;
                    lastAccelerationWithOffsetZ = accelerationWithOffsetZ;

                    lastAngularSpeedWithOffsetX = angularSpeedWithOffsetX;
                    lastAngularSpeedWithOffsetY = angularSpeedWithOffsetY;
                    lastAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaAccelerationWithOffsetX = accelerationWithOffsetX - lastAccelerationWithOffsetX;
                    deltaAccelerationWithOffsetY = accelerationWithOffsetY - lastAccelerationWithOffsetY;
                    deltaAccelerationWithOffsetZ = accelerationWithOffsetZ - lastAccelerationWithOffsetZ;

                    deltaAngularSpeedWithOffsetX = angularSpeedWithOffsetX - lastAngularSpeedWithOffsetX;
                    deltaAngularSpeedWithOffsetY = angularSpeedWithOffsetY - lastAngularSpeedWithOffsetY;
                    deltaAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ - lastAngularSpeedWithOffsetZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    // we also take into account control signals with offset
                    uWithOffset[0] = deltaOrientation.getA();
                    uWithOffset[1] = deltaOrientation.getB();
                    uWithOffset[2] = deltaOrientation.getC();
                    uWithOffset[3] = deltaOrientation.getD();
                    uWithOffset[4] = uWithOffset[5] = uWithOffset[6] = 0.0;
                    uWithOffset[7] = deltaAccelerationWithOffsetX;
                    uWithOffset[8] = deltaAccelerationWithOffsetY;
                    uWithOffset[9] = deltaAccelerationWithOffsetZ;
                    uWithOffset[10] = deltaAngularSpeedWithOffsetX;
                    uWithOffset[11] = deltaAngularSpeedWithOffsetY;
                    uWithOffset[12] = deltaAngularSpeedWithOffsetZ;
                    StatePredictor.predictWithRotationAdjustment(xWithOffset, uWithOffset, DELTA_SECONDS,
                            xWithOffset);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastAccelerationWithOffsetX = accelerationWithOffsetX;
                    lastAccelerationWithOffsetY = accelerationWithOffsetY;
                    lastAccelerationWithOffsetZ = accelerationWithOffsetZ;

                    lastAngularSpeedWithOffsetX = angularSpeedWithOffsetX;
                    lastAngularSpeedWithOffsetY = angularSpeedWithOffsetY;
                    lastAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered with calibrator - positionX: " + estimatorWithCalibration.getStatePositionX() +
                    ", positionY: " + estimatorWithCalibration.getStatePositionY() +
                    ", positionZ: " + estimatorWithCalibration.getStatePositionZ() +
                    ", velocityX: " + estimatorWithCalibration.getStateVelocityX() +
                    ", velocityY: " + estimatorWithCalibration.getStateVelocityY() +
                    ", velocityZ: " + estimatorWithCalibration.getStateVelocityZ() +
                    ", accelerationX: " + estimatorWithCalibration.getStateAccelerationX() +
                    ", accelerationY: " + estimatorWithCalibration.getStateAccelerationY() +
                    ", accelerationZ: " + estimatorWithCalibration.getStateAccelerationZ() +
                    ", quaternionA: " + estimatorWithCalibration.getStateQuaternionA() +
                    ", quaternionB: " + estimatorWithCalibration.getStateQuaternionB() +
                    ", quaternionC: " + estimatorWithCalibration.getStateQuaternionC() +
                    ", quaternionD: " + estimatorWithCalibration.getStateQuaternionD() +
                    ", angularSpeedX: " + estimatorWithCalibration.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimatorWithCalibration.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimatorWithCalibration.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion =
                    estimatorWithCalibration.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(),
                    estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionConstantSpeedWithNoiseAndCalibration() {
        int numSuccess = 0;
        for (int t = 0; t < 5 * REPEAT_TIMES; t++) {
            final UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());
            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final AbsoluteOrientationSlamCalibrator calibrator =
                    createFinishedCalibrator(accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                            angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final AbsoluteOrientationSlamCalibrationData calibration = calibrator.getCalibrationData();

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0;
            final double gtPositionY = 0.0;
            final double gtPositionZ = 0.0;
            final double gtSpeedX = randomizer.nextDouble();
            final double gtSpeedY = randomizer.nextDouble();
            final double gtSpeedZ = randomizer.nextDouble();
            final float gtAccelerationX = 0.0f;
            final float gtAccelerationY = 0.0f;
            final float gtAccelerationZ = 0.0f;
            final float gtAngularSpeedX = 0.0f;
            final float gtAngularSpeedY = 0.0f;
            final float gtAngularSpeedZ = 0.0f;
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final AbsoluteOrientationSlamEstimator estimatorWithCalibration =
                    new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            float accelerationWithOffsetX;
            float accelerationWithOffsetY;
            float accelerationWithOffsetZ;
            float angularSpeedWithOffsetX;
            float angularSpeedWithOffsetY;
            float angularSpeedWithOffsetZ;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastAccelerationWithOffsetX = 0.0;
            double lastAccelerationWithOffsetY = 0.0;
            double lastAccelerationWithOffsetZ = 0.0;
            double lastAngularSpeedWithOffsetX = 0.0;
            double lastAngularSpeedWithOffsetY = 0.0;
            double lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX;
            double deltaAccelerationWithOffsetY;
            double deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX, deltaAngularSpeedWithOffsetY, deltaAngularSpeedWithOffsetZ;

            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            final double[] uWithOffset = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            final double[] xWithOffset = Arrays.copyOf(x, x.length);

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);
            estimatorWithCalibration.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                accelerationWithOffsetX = accelerationX + accelerationOffsetX;
                accelerationWithOffsetY = accelerationY + accelerationOffsetY;
                accelerationWithOffsetZ = accelerationZ + accelerationOffsetZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                angularSpeedWithOffsetX = angularSpeedX + angularOffsetX;
                angularSpeedWithOffsetY = angularSpeedY + angularOffsetY;
                angularSpeedWithOffsetZ = angularSpeedZ + angularOffsetZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY, accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, angularSpeedWithOffsetZ);
                estimatorWithCalibration.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastAccelerationWithOffsetX = accelerationWithOffsetX;
                    lastAccelerationWithOffsetY = accelerationWithOffsetY;
                    lastAccelerationWithOffsetZ = accelerationWithOffsetZ;

                    lastAngularSpeedWithOffsetX = angularSpeedWithOffsetX;
                    lastAngularSpeedWithOffsetY = angularSpeedWithOffsetY;
                    lastAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaAccelerationWithOffsetX = accelerationWithOffsetX - lastAccelerationWithOffsetX;
                    deltaAccelerationWithOffsetY = accelerationWithOffsetY - lastAccelerationWithOffsetY;
                    deltaAccelerationWithOffsetZ = accelerationWithOffsetZ - lastAccelerationWithOffsetZ;

                    deltaAngularSpeedWithOffsetX = angularSpeedWithOffsetX - lastAngularSpeedWithOffsetX;
                    deltaAngularSpeedWithOffsetY = angularSpeedWithOffsetY - lastAngularSpeedWithOffsetY;
                    deltaAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ - lastAngularSpeedWithOffsetZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    // we also take into account control signals with offset
                    uWithOffset[0] = deltaOrientation.getA();
                    uWithOffset[1] = deltaOrientation.getB();
                    uWithOffset[2] = deltaOrientation.getC();
                    uWithOffset[3] = deltaOrientation.getD();
                    uWithOffset[4] = uWithOffset[5] = uWithOffset[6] = 0.0;
                    uWithOffset[7] = deltaAccelerationWithOffsetX;
                    uWithOffset[8] = deltaAccelerationWithOffsetY;
                    uWithOffset[9] = deltaAccelerationWithOffsetZ;
                    uWithOffset[10] = deltaAngularSpeedWithOffsetX;
                    uWithOffset[11] = deltaAngularSpeedWithOffsetY;
                    uWithOffset[12] = deltaAngularSpeedWithOffsetZ;
                    StatePredictor.predictWithRotationAdjustment(xWithOffset, uWithOffset, DELTA_SECONDS,
                            xWithOffset);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastAccelerationWithOffsetX = accelerationWithOffsetX;
                    lastAccelerationWithOffsetY = accelerationWithOffsetY;
                    lastAccelerationWithOffsetZ = accelerationWithOffsetZ;

                    lastAngularSpeedWithOffsetX = angularSpeedWithOffsetX;
                    lastAngularSpeedWithOffsetY = angularSpeedWithOffsetY;
                    lastAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered with calibrator - positionX: " + estimatorWithCalibration.getStatePositionX() +
                    ", positionY: " + estimatorWithCalibration.getStatePositionY() +
                    ", positionZ: " + estimatorWithCalibration.getStatePositionZ() +
                    ", velocityX: " + estimatorWithCalibration.getStateVelocityX() +
                    ", velocityY: " + estimatorWithCalibration.getStateVelocityY() +
                    ", velocityZ: " + estimatorWithCalibration.getStateVelocityZ() +
                    ", accelerationX: " + estimatorWithCalibration.getStateAccelerationX() +
                    ", accelerationY: " + estimatorWithCalibration.getStateAccelerationY() +
                    ", accelerationZ: " + estimatorWithCalibration.getStateAccelerationZ() +
                    ", quaternionA: " + estimatorWithCalibration.getStateQuaternionA() +
                    ", quaternionB: " + estimatorWithCalibration.getStateQuaternionB() +
                    ", quaternionC: " + estimatorWithCalibration.getStateQuaternionC() +
                    ", quaternionD: " + estimatorWithCalibration.getStateQuaternionD() +
                    ", angularSpeedX: " + estimatorWithCalibration.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimatorWithCalibration.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimatorWithCalibration.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion =
                    estimatorWithCalibration.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(),
                    estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionConstantAccelerationWithNoiseAndCalibration() {
        int numSuccess = 0;
        for (int t = 0; t < 2 * REPEAT_TIMES; t++) {
            final UniformRandomizer offsetRandomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final AbsoluteOrientationSlamCalibrator calibrator =
                    createFinishedCalibrator(accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                            angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final AbsoluteOrientationSlamCalibrationData calibration = calibrator.getCalibrationData();

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0;
            final double gtPositionY = 0.0;
            final double gtPositionZ = 0.0;
            final double gtSpeedX = 0.0;
            final double gtSpeedY = 0.0;
            final double gtSpeedZ = 0.0;
            final float gtAccelerationX = randomizer.nextFloat();
            final float gtAccelerationY = randomizer.nextFloat();
            final float gtAccelerationZ = randomizer.nextFloat();
            final float gtAngularSpeedX = 0.0f;
            final float gtAngularSpeedY = 0.0f;
            final float gtAngularSpeedZ = 0.0f;
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final AbsoluteOrientationSlamEstimator estimatorWithCalibration =
                    new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            float accelerationWithOffsetX;
            float accelerationWithOffsetY;
            float accelerationWithOffsetZ;
            float angularSpeedWithOffsetX;
            float angularSpeedWithOffsetY;
            float angularSpeedWithOffsetZ;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastAccelerationWithOffsetX = 0.0;
            double lastAccelerationWithOffsetY = 0.0;
            double lastAccelerationWithOffsetZ = 0.0;
            double lastAngularSpeedWithOffsetX = 0.0;
            double lastAngularSpeedWithOffsetY = 0.0;
            double lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX;
            double deltaAccelerationWithOffsetY;
            double deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX;
            double deltaAngularSpeedWithOffsetY;
            double deltaAngularSpeedWithOffsetZ;

            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            final double[] uWithOffset = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            final double[] xWithOffset = Arrays.copyOf(x, x.length);

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);
            estimatorWithCalibration.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                accelerationWithOffsetX = accelerationX + accelerationOffsetX;
                accelerationWithOffsetY = accelerationY + accelerationOffsetY;
                accelerationWithOffsetZ = accelerationZ + accelerationOffsetZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                angularSpeedWithOffsetX = angularSpeedX + angularOffsetX;
                angularSpeedWithOffsetY = angularSpeedY + angularOffsetY;
                angularSpeedWithOffsetZ = angularSpeedZ + angularOffsetZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY, accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, angularSpeedWithOffsetZ);
                estimatorWithCalibration.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastAccelerationWithOffsetX = accelerationWithOffsetX;
                    lastAccelerationWithOffsetY = accelerationWithOffsetY;
                    lastAccelerationWithOffsetZ = accelerationWithOffsetZ;

                    lastAngularSpeedWithOffsetX = angularSpeedWithOffsetX;
                    lastAngularSpeedWithOffsetY = angularSpeedWithOffsetY;
                    lastAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaAccelerationWithOffsetX = accelerationWithOffsetX - lastAccelerationWithOffsetX;
                    deltaAccelerationWithOffsetY = accelerationWithOffsetY - lastAccelerationWithOffsetY;
                    deltaAccelerationWithOffsetZ = accelerationWithOffsetZ - lastAccelerationWithOffsetZ;

                    deltaAngularSpeedWithOffsetX = angularSpeedWithOffsetX - lastAngularSpeedWithOffsetX;
                    deltaAngularSpeedWithOffsetY = angularSpeedWithOffsetY - lastAngularSpeedWithOffsetY;
                    deltaAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ - lastAngularSpeedWithOffsetZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    // we also take into account control signals with offset
                    uWithOffset[0] = deltaOrientation.getA();
                    uWithOffset[1] = deltaOrientation.getB();
                    uWithOffset[2] = deltaOrientation.getC();
                    uWithOffset[3] = deltaOrientation.getD();
                    uWithOffset[4] = uWithOffset[5] = uWithOffset[6] = 0.0;
                    uWithOffset[7] = deltaAccelerationWithOffsetX;
                    uWithOffset[8] = deltaAccelerationWithOffsetY;
                    uWithOffset[9] = deltaAccelerationWithOffsetZ;
                    uWithOffset[10] = deltaAngularSpeedWithOffsetX;
                    uWithOffset[11] = deltaAngularSpeedWithOffsetY;
                    uWithOffset[12] = deltaAngularSpeedWithOffsetZ;
                    StatePredictor.predictWithRotationAdjustment(xWithOffset, uWithOffset, DELTA_SECONDS,
                            xWithOffset);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastAccelerationWithOffsetX = accelerationWithOffsetX;
                    lastAccelerationWithOffsetY = accelerationWithOffsetY;
                    lastAccelerationWithOffsetZ = accelerationWithOffsetZ;

                    lastAngularSpeedWithOffsetX = angularSpeedWithOffsetX;
                    lastAngularSpeedWithOffsetY = angularSpeedWithOffsetY;
                    lastAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered with calibrator - positionX: " + estimatorWithCalibration.getStatePositionX() +
                    ", positionY: " + estimatorWithCalibration.getStatePositionY() +
                    ", positionZ: " + estimatorWithCalibration.getStatePositionZ() +
                    ", velocityX: " + estimatorWithCalibration.getStateVelocityX() +
                    ", velocityY: " + estimatorWithCalibration.getStateVelocityY() +
                    ", velocityZ: " + estimatorWithCalibration.getStateVelocityZ() +
                    ", accelerationX: " + estimatorWithCalibration.getStateAccelerationX() +
                    ", accelerationY: " + estimatorWithCalibration.getStateAccelerationY() +
                    ", accelerationZ: " + estimatorWithCalibration.getStateAccelerationZ() +
                    ", quaternionA: " + estimatorWithCalibration.getStateQuaternionA() +
                    ", quaternionB: " + estimatorWithCalibration.getStateQuaternionB() +
                    ", quaternionC: " + estimatorWithCalibration.getStateQuaternionC() +
                    ", quaternionD: " + estimatorWithCalibration.getStateQuaternionD() +
                    ", angularSpeedX: " + estimatorWithCalibration.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimatorWithCalibration.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimatorWithCalibration.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion =
                    estimatorWithCalibration.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
                break;
            }
        }

        assertTrue(numSuccess > 0);
    }

    @Test
    public void testPredictionRotationOnlyWithNoiseAndCalibration() {
        int numSuccess = 0;
        for (int t = 0; t < 5 * REPEAT_TIMES; t++) {
            final UniformRandomizer offsetRandomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final AbsoluteOrientationSlamCalibrator calibrator =
                    createFinishedCalibrator(accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                            angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final AbsoluteOrientationSlamCalibrationData calibration = calibrator.getCalibrationData();

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0;
            final double gtPositionY = 0.0;
            final double gtPositionZ = 0.0;
            final double gtSpeedX = 0.0;
            final double gtSpeedY = 0.0;
            final double gtSpeedZ = 0.0;
            final float gtAccelerationX = randomizer.nextFloat();
            final float gtAccelerationY = randomizer.nextFloat();
            final float gtAccelerationZ = randomizer.nextFloat();
            final float gtAngularSpeedX = 0.0f;
            final float gtAngularSpeedY = 0.0f;
            final float gtAngularSpeedZ = 0.0f;
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final AbsoluteOrientationSlamEstimator estimatorWithCalibration =
                    new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            float accelerationWithOffsetX;
            float accelerationWithOffsetY;
            float accelerationWithOffsetZ;
            float angularSpeedWithOffsetX;
            float angularSpeedWithOffsetY;
            float angularSpeedWithOffsetZ;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastAccelerationWithOffsetX = 0.0;
            double lastAccelerationWithOffsetY = 0.0;
            double lastAccelerationWithOffsetZ = 0.0;
            double lastAngularSpeedWithOffsetX = 0.0;
            double lastAngularSpeedWithOffsetY = 0.0;
            double lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX;
            double deltaAccelerationWithOffsetY;
            double deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX;
            double deltaAngularSpeedWithOffsetY;
            double deltaAngularSpeedWithOffsetZ;
            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            final double[] uWithOffset = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            final double[] xWithOffset = Arrays.copyOf(x, x.length);

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);
            estimatorWithCalibration.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                accelerationWithOffsetX = accelerationX + accelerationOffsetX;
                accelerationWithOffsetY = accelerationY + accelerationOffsetY;
                accelerationWithOffsetZ = accelerationZ + accelerationOffsetZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                angularSpeedWithOffsetX = angularSpeedX + angularOffsetX;
                angularSpeedWithOffsetY = angularSpeedY + angularOffsetY;
                angularSpeedWithOffsetZ = angularSpeedZ + angularOffsetZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY, accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, angularSpeedWithOffsetZ);
                estimatorWithCalibration.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastAccelerationWithOffsetX = accelerationWithOffsetX;
                    lastAccelerationWithOffsetY = accelerationWithOffsetY;
                    lastAccelerationWithOffsetZ = accelerationWithOffsetZ;

                    lastAngularSpeedWithOffsetX = angularSpeedWithOffsetX;
                    lastAngularSpeedWithOffsetY = angularSpeedWithOffsetY;
                    lastAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaAccelerationWithOffsetX = accelerationWithOffsetX - lastAccelerationWithOffsetX;
                    deltaAccelerationWithOffsetY = accelerationWithOffsetY - lastAccelerationWithOffsetY;
                    deltaAccelerationWithOffsetZ = accelerationWithOffsetZ - lastAccelerationWithOffsetZ;

                    deltaAngularSpeedWithOffsetX = angularSpeedWithOffsetX - lastAngularSpeedWithOffsetX;
                    deltaAngularSpeedWithOffsetY = angularSpeedWithOffsetY - lastAngularSpeedWithOffsetY;
                    deltaAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ - lastAngularSpeedWithOffsetZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contain noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    // we also take into account control signals with offset
                    uWithOffset[0] = deltaOrientation.getA();
                    uWithOffset[1] = deltaOrientation.getB();
                    uWithOffset[2] = deltaOrientation.getC();
                    uWithOffset[3] = deltaOrientation.getD();
                    uWithOffset[4] = uWithOffset[5] = uWithOffset[6] = 0.0;
                    uWithOffset[7] = deltaAccelerationWithOffsetX;
                    uWithOffset[8] = deltaAccelerationWithOffsetY;
                    uWithOffset[9] = deltaAccelerationWithOffsetZ;
                    uWithOffset[10] = deltaAngularSpeedWithOffsetX;
                    uWithOffset[11] = deltaAngularSpeedWithOffsetY;
                    uWithOffset[12] = deltaAngularSpeedWithOffsetZ;
                    StatePredictor.predictWithRotationAdjustment(xWithOffset, uWithOffset, DELTA_SECONDS,
                            xWithOffset);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastAccelerationWithOffsetX = accelerationWithOffsetX;
                    lastAccelerationWithOffsetY = accelerationWithOffsetY;
                    lastAccelerationWithOffsetZ = accelerationWithOffsetZ;

                    lastAngularSpeedWithOffsetX = angularSpeedWithOffsetX;
                    lastAngularSpeedWithOffsetY = angularSpeedWithOffsetY;
                    lastAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered with calibrator - positionX: " + estimatorWithCalibration.getStatePositionX() +
                    ", positionY: " + estimatorWithCalibration.getStatePositionY() +
                    ", positionZ: " + estimatorWithCalibration.getStatePositionZ() +
                    ", velocityX: " + estimatorWithCalibration.getStateVelocityX() +
                    ", velocityY: " + estimatorWithCalibration.getStateVelocityY() +
                    ", velocityZ: " + estimatorWithCalibration.getStateVelocityZ() +
                    ", accelerationX: " + estimatorWithCalibration.getStateAccelerationX() +
                    ", accelerationY: " + estimatorWithCalibration.getStateAccelerationY() +
                    ", accelerationZ: " + estimatorWithCalibration.getStateAccelerationZ() +
                    ", quaternionA: " + estimatorWithCalibration.getStateQuaternionA() +
                    ", quaternionB: " + estimatorWithCalibration.getStateQuaternionB() +
                    ", quaternionC: " + estimatorWithCalibration.getStateQuaternionC() +
                    ", quaternionD: " + estimatorWithCalibration.getStateQuaternionD() +
                    ", angularSpeedX: " + estimatorWithCalibration.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimatorWithCalibration.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimatorWithCalibration.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion =
                    estimatorWithCalibration.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionVariableAccelerationWithNoiseAndCalibration() {
        int numSuccess = 0;
        for (int t = 0; t < 10 * REPEAT_TIMES; t++) {
            final UniformRandomizer offsetRandomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final AbsoluteOrientationSlamCalibrator calibrator =
                    createFinishedCalibrator(accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                            angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final AbsoluteOrientationSlamCalibrationData calibration = calibrator.getCalibrationData();

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0;
            final double gtPositionY = 0.0;
            final double gtPositionZ = 0.0;
            final double gtSpeedX = randomizer.nextDouble();
            final double gtSpeedY = randomizer.nextDouble();
            final double gtSpeedZ = randomizer.nextDouble();
            final int period = N_PREDICTION_SAMPLES / 2;
            final int offsetAccelerationX = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final int offsetAccelerationY = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final int offsetAccelerationZ = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final float amplitudeAccelerationX = randomizer.nextFloat();
            final float amplitudeAccelerationY = randomizer.nextFloat();
            final float amplitudeAccelerationZ = randomizer.nextFloat();
            float gtAccelerationX = (float) (amplitudeAccelerationX * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAccelerationX)));
            float gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAccelerationY)));
            float gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAccelerationZ)));
            final float gtAngularSpeedX = 0.0f;
            final float gtAngularSpeedY = 0.0f;
            final float gtAngularSpeedZ = 0.0f;
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final AbsoluteOrientationSlamEstimator estimatorWithCalibration =
                    new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            float accelerationWithOffsetX;
            float accelerationWithOffsetY;
            float accelerationWithOffsetZ;
            float angularSpeedWithOffsetX;
            float angularSpeedWithOffsetY;
            float angularSpeedWithOffsetZ;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastAccelerationWithOffsetX = 0.0;
            double lastAccelerationWithOffsetY = 0.0;
            double lastAccelerationWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX;
            double deltaAccelerationWithOffsetY;
            double deltaAccelerationWithOffsetZ;

            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            final double[] uWithOffset = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            final double[] xWithOffset = Arrays.copyOf(x, x.length);

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);
            estimatorWithCalibration.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                gtAccelerationX = (float) (amplitudeAccelerationX * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAccelerationX)));
                gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAccelerationY)));
                gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAccelerationZ)));

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                accelerationWithOffsetX = accelerationX + accelerationOffsetX;
                accelerationWithOffsetY = accelerationY + accelerationOffsetY;
                accelerationWithOffsetZ = accelerationZ + accelerationOffsetZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                angularSpeedWithOffsetX = angularSpeedX + angularOffsetX;
                angularSpeedWithOffsetY = angularSpeedY + angularOffsetY;
                angularSpeedWithOffsetZ = angularSpeedZ + angularOffsetZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY, accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, angularSpeedWithOffsetZ);
                estimatorWithCalibration.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastAccelerationWithOffsetX = accelerationWithOffsetX;
                    lastAccelerationWithOffsetY = accelerationWithOffsetY;
                    lastAccelerationWithOffsetZ = accelerationWithOffsetZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaAccelerationWithOffsetX = accelerationWithOffsetX - lastAccelerationWithOffsetX;
                    deltaAccelerationWithOffsetY = accelerationWithOffsetY - lastAccelerationWithOffsetY;
                    deltaAccelerationWithOffsetZ = accelerationWithOffsetZ - lastAccelerationWithOffsetZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    // we also take into account control signals with offset
                    uWithOffset[0] = deltaOrientation.getA();
                    uWithOffset[1] = deltaOrientation.getB();
                    uWithOffset[2] = deltaOrientation.getC();
                    uWithOffset[3] = deltaOrientation.getD();
                    uWithOffset[4] = uWithOffset[5] = uWithOffset[6] = 0.0;
                    uWithOffset[7] = deltaAccelerationWithOffsetX;
                    uWithOffset[8] = deltaAccelerationWithOffsetY;
                    uWithOffset[9] = deltaAccelerationWithOffsetZ;
                    StatePredictor.predictWithRotationAdjustment(xWithOffset,
                            uWithOffset, DELTA_SECONDS, xWithOffset);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastAccelerationWithOffsetX = accelerationWithOffsetX;
                    lastAccelerationWithOffsetY = accelerationWithOffsetY;
                    lastAccelerationWithOffsetZ = accelerationWithOffsetZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered with calibrator - positionX: " + estimatorWithCalibration.getStatePositionX() +
                    ", positionY: " + estimatorWithCalibration.getStatePositionY() +
                    ", positionZ: " + estimatorWithCalibration.getStatePositionZ() +
                    ", velocityX: " + estimatorWithCalibration.getStateVelocityX() +
                    ", velocityY: " + estimatorWithCalibration.getStateVelocityY() +
                    ", velocityZ: " + estimatorWithCalibration.getStateVelocityZ() +
                    ", accelerationX: " + estimatorWithCalibration.getStateAccelerationX() +
                    ", accelerationY: " + estimatorWithCalibration.getStateAccelerationY() +
                    ", accelerationZ: " + estimatorWithCalibration.getStateAccelerationZ() +
                    ", quaternionA: " + estimatorWithCalibration.getStateQuaternionA() +
                    ", quaternionB: " + estimatorWithCalibration.getStateQuaternionB() +
                    ", quaternionC: " + estimatorWithCalibration.getStateQuaternionC() +
                    ", quaternionD: " + estimatorWithCalibration.getStateQuaternionD() +
                    ", angularSpeedX: " + estimatorWithCalibration.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimatorWithCalibration.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimatorWithCalibration.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion =
                    estimatorWithCalibration.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > 10 * REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionVariableAngularSpeedWithNoiseAndCalibration() {
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            final UniformRandomizer offsetRandomizer = new UniformRandomizer(new Random());
            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final AbsoluteOrientationSlamCalibrator calibrator =
                    createFinishedCalibrator(accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                            angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final AbsoluteOrientationSlamCalibrationData calibration = calibrator.getCalibrationData();

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0;
            final double gtPositionY = 0.0;
            final double gtPositionZ = 0.0;
            final double gtSpeedX = 0.0;
            final double gtSpeedY = 0.0;
            final double gtSpeedZ = 0.0;
            final float gtAccelerationX = 0.0f;
            final float gtAccelerationY = 0.0f;
            final float gtAccelerationZ = 0.0f;
            final int period = N_PREDICTION_SAMPLES / 2;
            final int offsetAngularSpeedX = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final int offsetAngularSpeedY = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final int offsetAngularSpeedZ = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final float amplitudeAngularSpeedX = randomizer.nextFloat();
            final float amplitudeAngularSpeedY = randomizer.nextFloat();
            final float amplitudeAngularSpeedZ = randomizer.nextFloat();
            float gtAngularSpeedX = (float) (amplitudeAngularSpeedX * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAngularSpeedX)));
            float gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAngularSpeedY)));
            float gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(
                    2.0 * Math.PI / (double) period *
                            (double) (-offsetAngularSpeedZ)));
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final AbsoluteOrientationSlamEstimator estimator = new AbsoluteOrientationSlamEstimator();

            final AbsoluteOrientationSlamEstimator estimatorWithCalibration =
                    new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            double orientationA;
            double orientationB;
            double orientationC;
            double orientationD;
            float accelerationWithOffsetX;
            float accelerationWithOffsetY;
            float accelerationWithOffsetZ;
            float angularSpeedWithOffsetX;
            float angularSpeedWithOffsetY;
            float angularSpeedWithOffsetZ;
            double noiseOrientationA;
            double noiseOrientationB;
            double noiseOrientationC;
            double noiseOrientationD;
            float noiseAccelerationX;
            float noiseAccelerationY;
            float noiseAccelerationZ;
            float noiseAngularSpeedX;
            float noiseAngularSpeedY;
            float noiseAngularSpeedZ;
            double lastAccelerationX = 0.0;
            double lastAccelerationY = 0.0;
            double lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0;
            double lastAngularSpeedY = 0.0;
            double lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final Quaternion lastOrientation = new Quaternion();
            final Quaternion deltaOrientation = new Quaternion();
            double lastAccelerationWithOffsetX = 0.0;
            double lastAccelerationWithOffsetY = 0.0;
            double lastAccelerationWithOffsetZ = 0.0;
            double lastAngularSpeedWithOffsetX = 0.0;
            double lastAngularSpeedWithOffsetY = 0.0;
            double lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX;
            double deltaAccelerationWithOffsetY;
            double deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX;
            double deltaAngularSpeedWithOffsetY;
            double deltaAngularSpeedWithOffsetZ;

            double lastGtAccelerationX = 0.0;
            double lastGtAccelerationY = 0.0;
            double lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0;
            double lastGtAngularSpeedY = 0.0;
            double lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC,
                    gtQuaternionD);
            final double[] x = new double[16];
            final double[] u = new double[13];
            final double[] uWithOffset = new double[13];
            x[0] = gtPositionX;
            x[1] = gtPositionY;
            x[2] = gtPositionZ;
            x[3] = gtQuaternionA;
            x[4] = gtQuaternionB;
            x[5] = gtQuaternionC;
            x[6] = gtQuaternionD;
            x[7] = gtSpeedX;
            x[8] = gtSpeedY;
            x[9] = gtSpeedZ;
            x[10] = gtAccelerationX;
            x[11] = gtAccelerationY;
            x[12] = gtAccelerationZ;
            x[13] = gtAngularSpeedX;
            x[14] = gtAngularSpeedY;
            x[15] = gtAngularSpeedZ;

            // ground truth state and control
            final double[] gtX = Arrays.copyOf(x, x.length);
            final double[] gtU = new double[13];

            final double[] xWithOffset = Arrays.copyOf(x, x.length);

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);
            estimatorWithCalibration.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                accelerationWithOffsetX = accelerationX + accelerationOffsetX;
                accelerationWithOffsetY = accelerationY + accelerationOffsetY;
                accelerationWithOffsetZ = accelerationZ + accelerationOffsetZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                gtAngularSpeedX = (float) (amplitudeAngularSpeedX * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAngularSpeedX)));
                gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAngularSpeedY)));
                gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(
                        2.0 * Math.PI / (double) period *
                                (double) (i - offsetAngularSpeedZ)));

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                angularSpeedWithOffsetX = angularSpeedX + angularOffsetX;
                angularSpeedWithOffsetY = angularSpeedY + angularOffsetY;
                angularSpeedWithOffsetZ = angularSpeedZ + angularOffsetZ;

                noiseOrientationA = orientationRandomizer.nextDouble();
                noiseOrientationB = orientationRandomizer.nextDouble();
                noiseOrientationC = orientationRandomizer.nextDouble();
                noiseOrientationD = orientationRandomizer.nextDouble();

                orientationA = gtQuaternionA + noiseOrientationA;
                orientationB = gtQuaternionB + noiseOrientationB;
                orientationC = gtQuaternionC + noiseOrientationC;
                orientationD = gtQuaternionD + noiseOrientationD;

                orientation.setA(orientationA);
                orientation.setB(orientationB);
                orientation.setC(orientationC);
                orientation.setD(orientationD);

                gtOrientation.setA(gtQuaternionA);
                gtOrientation.setB(gtQuaternionB);
                gtOrientation.setC(gtQuaternionC);
                gtOrientation.setD(gtQuaternionD);

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY, accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, angularSpeedWithOffsetZ);
                estimatorWithCalibration.updateOrientationSample(timestamp, orientation);

                timestamp += DELTA_NANOS;

                if (i == 0) {
                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastOrientation.setA(orientationA);
                    lastOrientation.setB(orientationB);
                    lastOrientation.setC(orientationC);
                    lastOrientation.setD(orientationD);

                    lastAccelerationWithOffsetX = accelerationWithOffsetX;
                    lastAccelerationWithOffsetY = accelerationWithOffsetY;
                    lastAccelerationWithOffsetZ = accelerationWithOffsetZ;

                    lastAngularSpeedWithOffsetX = angularSpeedWithOffsetX;
                    lastAngularSpeedWithOffsetY = angularSpeedWithOffsetY;
                    lastAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;

                    lastGtOrientation.setA(gtQuaternionA);
                    lastGtOrientation.setB(gtQuaternionB);
                    lastGtOrientation.setC(gtQuaternionC);
                    lastGtOrientation.setD(gtQuaternionD);

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    lastOrientation.inverse(deltaOrientation);
                    deltaOrientation.combine(orientation);
                    deltaOrientation.normalize();

                    deltaAccelerationWithOffsetX = accelerationWithOffsetX - lastAccelerationWithOffsetX;
                    deltaAccelerationWithOffsetY = accelerationWithOffsetY - lastAccelerationWithOffsetY;
                    deltaAccelerationWithOffsetZ = accelerationWithOffsetZ - lastAccelerationWithOffsetZ;

                    deltaAngularSpeedWithOffsetX = angularSpeedWithOffsetX - lastAngularSpeedWithOffsetX;
                    deltaAngularSpeedWithOffsetY = angularSpeedWithOffsetY - lastAngularSpeedWithOffsetY;
                    deltaAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ - lastAngularSpeedWithOffsetZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    lastGtOrientation.inverse(deltaGtOrientation);
                    deltaGtOrientation.combine(gtOrientation);
                    deltaGtOrientation.normalize();

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = deltaOrientation.getA();
                    u[1] = deltaOrientation.getB();
                    u[2] = deltaOrientation.getC();
                    u[3] = deltaOrientation.getD();
                    u[4] = u[5] = u[6] = 0.0;
                    u[7] = deltaAccelerationX;
                    u[8] = deltaAccelerationY;
                    u[9] = deltaAccelerationZ;
                    u[10] = deltaAngularSpeedX;
                    u[11] = deltaAngularSpeedY;
                    u[12] = deltaAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(x, u, DELTA_SECONDS, x);

                    // we also take into account control signals with offset
                    uWithOffset[0] = deltaOrientation.getA();
                    uWithOffset[1] = deltaOrientation.getB();
                    uWithOffset[2] = deltaOrientation.getC();
                    uWithOffset[3] = deltaOrientation.getD();
                    uWithOffset[4] = uWithOffset[5] = uWithOffset[6] = 0.0;
                    uWithOffset[7] = deltaAccelerationWithOffsetX;
                    uWithOffset[8] = deltaAccelerationWithOffsetY;
                    uWithOffset[9] = deltaAccelerationWithOffsetZ;
                    uWithOffset[10] = deltaAngularSpeedWithOffsetX;
                    uWithOffset[11] = deltaAngularSpeedWithOffsetY;
                    uWithOffset[12] = deltaAngularSpeedWithOffsetZ;
                    StatePredictor.predictWithRotationAdjustment(xWithOffset, uWithOffset, DELTA_SECONDS,
                            xWithOffset);

                    gtU[0] = deltaGtOrientation.getA();
                    gtU[1] = deltaGtOrientation.getB();
                    gtU[2] = deltaGtOrientation.getC();
                    gtU[3] = deltaGtOrientation.getD();
                    gtU[4] = gtU[5] = gtU[6] = 0.0;
                    gtU[7] = deltaGtAccelerationX;
                    gtU[8] = deltaGtAccelerationY;
                    gtU[9] = deltaGtAccelerationZ;
                    gtU[10] = deltaGtAngularSpeedX;
                    gtU[11] = deltaGtAngularSpeedY;
                    gtU[12] = deltaGtAngularSpeedZ;
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU, DELTA_SECONDS, gtX);

                    lastAccelerationX = accelerationX;
                    lastAccelerationY = accelerationY;
                    lastAccelerationZ = accelerationZ;

                    lastAngularSpeedX = angularSpeedX;
                    lastAngularSpeedY = angularSpeedY;
                    lastAngularSpeedZ = angularSpeedZ;

                    lastAccelerationWithOffsetX = accelerationWithOffsetX;
                    lastAccelerationWithOffsetY = accelerationWithOffsetY;
                    lastAccelerationWithOffsetZ = accelerationWithOffsetZ;

                    lastAngularSpeedWithOffsetX = angularSpeedWithOffsetX;
                    lastAngularSpeedWithOffsetY = angularSpeedWithOffsetY;
                    lastAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ;

                    lastGtAccelerationX = gtAccelerationX;
                    lastGtAccelerationY = gtAccelerationY;
                    lastGtAccelerationZ = gtAccelerationZ;

                    lastGtAngularSpeedX = gtAngularSpeedX;
                    lastGtAngularSpeedY = gtAngularSpeedY;
                    lastGtAngularSpeedZ = gtAngularSpeedZ;
                }
            }

            msg = "Filtered with calibrator - positionX: " + estimatorWithCalibration.getStatePositionX() +
                    ", positionY: " + estimatorWithCalibration.getStatePositionY() +
                    ", positionZ: " + estimatorWithCalibration.getStatePositionZ() +
                    ", velocityX: " + estimatorWithCalibration.getStateVelocityX() +
                    ", velocityY: " + estimatorWithCalibration.getStateVelocityY() +
                    ", velocityZ: " + estimatorWithCalibration.getStateVelocityZ() +
                    ", accelerationX: " + estimatorWithCalibration.getStateAccelerationX() +
                    ", accelerationY: " + estimatorWithCalibration.getStateAccelerationY() +
                    ", accelerationZ: " + estimatorWithCalibration.getStateAccelerationZ() +
                    ", quaternionA: " + estimatorWithCalibration.getStateQuaternionA() +
                    ", quaternionB: " + estimatorWithCalibration.getStateQuaternionB() +
                    ", quaternionC: " + estimatorWithCalibration.getStateQuaternionC() +
                    ", quaternionD: " + estimatorWithCalibration.getStateQuaternionD() +
                    ", angularSpeedX: " + estimatorWithCalibration.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimatorWithCalibration.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimatorWithCalibration.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Filtered - positionX: " + estimator.getStatePositionX() +
                    ", positionY: " + estimator.getStatePositionY() +
                    ", positionZ: " + estimator.getStatePositionZ() +
                    ", velocityX: " + estimator.getStateVelocityX() +
                    ", velocityY: " + estimator.getStateVelocityY() +
                    ", velocityZ: " + estimator.getStateVelocityZ() +
                    ", accelerationX: " + estimator.getStateAccelerationX() +
                    ", accelerationY: " + estimator.getStateAccelerationY() +
                    ", accelerationZ: " + estimator.getStateAccelerationZ() +
                    ", quaternionA: " + estimator.getStateQuaternionA() +
                    ", quaternionB: " + estimator.getStateQuaternionB() +
                    ", quaternionC: " + estimator.getStateQuaternionC() +
                    ", quaternionD: " + estimator.getStateQuaternionD() +
                    ", angularSpeedX: " + estimator.getStateAngularSpeedX() +
                    ", angularSpeedY: " + estimator.getStateAngularSpeedY() +
                    ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0] +
                    ", positionY: " + x[1] +
                    ", positionZ: " + x[2] +
                    ", velocityX: " + x[7] +
                    ", velocityY: " + x[8] +
                    ", velocityZ: " + x[9] +
                    ", accelerationX: " + x[10] +
                    ", accelerationY: " + x[11] +
                    ", accelerationZ: " + x[12] +
                    ", quaternionA: " + x[3] +
                    ", quaternionB: " + x[4] +
                    ", quaternionC: " + x[5] +
                    ", quaternionD: " + x[6] +
                    ", angularSpeedX: " + x[13] +
                    ", angularSpeedY: " + x[14] +
                    ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0] +
                    ", positionY: " + gtX[1] +
                    ", positionZ: " + gtX[2] +
                    ", velocityX: " + gtX[7] +
                    ", velocityY: " + gtX[8] +
                    ", velocityZ: " + gtX[9] +
                    ", accelerationX: " + gtX[10] +
                    ", accelerationY: " + gtX[11] +
                    ", accelerationZ: " + gtX[12] +
                    ", quaternionA: " + gtX[3] +
                    ", quaternionB: " + gtX[4] +
                    ", quaternionC: " + gtX[5] +
                    ", quaternionD: " + gtX[6] +
                    ", angularSpeedX: " + gtX[13] +
                    ", angularSpeedY: " + gtX[14] +
                    ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final Quaternion filteredQuaternion =
                    estimatorWithCalibration.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved || positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final AbsoluteOrientationSlamEstimator estimator1 = new AbsoluteOrientationSlamEstimator();

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final AbsoluteOrientationSlamCalibrationData data = new AbsoluteOrientationSlamCalibrationData();
        estimator1.setCalibrationData(data);
        final double statePositionX = randomizer.nextDouble();
        estimator1.mStatePositionX = statePositionX;
        final double statePositionY = randomizer.nextDouble();
        estimator1.mStatePositionY = statePositionY;
        final double statePositionZ = randomizer.nextDouble();
        estimator1.mStatePositionZ = statePositionZ;
        final double stateVelocityX = randomizer.nextDouble();
        estimator1.mStateVelocityX = stateVelocityX;
        final double stateVelocityY = randomizer.nextDouble();
        estimator1.mStateVelocityY = stateVelocityY;
        final double stateVelocityZ = randomizer.nextDouble();
        estimator1.mStateVelocityZ = stateVelocityZ;
        final double stateAccelerationX = randomizer.nextDouble();
        estimator1.mStateAccelerationX = stateAccelerationX;
        final double stateAccelerationY = randomizer.nextDouble();
        estimator1.mStateAccelerationY = stateAccelerationY;
        final double stateAccelerationZ = randomizer.nextDouble();
        estimator1.mStateAccelerationZ = stateAccelerationZ;
        final double stateQuaternionA = randomizer.nextDouble();
        estimator1.mStateQuaternionA = stateQuaternionA;
        final double stateQuaternionB = randomizer.nextDouble();
        estimator1.mStateQuaternionB = stateQuaternionB;
        final double stateQuaternionC = randomizer.nextDouble();
        estimator1.mStateQuaternionC = stateQuaternionC;
        final double stateQuaternionD = randomizer.nextDouble();
        estimator1.mStateQuaternionD = stateQuaternionD;
        final double stateAngularSpeedX = randomizer.nextDouble();
        estimator1.mStateAngularSpeedX = stateAngularSpeedX;
        final double stateAngularSpeedY = randomizer.nextDouble();
        estimator1.mStateAngularSpeedY = stateAngularSpeedY;
        final double stateAngularSpeedZ = randomizer.nextDouble();
        estimator1.mStateAngularSpeedZ = stateAngularSpeedZ;
        estimator1.mError = true;
        estimator1.setAccumulationEnabled(false);
        estimator1.mAccelerometerTimestampNanos = 1000;
        estimator1.mGyroscopeTimestampNanos = 2000;
        estimator1.mOrientationTimestampNanos = 3000;
        estimator1.mAccumulatedAccelerometerSamples = 10;
        estimator1.mAccumulatedGyroscopeSamples = 20;
        final Quaternion q = new Quaternion(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        estimator1.mAccumulatedOrientation = q;
        final double accumulatedAccelerationSampleX = randomizer.nextDouble();
        estimator1.mAccumulatedAccelerationSampleX = accumulatedAccelerationSampleX;
        final double accumulatedAccelerationSampleY = randomizer.nextDouble();
        estimator1.mAccumulatedAccelerationSampleY = accumulatedAccelerationSampleY;
        final double accumulatedAccelerationSampleZ = randomizer.nextDouble();
        estimator1.mAccumulatedAccelerationSampleZ = accumulatedAccelerationSampleZ;
        final double accumulatedAngularSpeedSampleX = randomizer.nextDouble();
        estimator1.mAccumulatedAngularSpeedSampleX = accumulatedAngularSpeedSampleX;
        final double accumulatedAngularSpeedSampleY = randomizer.nextDouble();
        estimator1.mAccumulatedAngularSpeedSampleY = accumulatedAngularSpeedSampleY;
        final double accumulatedAngularSpeedSampleZ = randomizer.nextDouble();
        estimator1.mAccumulatedAngularSpeedSampleZ = accumulatedAngularSpeedSampleZ;
        final Matrix positionCovariance = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);
        estimator1.setPositionCovarianceMatrix(positionCovariance);

        // check
        assertSame(data, estimator1.getCalibrationData());
        assertEquals(statePositionX, estimator1.getStatePositionX(), 0.0);
        assertEquals(statePositionY, estimator1.getStatePositionY(), 0.0);
        assertEquals(statePositionZ, estimator1.getStatePositionZ(), 0.0);
        assertEquals(stateVelocityX, estimator1.getStateVelocityX(), 0.0);
        assertEquals(stateVelocityY, estimator1.getStateVelocityY(), 0.0);
        assertEquals(stateVelocityZ, estimator1.getStateVelocityZ(), 0.0);
        assertEquals(stateAccelerationX, estimator1.getStateAccelerationX(), 0.0);
        assertEquals(stateAccelerationY, estimator1.getStateAccelerationY(), 0.0);
        assertEquals(stateAccelerationZ, estimator1.getStateAccelerationZ(), 0.0);
        assertEquals(stateQuaternionA, estimator1.getStateQuaternionA(), 0.0);
        assertEquals(stateQuaternionB, estimator1.getStateQuaternionB(), 0.0);
        assertEquals(stateQuaternionC, estimator1.getStateQuaternionC(), 0.0);
        assertEquals(stateQuaternionD, estimator1.getStateQuaternionD(), 0.0);
        assertEquals(stateAngularSpeedX, estimator1.getStateAngularSpeedX(), 0.0);
        assertEquals(stateAngularSpeedY, estimator1.getStateAngularSpeedY(), 0.0);
        assertEquals(stateAngularSpeedZ, estimator1.getStateAngularSpeedZ(), 0.0);
        assertTrue(estimator1.hasError());
        assertFalse(estimator1.isAccumulationEnabled());
        assertEquals(1000, estimator1.getAccelerometerTimestampNanos());
        assertEquals(2000, estimator1.getGyroscopeTimestampNanos());
        assertEquals(3000, estimator1.getOrientationTimestampNanos());
        assertEquals(10, estimator1.getAccumulatedAccelerometerSamples());
        assertEquals(20, estimator1.getAccumulatedGyroscopeSamples());
        final Quaternion accumulatedOrientation1 = (Quaternion) estimator1.getAccumulatedOrientation();
        assertEquals(q.getA(), accumulatedOrientation1.getA(), ABSOLUTE_ERROR);
        assertEquals(q.getB(), accumulatedOrientation1.getB(), ABSOLUTE_ERROR);
        assertEquals(q.getC(), accumulatedOrientation1.getC(), ABSOLUTE_ERROR);
        assertEquals( q.getD(), accumulatedOrientation1.getD(), ABSOLUTE_ERROR);
        assertEquals(accumulatedAccelerationSampleX, estimator1.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(accumulatedAccelerationSampleY, estimator1.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(accumulatedAccelerationSampleZ, estimator1.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(accumulatedAngularSpeedSampleX, estimator1.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(accumulatedAngularSpeedSampleY, estimator1.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(accumulatedAngularSpeedSampleZ, estimator1.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertEquals(positionCovariance, estimator1.getPositionCovarianceMatrix());

        final byte[] bytes = SerializationHelper.serialize(estimator1);
        final AbsoluteOrientationSlamEstimator estimator2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(estimator1.getCalibrationData().getControlLength(),
                estimator2.getCalibrationData().getControlLength());
        assertEquals(estimator1.getCalibrationData().getStateLength(),
                estimator2.getCalibrationData().getStateLength());
        assertEquals(estimator1.getStatePositionX(), estimator2.getStatePositionX(), 0.0);
        assertEquals(estimator1.getStatePositionY(), estimator2.getStatePositionY(), 0.0);
        assertEquals(estimator1.getStatePositionZ(), estimator2.getStatePositionZ(), 0.0);
        assertEquals(estimator1.getStateVelocityX(), estimator2.getStateVelocityX(), 0.0);
        assertEquals(estimator1.getStateVelocityY(), estimator2.getStateVelocityY(), 0.0);
        assertEquals(estimator1.getStateVelocityZ(), estimator2.getStateVelocityZ(), 0.0);
        assertEquals(estimator1.getStateAccelerationX(), estimator2.getStateAccelerationX(), 0.0);
        assertEquals(estimator1.getStateAccelerationY(), estimator2.getStateAccelerationY(), 0.0);
        assertEquals(estimator1.getStateAccelerationZ(), estimator2.getStateAccelerationZ(), 0.0);
        assertEquals(estimator1.getStateQuaternionA(), estimator2.getStateQuaternionA(), 0.0);
        assertEquals(estimator1.getStateQuaternionB(), estimator2.getStateQuaternionB(), 0.0);
        assertEquals(estimator1.getStateQuaternionC(), estimator2.getStateQuaternionC(), 0.0);
        assertEquals(estimator1.getStateQuaternionD(), estimator2.getStateQuaternionD(), 0.0);
        assertEquals(estimator1.getStateAngularSpeedX(), estimator2.getStateAngularSpeedX(), 0.0);
        assertEquals(estimator1.getStateAngularSpeedY(), estimator2.getStateAngularSpeedY(), 0.0);
        assertEquals(estimator1.getStateAngularSpeedZ(), estimator2.getStateAngularSpeedZ(), 0.0);
        assertEquals(estimator1.hasError(), estimator2.hasError());
        assertEquals(estimator1.isAccumulationEnabled(), estimator2.isAccumulationEnabled());
        assertEquals(estimator1.getAccelerometerTimestampNanos(), estimator2.getAccelerometerTimestampNanos());
        assertEquals(estimator1.getGyroscopeTimestampNanos(), estimator2.getGyroscopeTimestampNanos());
        assertEquals(estimator1.getOrientationTimestampNanos(), estimator2.getOrientationTimestampNanos());
        assertEquals(estimator1.getAccumulatedAccelerometerSamples(),
                estimator2.getAccumulatedAccelerometerSamples());
        assertEquals(estimator1.getAccumulatedGyroscopeSamples(), estimator2.getAccumulatedGyroscopeSamples());
        final Quaternion accumulatedOrientation2 = (Quaternion) estimator2.getAccumulatedOrientation();
        assertEquals(accumulatedOrientation1, accumulatedOrientation2);
        assertEquals(estimator1.getAccumulatedAccelerationSampleX(),
                estimator2.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(estimator1.getAccumulatedAccelerationSampleY(),
                estimator2.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(estimator1.getAccumulatedAccelerationSampleZ(),
                estimator2.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(estimator1.getAccumulatedAngularSpeedSampleX(),
                estimator2.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(estimator1.getAccumulatedAngularSpeedSampleY(),
                estimator2.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(estimator1.getAccumulatedAngularSpeedSampleZ(),
                estimator2.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertEquals(estimator1.getPositionCovarianceMatrix(), estimator2.getPositionCovarianceMatrix());
    }

    @Override
    public void onFullSampleReceived(
            final BaseSlamEstimator<AbsoluteOrientationSlamCalibrationData> estimator) {
        fullSampleReceived++;
    }

    @Override
    public void onFullSampleProcessed(
            final BaseSlamEstimator<AbsoluteOrientationSlamCalibrationData> estimator) {
        fullSampleProcessed++;
    }

    @Override
    public void onCorrectWithPositionMeasure(
            final BaseSlamEstimator<AbsoluteOrientationSlamCalibrationData> estimator) {
        correctWithPositionMeasure++;
    }

    @Override
    public void onCorrectedWithPositionMeasure(
            final BaseSlamEstimator<AbsoluteOrientationSlamCalibrationData> estimator) {
        correctedWithPositionMeasure++;
    }

    private void reset() {
        fullSampleReceived = fullSampleProcessed = correctWithPositionMeasure =
                correctedWithPositionMeasure = 0;
    }

    private AbsoluteOrientationSlamCalibrator createFinishedCalibrator(
            final float accelerationOffsetX, final float accelerationOffsetY,
            final float accelerationOffsetZ, final float angularOffsetX,
            final float angularOffsetY, final float angularOffsetZ,
            final GaussianRandomizer noiseRandomizer) {
        final AbsoluteOrientationSlamCalibrator calibrator =
                AbsoluteOrientationSlamEstimator.createCalibrator();
        calibrator.setConvergenceThreshold(ABSOLUTE_ERROR);
        calibrator.setMaxNumSamples(MAX_CALIBRATION_SAMPLES);

        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

        float accelerationNoiseX;
        float accelerationNoiseY;
        float accelerationNoiseZ;
        float angularNoiseX;
        float angularNoiseY;
        float angularNoiseZ;

        double accelerationX;
        double accelerationY;
        double accelerationZ;
        double angularX;
        double angularY;
        double angularZ;
        final Quaternion orientation = new Quaternion();

        for (int i = 0; i < MAX_CALIBRATION_SAMPLES; i++) {
            accelerationNoiseX = noiseRandomizer.nextFloat();
            accelerationNoiseY = noiseRandomizer.nextFloat();
            accelerationNoiseZ = noiseRandomizer.nextFloat();

            angularNoiseX = noiseRandomizer.nextFloat();
            angularNoiseY = noiseRandomizer.nextFloat();
            angularNoiseZ = noiseRandomizer.nextFloat();

            accelerationX = accelerationOffsetX + accelerationNoiseX;
            accelerationY = accelerationOffsetY + accelerationNoiseY;
            accelerationZ = accelerationOffsetZ + accelerationNoiseZ;

            angularX = angularOffsetX + angularNoiseX;
            angularY = angularOffsetY + angularNoiseY;
            angularZ = angularOffsetZ + angularNoiseZ;

            calibrator.updateAccelerometerSample(timestamp, (float) accelerationX,
                    (float) accelerationY, (float) accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY,
                    (float) angularZ);
            calibrator.updateOrientationSample(timestamp, orientation);

            if (calibrator.isFinished()) {
                break;
            }

            timestamp += DELTA_NANOS;
        }

        return calibrator;
    }
}
