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

public class SlamEstimatorTest implements BaseSlamEstimatorListener<SlamCalibrationData> {

    private static final int TIMES = 50;
    private static final double ABSOLUTE_ERROR = 1e-8;

    // conversion from milliseconds to nanoseconds
    private static final int MILLIS_TO_NANOS = 1000000;

    // time between samples expressed in nanoseconds (a typical sensor in Android
    // delivers a sample every 20ms)
    private static final int DELTA_NANOS = 20000000; // 0.02 seconds
    private static final double DELTA_SECONDS = 0.02; // 0.02 seconds;

    // gain below is only valid when measure variance is 1e-8 and process noise
    // variance is 1e-6. This is the gain component of the Kalman filter for
    // those error covariances. This gain will change for other values
    private static final double VELOCITY_GAIN = 0.019991983014891204;

    private static final int REPEAT_TIMES = 10;
    private static final int N_PREDICTION_SAMPLES = 10000;

    private static final double ACCELERATION_NOISE_STANDARD_DEVIATION = 1e-4;
    private static final double ANGULAR_SPEED_NOISE_STANDARD_DEVIATION = 1e-4;

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
        final SlamEstimator estimator = new SlamEstimator();

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

        assertEquals(Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                KalmanFilter.DEFAULT_MEASUREMENT_NOISE_VARIANCE), estimator.getPositionCovarianceMatrix());
        assertNotNull(estimator.getStateCovariance());
    }

    @Test
    public void testGetSetListener() {
        final SlamEstimator estimator = new SlamEstimator();

        // initial value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testGetSetCalibrationData() {
        final SlamEstimator estimator = new SlamEstimator();

        // initial value
        assertNull(estimator.getCalibrationData());

        // set new value
        final SlamCalibrationData data = new SlamCalibrationData();
        estimator.setCalibrationData(data);

        // check correctness
        assertSame(data, estimator.getCalibrationData());
    }

    @Test
    public void testResetPosition() {
        final SlamEstimator estimator = new SlamEstimator();

        estimator.resetPosition();

        assertEquals(0.0, estimator.getStatePositionX(), 0.0);
        assertEquals(0.0, estimator.getStatePositionY(), 0.0);
        assertEquals(0.0, estimator.getStatePositionZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testResetVelocity() {
        final SlamEstimator estimator = new SlamEstimator();

        estimator.resetVelocity();

        assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testResetPositionAndVelocity() {
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

        estimator.resetAcceleration();

        assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
        assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
        assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testResetOrientation() {
        final SlamEstimator estimator = new SlamEstimator();

        estimator.resetOrientation();

        assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testResetAngularSpeed() {
        final SlamEstimator estimator = new SlamEstimator();

        estimator.resetAngularSpeed();

        assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
        assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
        assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testReset() {
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetStateVelocityX() {
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        assertArrayEquals(new double[]{valueA, valueB, valueC, valueD}, estimator.getStateQuaternionArray(),
                0.0);

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

        assertFalse(estimator.hasError());

        estimator.mError = true;

        assertTrue(estimator.hasError());
    }

    @Test
    public void testIsAccumulationEnabled() {
        final SlamEstimator estimator = new SlamEstimator();

        assertTrue(estimator.isAccumulationEnabled());

        estimator.setAccumulationEnabled(false);

        assertFalse(estimator.isAccumulationEnabled());
    }

    @Test
    public void testGetAccelerometerTimestampNanos() {
        final SlamEstimator estimator = new SlamEstimator();

        assertEquals(-1, estimator.getAccelerometerTimestampNanos());

        // set new value
        estimator.mAccelerometerTimestampNanos = 1000;

        // check correctness
        assertEquals(1000, estimator.getAccelerometerTimestampNanos());
    }

    @Test
    public void testGetGyroscopeTimestampNanos() {
        final SlamEstimator estimator = new SlamEstimator();

        assertEquals(-1, estimator.getGyroscopeTimestampNanos());

        // set new value
        estimator.mGyroscopeTimestampNanos = 2000;

        // check correctness
        assertEquals(2000, estimator.getGyroscopeTimestampNanos());
    }

    @Test
    public void testGetAccumulatedAccelerometerSamplesAndIsAccelerometerSampleReceived() {
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

        assertEquals(0, estimator.getAccumulatedGyroscopeSamples());
        assertFalse(estimator.isGyroscopeSampleReceived());

        // set new value
        estimator.mAccumulatedGyroscopeSamples = 20;

        // check correctness
        assertEquals(20, estimator.getAccumulatedGyroscopeSamples());
        assertTrue(estimator.isGyroscopeSampleReceived());
    }

    @Test
    public void testIsFullSampleAvailable() {
        final SlamEstimator estimator = new SlamEstimator();

        assertEquals(0, estimator.getAccumulatedAccelerometerSamples());
        assertEquals(0, estimator.getAccumulatedGyroscopeSamples());
        assertFalse(estimator.isAccelerometerSampleReceived());
        assertFalse(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.mAccumulatedAccelerometerSamples = 1;

        assertEquals(1, estimator.getAccumulatedAccelerometerSamples());
        assertEquals(0, estimator.getAccumulatedGyroscopeSamples());
        assertTrue(estimator.isAccelerometerSampleReceived());
        assertFalse(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.mAccumulatedGyroscopeSamples = 2;

        assertEquals(1, estimator.getAccumulatedAccelerometerSamples());
        assertEquals(2, estimator.getAccumulatedGyroscopeSamples());
        assertTrue(estimator.isAccelerometerSampleReceived());
        assertTrue(estimator.isGyroscopeSampleReceived());
        assertTrue(estimator.isFullSampleAvailable());
    }

    @Test
    public void testGetAccumulatedAccelerationSampleX() {
        final SlamEstimator estimator = new SlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleX(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleX = value;

        // check correctness
        assertEquals(value, estimator.getAccumulatedAccelerationSampleX(), 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSampleY() {
        final SlamEstimator estimator = new SlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleY(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleY = value;

        // check correctness
        assertEquals(value, estimator.getAccumulatedAccelerationSampleY(), 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSampleZ() {
        final SlamEstimator estimator = new SlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleZ(), 0.0);

        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleZ = value;

        // check correctness
        assertEquals(value, estimator.getAccumulatedAccelerationSampleZ(), 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSample() {
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();

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
        final SlamEstimator estimator = new SlamEstimator();
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
        assertEquals(timestamp, estimator.getAccelerometerTimestampNanos());
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
        final SlamEstimator estimator = new SlamEstimator();
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
        assertEquals(timestamp, estimator.getAccelerometerTimestampNanos());
        assertEquals(avgAccelerationX, estimator.getAccumulatedAccelerationSampleX(), ABSOLUTE_ERROR);
        assertEquals(avgAccelerationY, estimator.getAccumulatedAccelerationSampleY(), ABSOLUTE_ERROR);
        assertEquals(avgAccelerationZ, estimator.getAccumulatedAccelerationSampleZ(), ABSOLUTE_ERROR);
        assertEquals(TIMES, estimator.getAccumulatedAccelerometerSamples());
        assertFalse(estimator.isFullSampleAvailable());
    }

    @Test
    public void testUpdateGyroscopeSampleWithAccumulationDisabled() {
        final SlamEstimator estimator = new SlamEstimator();
        estimator.setAccumulationEnabled(false);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        long timestamp = System.currentTimeMillis();
        float angularSpeedX = randomizer.nextFloat();
        float angularSpeedY = randomizer.nextFloat();
        float angularSpeedZ = randomizer.nextFloat();

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
        final SlamEstimator estimator = new SlamEstimator();
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
    public void testGetMostRecentTimestampNanos() {
        final SlamEstimator estimator = new SlamEstimator();

        final long timestamp = System.currentTimeMillis();
        estimator.mAccelerometerTimestampNanos = timestamp;
        estimator.mGyroscopeTimestampNanos = timestamp + 1000;

        assertEquals(timestamp + 1000, estimator.getMostRecentTimestampNanos());

        estimator.mAccelerometerTimestampNanos = timestamp + 2000;

        assertEquals(timestamp + 2000, estimator.getMostRecentTimestampNanos());
    }

    @Test
    public void testCorrectWithPositionMeasureCoordinatesAndCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final SlamEstimator estimator = new SlamEstimator();
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
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
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
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
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
    public void testCorrectWithPositionMeasureArrayAndCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final SlamEstimator estimator = new SlamEstimator();
            estimator.setListener(this);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double positionX = randomizer.nextDouble();
            final double positionY = randomizer.nextDouble();
            final double positionZ = randomizer.nextDouble();
            final double[] position = new double[]{positionX, positionY, positionZ};

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
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
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

            // correct providing a certain position (it is not ignored this time
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
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, correctWithPositionMeasure);
            assertEquals(1, correctedWithPositionMeasure);

            // Force IllegalArgumentException
            // wrong covariance matrix size
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

            final SlamEstimator estimator = new SlamEstimator();
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
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
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
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
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
    public void testGetSetPositionCovarianceMatrix() throws AlgebraException {

        final SlamEstimator estimator = new SlamEstimator();

        // check initial value
        assertEquals(Matrix.identity(3, 3)
                        .multiplyByScalarAndReturnNew(KalmanFilter.DEFAULT_MEASUREMENT_NOISE_VARIANCE),
                estimator.getPositionCovarianceMatrix());

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
    public void testCorrectWithPositionMeasureCoordinatesWithoutCovariance() throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final SlamEstimator estimator = new SlamEstimator();
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
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
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
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
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

            final SlamEstimator estimator = new SlamEstimator();
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
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
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
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
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
    public void testCorrectWithPositionMeasurePointWithoutCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final SlamEstimator estimator = new SlamEstimator();
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
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp,
                    0.0f, 0.0f, 0.0f);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
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
            assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
            assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
            assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);
            assertFalse(estimator.hasError());
            assertEquals(1, correctWithPositionMeasure);
            assertEquals(1, correctedWithPositionMeasure);
        }
    }

    @Test
    public void testCreateCalibrator() {
        //noinspection ConstantValue
        assertTrue(SlamEstimator.createCalibrator() instanceof SlamCalibrator);
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

            final SlamEstimator estimator = new SlamEstimator();

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
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
            final double[] gtU = new double[9];

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

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);

                }
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
    public void testPredictionConstantSpeedWithNoise() {
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0, gtPositionY = 0.0, gtPositionZ = 0.0;
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

            final SlamEstimator estimator = new SlamEstimator();

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
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
            final double[] gtU = new double[9];

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

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);

                }
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

            final SlamEstimator estimator = new SlamEstimator();

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
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
            final double[] gtU = new double[9];

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

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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

            assertTrue(filteredRotated.distanceTo(groundTruthRotated) <=
                    predictedRotated.distanceTo(groundTruthRotated));

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            assertTrue(filteredPos.distanceTo(groundTruthPos) <
                    predictedPos.distanceTo(groundTruthPos));
        }
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
            final float gtAccelerationX = 0.0f;
            final float gtAccelerationY = 0.0f;
            final float gtAccelerationZ = 0.0f;
            final float gtAngularSpeedX = randomizer.nextFloat();
            final float gtAngularSpeedY = randomizer.nextFloat();
            final float gtAngularSpeedZ = randomizer.nextFloat();
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final SlamEstimator estimator = new SlamEstimator();

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
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
            final double[] gtU = new double[9];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (int i = 0; i < 10000; i++) {
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

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a constant
                    // speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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

            // check that filtered position is closer to ground truth than predicted
            // position, and hence Kalman filter improves results
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
                    2.0 * Math.PI / (double) period * (double) (-offsetAccelerationX)));
            float gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(
                    2.0 * Math.PI / (double) period * (double) (-offsetAccelerationY)));
            float gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(
                    2.0 * Math.PI / (double) period * (double) (-offsetAccelerationZ)));
            final float gtAngularSpeedX = 0.0f;
            final float gtAngularSpeedY = 0.0f;
            final float gtAngularSpeedZ = 0.0f;
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final SlamEstimator estimator = new SlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
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
            final double[] gtU = new double[9];

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
                        2.0 * Math.PI / (double) period * (double) (i - offsetAccelerationX)));
                gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(
                        2.0 * Math.PI / (double) period * (double) (i - offsetAccelerationY)));
                gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(
                        2.0 * Math.PI / (double) period * (double) (i - offsetAccelerationZ)));

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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
                    2.0 * Math.PI / (double) period * (double) (-offsetAngularSpeedX)));
            float gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(
                    2.0 * Math.PI / (double) period * (double) (-offsetAngularSpeedY)));
            float gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(
                    2.0 * Math.PI / (double) period * (double) (-offsetAngularSpeedZ)));
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final SlamEstimator estimator = new SlamEstimator();

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
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
            final double[] gtU = new double[9];

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
                        2.0 * Math.PI / (double) period * (double) (i - offsetAngularSpeedX)));
                gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(
                        2.0 * Math.PI / (double) period * (double) (i - offsetAngularSpeedY)));
                gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(
                        2.0 * Math.PI / (double) period * (double) (i - offsetAngularSpeedZ)));

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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

            final SlamEstimator estimator = new SlamEstimator();

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
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
            final double[] gtU = new double[9];

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

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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
                    2.0 * Math.PI / (double) period * (double) (-offsetAccelerationX)));
            float gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(
                    2.0 * Math.PI / (double) period * (double) (-offsetAccelerationY)));
            float gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(
                    2.0 * Math.PI / (double) period * (double) (-offsetAccelerationZ)));
            final float gtAngularSpeedX = randomizer.nextFloat();
            final float gtAngularSpeedY = randomizer.nextFloat();
            final float gtAngularSpeedZ = randomizer.nextFloat();
            final Quaternion gtQuaternion = new Quaternion(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());
            final float gtQuaternionA = (float) gtQuaternion.getA();
            final float gtQuaternionB = (float) gtQuaternion.getB();
            final float gtQuaternionC = (float) gtQuaternion.getC();
            final float gtQuaternionD = (float) gtQuaternion.getD();

            final SlamEstimator estimator = new SlamEstimator();

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
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
            final double[] gtU = new double[9];

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
                        2.0 * Math.PI / (double) period * (double) (i - offsetAccelerationX)));
                gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(
                        2.0 * Math.PI / (double) period * (double) (i - offsetAccelerationY)));
                gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(
                        2.0 * Math.PI / (double) period * (double) (i - offsetAccelerationZ)));

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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
                    2.0 * Math.PI / (double) period * (double) (-offsetAngularSpeedX)));
            float gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(
                    2.0 * Math.PI / (double) period * (double) (-offsetAngularSpeedY)));
            float gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(
                    2.0 * Math.PI / (double) period * (double) (-offsetAngularSpeedZ)));
            final Quaternion gtQuaternion = new Quaternion(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());
            final float gtQuaternionA = (float) gtQuaternion.getA();
            final float gtQuaternionB = (float) gtQuaternion.getB();
            final float gtQuaternionC = (float) gtQuaternion.getC();
            final float gtQuaternionD = (float) gtQuaternion.getD();

            final SlamEstimator estimator = new SlamEstimator();

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
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
            final double[] gtU = new double[9];

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
                        2.0 * Math.PI / (double) period * (double) (i - offsetAngularSpeedX)));
                gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(
                        2.0 * Math.PI / (double) period * (double) (i - offsetAngularSpeedY)));
                gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(
                        2.0 * Math.PI / (double) period * (double) (i - offsetAngularSpeedZ)));

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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
    public void testPredictionRandomAccelerationAndAngularSpeedWithNoise() {
        // when acceleration or angular speed has random values (abrupt changes)
        // Kalman Filter might obtain worse results because those changes are
        // treated as noise
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
            final float gtQuaternionB = (float) gtQuaternion.getB();
            final float gtQuaternionC = (float) gtQuaternion.getC();
            final float gtQuaternionD = (float) gtQuaternion.getD();

            final SlamEstimator estimator = new SlamEstimator();

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
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
            final double[] gtU = new double[9];

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

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

                    deltaGtAccelerationX = gtAccelerationX - lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY - lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ - lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX - lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY - lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ - lastGtAngularSpeedZ;

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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
        for (int t = 0; t < REPEAT_TIMES; t++) {
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

            final SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                    angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final SlamCalibrationData calibration = calibrator.getCalibrationData();

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

            final SlamEstimator estimator = new SlamEstimator();

            final SlamEstimator estimatorWithCalibration = new SlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            float accelerationWithOffsetX;
            float accelerationWithOffsetY;
            float accelerationWithOffsetZ;
            float angularSpeedWithOffsetX;
            float angularSpeedWithOffsetY;
            float angularSpeedWithOffsetZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
            final double[] uWithOffset = new double[9];
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
            final double[] gtU = new double[9];

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

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY, accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, angularSpeedWithOffsetZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

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

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    // we also take into account control signals with offset
                    uWithOffset[0] = uWithOffset[1] = uWithOffset[2] = 0.0;
                    uWithOffset[3] = deltaAccelerationWithOffsetX;
                    uWithOffset[4] = deltaAccelerationWithOffsetY;
                    uWithOffset[5] = deltaAccelerationWithOffsetZ;
                    uWithOffset[6] = deltaAngularSpeedWithOffsetX;
                    uWithOffset[7] = deltaAngularSpeedWithOffsetY;
                    uWithOffset[8] = deltaAngularSpeedWithOffsetZ;
                    StatePredictor.predict(xWithOffset, uWithOffset, DELTA_SECONDS, xWithOffset);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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

        assertTrue(numSuccess >= REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionConstantSpeedWithNoiseAndCalibration() {
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

            final SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                    angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final SlamCalibrationData calibration = calibrator.getCalibrationData();

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

            final SlamEstimator estimator = new SlamEstimator();

            final SlamEstimator estimatorWithCalibration = new SlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            float accelerationWithOffsetX;
            float accelerationWithOffsetY;
            float accelerationWithOffsetZ;
            float angularSpeedWithOffsetX;
            float angularSpeedWithOffsetY;
            float angularSpeedWithOffsetZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
            final double[] uWithOffset = new double[9];
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
            final double[] gtU = new double[9];

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

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY, accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, angularSpeedWithOffsetZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

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

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    // we also take into account control signals with offset
                    uWithOffset[0] = uWithOffset[1] = uWithOffset[2] = 0.0;
                    uWithOffset[3] = deltaAccelerationWithOffsetX;
                    uWithOffset[4] = deltaAccelerationWithOffsetY;
                    uWithOffset[5] = deltaAccelerationWithOffsetZ;
                    uWithOffset[6] = deltaAngularSpeedWithOffsetX;
                    uWithOffset[7] = deltaAngularSpeedWithOffsetY;
                    uWithOffset[8] = deltaAngularSpeedWithOffsetZ;
                    StatePredictor.predict(xWithOffset, uWithOffset, DELTA_SECONDS,
                            xWithOffset);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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
    public void testPredictionConstantAccelerationWithNoiseAndCalibration() {
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

            final SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                    angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final SlamCalibrationData calibration = calibrator.getCalibrationData();

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

            final SlamEstimator estimator = new SlamEstimator();

            final SlamEstimator estimatorWithCalibration = new SlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            float accelerationWithOffsetX;
            float accelerationWithOffsetY;
            float accelerationWithOffsetZ;
            float angularSpeedWithOffsetX;
            float angularSpeedWithOffsetY;
            float angularSpeedWithOffsetZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
            final double[] uWithOffset = new double[9];
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
            final double[] gtU = new double[9];

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

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY, accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, angularSpeedWithOffsetZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

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

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    // we also take into account control signals with offset
                    uWithOffset[0] = uWithOffset[1] = uWithOffset[2] = 0.0;
                    uWithOffset[3] = deltaAccelerationWithOffsetX;
                    uWithOffset[4] = deltaAccelerationWithOffsetY;
                    uWithOffset[5] = deltaAccelerationWithOffsetZ;
                    uWithOffset[6] = deltaAngularSpeedWithOffsetX;
                    uWithOffset[7] = deltaAngularSpeedWithOffsetY;
                    uWithOffset[8] = deltaAngularSpeedWithOffsetZ;
                    StatePredictor.predict(xWithOffset, uWithOffset, DELTA_SECONDS, xWithOffset);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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

        assertTrue(numSuccess >= REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
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

            final SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                    angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final SlamCalibrationData calibration = calibrator.getCalibrationData();

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
            final float gtAngularSpeedX = randomizer.nextFloat();
            final float gtAngularSpeedY = randomizer.nextFloat();
            final float gtAngularSpeedZ = randomizer.nextFloat();
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final SlamEstimator estimator = new SlamEstimator();

            final SlamEstimator estimatorWithCalibration = new SlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            float accelerationWithOffsetX;
            float accelerationWithOffsetY;
            float accelerationWithOffsetZ;
            float angularSpeedWithOffsetX;
            float angularSpeedWithOffsetY;
            float angularSpeedWithOffsetZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
            final double[] uWithOffset = new double[9];
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
            final double[] gtU = new double[9];

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
            for (int i = 0; i < 10000; i++) {
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

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY, accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, angularSpeedWithOffsetZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

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

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a constant
                    // speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    // we also take into account control signals with offset
                    uWithOffset[0] = uWithOffset[1] = uWithOffset[2] = 0.0;
                    uWithOffset[3] = deltaAccelerationWithOffsetX;
                    uWithOffset[4] = deltaAccelerationWithOffsetY;
                    uWithOffset[5] = deltaAccelerationWithOffsetZ;
                    uWithOffset[6] = deltaAngularSpeedWithOffsetX;
                    uWithOffset[7] = deltaAngularSpeedWithOffsetY;
                    uWithOffset[8] = deltaAngularSpeedWithOffsetZ;
                    StatePredictor.predict(xWithOffset, uWithOffset, DELTA_SECONDS, xWithOffset);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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

            final Quaternion filteredQuaternion = estimatorWithCalibration.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final boolean rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than predicted
            // position, and hence Kalman filter improves results
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

        assertTrue(numSuccess >= REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
    }

    @Test
    public void testPredictionVariableAccelerationWithNoiseAndCalibration() {
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

            final SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                    angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final SlamCalibrationData calibration = calibrator.getCalibrationData();

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
                    2.0 * Math.PI / (double) period * (double) (-offsetAccelerationX)));
            float gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(
                    2.0 * Math.PI / (double) period * (double) (-offsetAccelerationY)));
            float gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(
                    2.0 * Math.PI / (double) period * (double) (-offsetAccelerationZ)));
            final float gtAngularSpeedX = 0.0f;
            final float gtAngularSpeedY = 0.0f;
            final float gtAngularSpeedZ = 0.0f;
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final SlamEstimator estimator = new SlamEstimator();

            final SlamEstimator estimatorWithCalibration = new SlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            float accelerationWithOffsetX;
            float accelerationWithOffsetY;
            float accelerationWithOffsetZ;
            float angularSpeedWithOffsetX;
            float angularSpeedWithOffsetY;
            float angularSpeedWithOffsetZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
            final double[] uWithOffset = new double[9];
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
            final double[] gtU = new double[9];

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
                        2.0 * Math.PI / (double) period * (double) (i - offsetAccelerationX)));
                gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(
                        2.0 * Math.PI / (double) period * (double) (i - offsetAccelerationY)));
                gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(
                        2.0 * Math.PI / (double) period * (double) (i - offsetAccelerationZ)));

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

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY, accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, angularSpeedWithOffsetZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

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

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    // we also take into account control signals with offset
                    uWithOffset[0] = uWithOffset[1] = uWithOffset[2] = 0.0;
                    uWithOffset[3] = deltaAccelerationWithOffsetX;
                    uWithOffset[4] = deltaAccelerationWithOffsetY;
                    uWithOffset[5] = deltaAccelerationWithOffsetZ;
                    uWithOffset[6] = deltaAngularSpeedWithOffsetX;
                    uWithOffset[7] = deltaAngularSpeedWithOffsetY;
                    uWithOffset[8] = deltaAngularSpeedWithOffsetZ;
                    StatePredictor.predict(xWithOffset, uWithOffset, DELTA_SECONDS, xWithOffset);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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
    public void testPredictionVariableAngularSpeedWithNoiseAndCalibration() {
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

            final SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                    angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final SlamCalibrationData calibration = calibrator.getCalibrationData();

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
                    2.0 * Math.PI / (double) period * (double) (-offsetAngularSpeedX)));
            float gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(
                    2.0 * Math.PI / (double) period * (double) (-offsetAngularSpeedY)));
            float gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(
                    2.0 * Math.PI / (double) period * (double) (-offsetAngularSpeedZ)));
            final float gtQuaternionA = 1.0f;
            final float gtQuaternionB = 0.0f;
            final float gtQuaternionC = 0.0f;
            final float gtQuaternionD = 0.0f;

            final SlamEstimator estimator = new SlamEstimator();

            final SlamEstimator estimatorWithCalibration = new SlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX;
            float accelerationY;
            float accelerationZ;
            float angularSpeedX;
            float angularSpeedY;
            float angularSpeedZ;
            float accelerationWithOffsetX;
            float accelerationWithOffsetY;
            float accelerationWithOffsetZ;
            float angularSpeedWithOffsetX;
            float angularSpeedWithOffsetY;
            float angularSpeedWithOffsetZ;
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
            final double[] x = new double[16];
            final double[] u = new double[9];
            final double[] uWithOffset = new double[9];
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
            final double[] gtU = new double[9];

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
                        2.0 * Math.PI / (double) period * (double) (i - offsetAngularSpeedX)));
                gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(
                        2.0 * Math.PI / (double) period * (double) (i - offsetAngularSpeedY)));
                gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(
                        2.0 * Math.PI / (double) period * (double) (i - offsetAngularSpeedZ)));

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;

                angularSpeedWithOffsetX = angularSpeedX + angularOffsetX;
                angularSpeedWithOffsetY = angularSpeedY + angularOffsetY;
                angularSpeedWithOffsetZ = angularSpeedZ + angularOffsetZ;

                estimator.updateAccelerometerSample(timestamp, accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, angularSpeedZ);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY, accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, angularSpeedWithOffsetZ);

                timestamp += DELTA_NANOS;

                if (i != 0) {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

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

                    // delta acceleration and angular speed only contains noise,
                    // since no real change in those values is present on a
                    // constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    // we also take into account control signals with offset
                    uWithOffset[0] = uWithOffset[1] = uWithOffset[2] = 0.0;
                    uWithOffset[3] = deltaAccelerationWithOffsetX;
                    uWithOffset[4] = deltaAccelerationWithOffsetY;
                    uWithOffset[5] = deltaAccelerationWithOffsetZ;
                    uWithOffset[6] = deltaAngularSpeedWithOffsetX;
                    uWithOffset[7] = deltaAngularSpeedWithOffsetY;
                    uWithOffset[8] = deltaAngularSpeedWithOffsetZ;
                    StatePredictor.predict(xWithOffset, uWithOffset, DELTA_SECONDS, xWithOffset);

                    gtU[0] = gtU[1] = gtU[2] = 0.0;
                    gtU[3] = deltaGtAccelerationX;
                    gtU[4] = deltaGtAccelerationY;
                    gtU[5] = deltaGtAccelerationZ;
                    gtU[6] = deltaGtAngularSpeedX;
                    gtU[7] = deltaGtAngularSpeedY;
                    gtU[8] = deltaGtAngularSpeedZ;
                    StatePredictor.predict(gtX, gtU, DELTA_SECONDS, gtX);
                }
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

            final Quaternion filteredQuaternion = estimatorWithCalibration.getStateQuaternion();
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

        assertTrue(numSuccess >= REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final SlamEstimator estimator1 = new SlamEstimator();

        // set new values
        final SlamCalibrationData data = new SlamCalibrationData();
        estimator1.setCalibrationData(data);
        estimator1.mStatePositionX = 1.0;
        estimator1.mStatePositionY = 2.0;
        estimator1.mStatePositionZ = 3.0;
        estimator1.mStateVelocityX = 4.0;
        estimator1.mStateVelocityY = 5.0;
        estimator1.mStateVelocityZ = 6.0;
        estimator1.mStateAccelerationX = 7.0;
        estimator1.mStateAccelerationY = 8.0;
        estimator1.mStateAccelerationZ = 9.0;
        estimator1.mStateQuaternionA = 0.1;
        estimator1.mStateQuaternionB = 0.2;
        estimator1.mStateQuaternionC = 0.3;
        estimator1.mStateQuaternionD = 0.4;
        estimator1.mStateAngularSpeedX = 10.0;
        estimator1.mStateAngularSpeedY = 11.0;
        estimator1.mStateAngularSpeedZ = 12.0;
        estimator1.mError = true;
        estimator1.mAccumulationEnabled = true;
        estimator1.mAccelerometerTimestampNanos = 1L;
        estimator1.mGyroscopeTimestampNanos = 2L;
        estimator1.mAccumulatedAccelerometerSamples = 100;
        estimator1.mAccumulatedGyroscopeSamples = 200;
        estimator1.mAccumulatedAccelerationSampleX = -1.0;
        estimator1.mAccumulatedAccelerationSampleY = -2.0;
        estimator1.mAccumulatedAccelerationSampleZ = -3.0;
        estimator1.mAccumulatedAngularSpeedSampleX = -4.0;
        estimator1.mAccumulatedAngularSpeedSampleY = -5.0;
        estimator1.mAccumulatedAngularSpeedSampleZ = -6.0;

        // check
        assertSame(data, estimator1.getCalibrationData());
        assertEquals(1.0, estimator1.mStatePositionX, 0.0);
        assertEquals(2.0, estimator1.mStatePositionY, 0.0);
        assertEquals(3.0, estimator1.mStatePositionZ, 0.0);
        assertEquals(4.0, estimator1.mStateVelocityX, 0.0);
        assertEquals(5.0, estimator1.mStateVelocityY, 0.0);
        assertEquals(6.0, estimator1.mStateVelocityZ, 0.0);
        assertEquals(7.0, estimator1.mStateAccelerationX, 0.0);
        assertEquals(8.0, estimator1.mStateAccelerationY, 0.0);
        assertEquals(9.0, estimator1.mStateAccelerationZ, 0.0);
        assertEquals(0.1, estimator1.mStateQuaternionA, 0.0);
        assertEquals(0.2, estimator1.mStateQuaternionB, 0.0);
        assertEquals(0.3, estimator1.mStateQuaternionC, 0.0);
        assertEquals(0.4, estimator1.mStateQuaternionD, 0.0);
        assertEquals(10.0, estimator1.mStateAngularSpeedX, 0.0);
        assertEquals(11.0, estimator1.mStateAngularSpeedY, 0.0);
        assertEquals(12.0, estimator1.mStateAngularSpeedZ, 0.0);
        assertTrue(estimator1.hasError());
        assertTrue(estimator1.isAccumulationEnabled());
        assertEquals(1L, estimator1.getAccelerometerTimestampNanos());
        assertEquals(2L, estimator1.getGyroscopeTimestampNanos());
        assertEquals(100, estimator1.getAccumulatedAccelerometerSamples());
        assertEquals(200, estimator1.getAccumulatedGyroscopeSamples());
        assertEquals(-1.0, estimator1.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(-2.0, estimator1.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(-3.0, estimator1.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(-4.0, estimator1.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(-5.0, estimator1.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(-6.0, estimator1.getAccumulatedAngularSpeedSampleZ(), 0.0);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(estimator1);
        final SlamEstimator estimator2 = SerializationHelper.deserialize(bytes);

        // check
        assertNotSame(estimator1.getCalibrationData(), estimator2.getCalibrationData());
        assertEquals(estimator1.mStatePositionX, estimator2.mStatePositionX, 0.0);
        assertEquals(estimator1.mStatePositionY, estimator2.mStatePositionY, 0.0);
        assertEquals(estimator1.mStatePositionZ, estimator2.mStatePositionZ, 0.0);
        assertEquals(estimator1.mStateVelocityX, estimator2.mStateVelocityX, 0.0);
        assertEquals(estimator1.mStateVelocityY, estimator2.mStateVelocityY, 0.0);
        assertEquals(estimator1.mStateVelocityZ, estimator2.mStateVelocityZ, 0.0);
        assertEquals(estimator1.mStateAccelerationX, estimator2.mStateAccelerationX, 0.0);
        assertEquals(estimator1.mStateAccelerationY, estimator2.mStateAccelerationY, 0.0);
        assertEquals(estimator1.mStateAccelerationZ, estimator2.mStateAccelerationZ, 0.0);
        assertEquals(estimator1.mStateQuaternionA, estimator2.mStateQuaternionA, 0.0);
        assertEquals(estimator1.mStateQuaternionB, estimator2.mStateQuaternionB, 0.0);
        assertEquals(estimator1.mStateQuaternionC, estimator2.mStateQuaternionC, 0.0);
        assertEquals(estimator1.mStateQuaternionD, estimator2.mStateQuaternionD, 0.0);
        assertEquals(estimator1.mStateAngularSpeedX, estimator2.mStateAngularSpeedX, 0.0);
        assertEquals(estimator1.mStateAngularSpeedY, estimator2.mStateAngularSpeedY, 0.0);
        assertEquals(estimator1.mStateAngularSpeedZ, estimator2.mStateAngularSpeedZ, 0.0);
        assertEquals(estimator1.hasError(), estimator2.hasError());
        assertEquals(estimator1.isAccumulationEnabled(), estimator2.isAccumulationEnabled());
        assertEquals(estimator1.getAccelerometerTimestampNanos(), estimator2.getAccelerometerTimestampNanos());
        assertEquals(estimator1.getGyroscopeTimestampNanos(), estimator2.getGyroscopeTimestampNanos());
        assertEquals(estimator1.getAccumulatedAccelerometerSamples(),
                estimator2.getAccumulatedAccelerometerSamples());
        assertEquals(estimator1.getAccumulatedGyroscopeSamples(), estimator2.getAccumulatedGyroscopeSamples());
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
    }

    @Override
    public void onFullSampleReceived(final BaseSlamEstimator<SlamCalibrationData> estimator) {
        fullSampleReceived++;
    }

    @Override
    public void onFullSampleProcessed(final BaseSlamEstimator<SlamCalibrationData> estimator) {
        fullSampleProcessed++;
    }

    @Override
    public void onCorrectWithPositionMeasure(final BaseSlamEstimator<SlamCalibrationData> estimator) {
        correctWithPositionMeasure++;
    }

    @Override
    public void onCorrectedWithPositionMeasure(final BaseSlamEstimator<SlamCalibrationData> estimator) {
        correctedWithPositionMeasure++;
    }

    private void reset() {
        fullSampleReceived = fullSampleProcessed = correctWithPositionMeasure =
                correctedWithPositionMeasure = 0;
    }

    private SlamCalibrator createFinishedCalibrator(
            final float accelerationOffsetX, final float accelerationOffsetY, final float accelerationOffsetZ,
            final float angularOffsetX, final float angularOffsetY, final float angularOffsetZ,
            final GaussianRandomizer noiseRandomizer) {
        final SlamCalibrator calibrator = SlamEstimator.createCalibrator();
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
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY, (float) angularZ);

            if (calibrator.isFinished()) {
                break;
            }

            timestamp += DELTA_NANOS;
        }

        return calibrator;
    }
}
