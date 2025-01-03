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
import com.irurueta.geometry.Quaternion;
import com.irurueta.numerical.signal.processing.KalmanFilter;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.jupiter.api.Assertions.*;

class AbsoluteOrientationSlamEstimatorTest implements
        BaseSlamEstimatorListener<AbsoluteOrientationSlamCalibrationData> {

    private static final int TIMES = 50;
    private static final double ABSOLUTE_ERROR = 1e-8;

    // conversion from milliseconds to nanoseconds
    private static final int MILLIS_TO_NANOS = 1000000;

    // time between samples expressed in nanoseconds (a typical sensor in Android
    // delivers a sample every 20ms)
    private static final int DELTA_NANOS = 20000000; // 0.02 seconds
    private static final double DELTA_SECONDS = 0.02; // 0.2 seconds

    // gain below is only valid when measure variance is 1e-8 and process noise
    // variance is 1e-6. This is the gain component of the Kalman filter for
    // those error covariances. This gain will change for other values
    private static final double VELOCITY_GAIN = 0.019991983014891204; //0.1923075055475187

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
    void testConstructor() throws WrongSizeException {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        // check initial values
        assertNull(estimator.getListener());
        assertNull(estimator.getCalibrationData());
        assertEquals(0.0, estimator.getStatePositionX(), 0.0);
        assertEquals(0.0, estimator.getStatePositionY(), 0.0);
        assertEquals(0.0, estimator.getStatePositionZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getStatePosition(), 0.0);
        final var position = new double[3];
        estimator.getStatePosition(position);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, position, 0.0);

        assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getStateVelocity(), 0.0);
        final var velocity = new double[3];
        estimator.getStateVelocity(velocity);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, velocity, 0.0);

        assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
        assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
        assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getStateAcceleration(), 0.0);
        final var acceleration = new double[3];
        estimator.getStateAcceleration(acceleration);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, acceleration, 0.0);

        assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);
        assertArrayEquals(new double[]{1.0, 0.0, 0.0, 0.0}, estimator.getStateQuaternionArray(), 0.0);
        final var quaternionArray = new double[4];
        estimator.getStateQuaternionArray(quaternionArray);
        assertArrayEquals(new double[]{1.0, 0.0, 0.0, 0.0}, quaternionArray, 0.0);

        var q = estimator.getStateQuaternion();
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
        final var angularSpeed = new double[3];
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
        final var accumAcc = new double[3];
        estimator.getAccumulatedAccelerationSample(accumAcc);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, accumAcc, 0.0);

        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getAccumulatedAngularSpeedSample(), 0.0);
        final var accumAngularSpeed = new double[3];
        estimator.getAccumulatedAngularSpeedSample(accumAngularSpeed);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, accumAngularSpeed, 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());

        assertEquals(estimator.getPositionCovarianceMatrix(),
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        KalmanFilter.DEFAULT_MEASUREMENT_NOISE_VARIANCE));

        assertEquals(-1, estimator.getOrientationTimestampNanos());
        assertInstanceOf(Quaternion.class, estimator.getAccumulatedOrientation());
        var accumulatedOrientation = (Quaternion) estimator.getAccumulatedOrientation();
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
    void testGetSetListener() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        // initial value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetCalibrationData() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        // initial value
        assertNull(estimator.getCalibrationData());

        // set new value
        final var data = new AbsoluteOrientationSlamCalibrationData();
        estimator.setCalibrationData(data);

        // check correctness
        assertSame(data, estimator.getCalibrationData());
    }

    @Test
    void testResetPosition() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        estimator.resetPosition();

        assertEquals(0.0, estimator.getStatePositionX(), 0.0);
        assertEquals(0.0, estimator.getStatePositionY(), 0.0);
        assertEquals(0.0, estimator.getStatePositionZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    void testResetVelocity() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        estimator.resetVelocity();

        assertEquals(0.0, estimator.getStateVelocityX(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityY(), 0.0);
        assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    void testResetPositionAndVelocity() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

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
    void testResetAcceleration() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        estimator.resetAcceleration();

        assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);
        assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);
        assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    void testResetOrientation() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        estimator.resetOrientation();

        assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);
        assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    void testResetAngularSpeed() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        estimator.resetAngularSpeed();

        assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);
        assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);
        assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);

        assertEquals(-1, estimator.getMostRecentTimestampNanos());
    }

    @Test
    void testReset() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

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
    void testGetStatePositionX() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(0.0, estimator.getStatePositionX(), 0.0);

        final var value = randomizer.nextDouble();
        estimator.statePositionX = value;

        // check correctness
        assertEquals(value, estimator.getStatePositionX(), 0.0);
    }

    @Test
    void testGetStatePositionY() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(0.0, estimator.getStatePositionY(), 0.0);

        final var value = randomizer.nextDouble();
        estimator.statePositionY = value;

        // check correctness
        assertEquals(value, estimator.getStatePositionY(), 0.0);
    }

    @Test
    void testGetStatePositionZ() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(0.0, estimator.getStatePositionZ(), 0.0);

        final var value = randomizer.nextDouble();
        estimator.statePositionZ = value;

        // check correctness
        assertEquals(value, estimator.getStatePositionZ(), 0.0);
    }

    @Test
    void testGetStatePosition() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();
        estimator.statePositionX = valueX;
        estimator.statePositionY = valueY;
        estimator.statePositionZ = valueZ;

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, estimator.getStatePosition(), 0.0);

        final var position = new double[3];
        estimator.getStatePosition(position);

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, position, 0.0);

        // Force IllegalArgumentException
        final var wrong = new double[4];
        assertThrows(IllegalArgumentException.class, () -> estimator.getStatePosition(wrong));
    }

    @Test
    void testGetStateVelocityX() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(0.0, estimator.getStateVelocityX(), 0.0);

        final var value = randomizer.nextDouble();
        estimator.stateVelocityX = value;

        // check correctness
        assertEquals(value, estimator.getStateVelocityX(), 0.0);
    }

    @Test
    void testGetStateVelocityY() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(0.0, estimator.getStateVelocityY(), 0.0);

        final var value = randomizer.nextDouble();
        estimator.stateVelocityY = value;

        // check correctness
        assertEquals(value, estimator.getStateVelocityY(), 0.0);
    }

    @Test
    void testGetStateVelocityZ() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(0.0, estimator.getStateVelocityZ(), 0.0);

        final var value = randomizer.nextDouble();
        estimator.stateVelocityZ = value;

        // check correctness
        assertEquals(value, estimator.getStateVelocityZ(), 0.0);
    }

    @Test
    void testGetStateVelocity() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();
        estimator.stateVelocityX = valueX;
        estimator.stateVelocityY = valueY;
        estimator.stateVelocityZ = valueZ;

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, estimator.getStateVelocity(), 0.0);

        final var velocity = new double[3];
        estimator.getStateVelocity(velocity);

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, velocity, 0.0);

        // Force IllegalArgumentException
        final var wrong = new double[4];
        assertThrows(IllegalArgumentException.class, () -> estimator.getStateVelocity(wrong));
    }

    @Test
    void testGetStateAccelerationX() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(0.0, estimator.getStateAccelerationX(), 0.0);

        final var value = randomizer.nextDouble();
        estimator.stateAccelerationX = value;

        // check correctness
        assertEquals(value, estimator.getStateAccelerationX(), 0.0);
    }

    @Test
    void testGetStateAccelerationY() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(0.0, estimator.getStateAccelerationY(), 0.0);

        final var value = randomizer.nextDouble();
        estimator.stateAccelerationY = value;

        // check correctness
        assertEquals(value, estimator.getStateAccelerationY(), 0.0);
    }

    @Test
    void testGetStateAccelerationZ() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(0.0, estimator.getStateAccelerationZ(), 0.0);

        final var value = randomizer.nextDouble();
        estimator.stateAccelerationZ = value;

        // check correctness
        assertEquals(value, estimator.getStateAccelerationZ(), 0.0);
    }

    @Test
    void testGetStateAcceleration() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();
        estimator.stateAccelerationX = valueX;
        estimator.stateAccelerationY = valueY;
        estimator.stateAccelerationZ = valueZ;

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, estimator.getStateAcceleration(), 0.0);

        final var acceleration = new double[3];
        estimator.getStateAcceleration(acceleration);

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, acceleration, 0.0);

        // Force IllegalArgumentException
        final var wrong = new double[4];
        assertThrows(IllegalArgumentException.class, () -> estimator.getStateAcceleration(wrong));
    }

    @Test
    void testGetStateQuaternionA() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial value
        assertEquals(1.0, estimator.getStateQuaternionA(), 0.0);

        // set new value
        final var value = randomizer.nextDouble();
        estimator.stateQuaternionA = value;

        // check correctness
        assertEquals(value, estimator.getStateQuaternionA(), 0.0);
    }

    @Test
    void testGetStateQuaternionB() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial value
        assertEquals(0.0, estimator.getStateQuaternionB(), 0.0);

        // set new value
        final var value = randomizer.nextDouble();
        estimator.stateQuaternionB = value;

        // check correctness
        assertEquals(value, estimator.getStateQuaternionB(), 0.0);
    }

    @Test
    void testGetStateQuaternionC() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial value
        assertEquals(0.0, estimator.getStateQuaternionC(), 0.0);

        // set new value
        final var value = randomizer.nextDouble();
        estimator.stateQuaternionC = value;

        // check correctness
        assertEquals(value, estimator.getStateQuaternionC(), 0.0);
    }

    @Test
    void testGetStateQuaternionD() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial value
        assertEquals(0.0, estimator.getStateQuaternionD(), 0.0);

        // set new value
        final var value = randomizer.nextDouble();
        estimator.stateQuaternionD = value;

        // check correctness
        assertEquals(value, estimator.getStateQuaternionD(), 0.0);
    }

    @Test
    void testGetStateQuaternionArray() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial value
        assertArrayEquals(new double[]{1.0, 0.0, 0.0, 0.0}, estimator.getStateQuaternionArray(), 0.0);

        // set new values
        final var valueA = randomizer.nextDouble();
        final var valueB = randomizer.nextDouble();
        final var valueC = randomizer.nextDouble();
        final var valueD = randomizer.nextDouble();
        estimator.stateQuaternionA = valueA;
        estimator.stateQuaternionB = valueB;
        estimator.stateQuaternionC = valueC;
        estimator.stateQuaternionD = valueD;

        // check correctness
        assertArrayEquals(new double[]{valueA, valueB, valueC, valueD}, estimator.getStateQuaternionArray(), 0.0);

        final var quaternion = new double[4];
        estimator.getStateQuaternionArray(quaternion);

        assertArrayEquals(new double[]{valueA, valueB, valueC, valueD}, quaternion, 0.0);

        // Force IllegalArgumentException
        final var wrong = new double[5];
        assertThrows(IllegalArgumentException.class, () -> estimator.getStateQuaternionArray(wrong));
    }

    @Test
    void testGetStateQuaternion() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial value
        var q = estimator.getStateQuaternion();

        assertEquals(1.0, q.getA(), 0.0);
        assertEquals(0.0, q.getB(), 0.0);
        assertEquals(0.0, q.getC(), 0.0);
        assertEquals(0.0, q.getD(), 0.0);

        // set new values
        final var valueA = randomizer.nextDouble();
        final var valueB = randomizer.nextDouble();
        final var valueC = randomizer.nextDouble();
        final var valueD = randomizer.nextDouble();
        estimator.stateQuaternionA = valueA;
        estimator.stateQuaternionB = valueB;
        estimator.stateQuaternionC = valueC;
        estimator.stateQuaternionD = valueD;

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
    void testGetStateAngularSpeedX() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial value
        assertEquals(0.0, estimator.getStateAngularSpeedX(), 0.0);

        // set new value
        final var value = randomizer.nextDouble();
        estimator.stateAngularSpeedX = value;

        // check correctness
        assertEquals(value, estimator.getStateAngularSpeedX(), 0.0);
    }

    @Test
    void testGetStateAngularSpeedY() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial value
        assertEquals(0.0, estimator.getStateAngularSpeedY(), 0.0);

        // set new value
        final var value = randomizer.nextDouble();
        estimator.stateAngularSpeedY = value;

        // check correctness
        assertEquals(value, estimator.getStateAngularSpeedY(), 0.0);
    }

    @Test
    void testGetStateAngularSpeedZ() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial value
        assertEquals(0.0, estimator.getStateAngularSpeedZ(), 0.0);

        // set new value
        final var value = randomizer.nextDouble();
        estimator.stateAngularSpeedZ = value;

        // check correctness
        assertEquals(value, estimator.getStateAngularSpeedZ(), 0.0);
    }

    @Test
    void testGetStateAngularSpeed() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        // check initial value
        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getStateAngularSpeed(), 0.0);

        // set new value
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();
        estimator.stateAngularSpeedX = valueX;
        estimator.stateAngularSpeedY = valueY;
        estimator.stateAngularSpeedZ = valueZ;

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, estimator.getStateAngularSpeed(), 0.0);

        final var angularSpeed = new double[3];
        estimator.getStateAngularSpeed(angularSpeed);

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, angularSpeed, 0.0);

        // Force IllegalArgumentException
        final var wrong = new double[4];
        assertThrows(IllegalArgumentException.class, () -> estimator.getStateAngularSpeed(wrong));
    }

    @Test
    void testHasError() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        assertFalse(estimator.hasError());

        estimator.error = true;

        assertTrue(estimator.hasError());
    }

    @Test
    void testIsAccumulationEnabled() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        assertTrue(estimator.isAccumulationEnabled());

        estimator.setAccumulationEnabled(false);

        assertFalse(estimator.isAccumulationEnabled());
    }

    @Test
    void testGetAccelerometerTimestampNanos() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(-1, estimator.getAccelerometerTimestampNanos());

        // set new value
        estimator.accelerometerTimestampNanos = 1000;

        // check correctness
        assertEquals(1000, estimator.getAccelerometerTimestampNanos());
    }

    @Test
    void testGetGyroscopeTimestampNanos() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(-1, estimator.getGyroscopeTimestampNanos());

        // set new value
        estimator.gyroscopeTimestampNanos = 2000;

        // check correctness
        assertEquals(2000, estimator.getGyroscopeTimestampNanos());
    }

    @Test
    void testGetOrientationTimestampNanos() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(-1, estimator.getOrientationTimestampNanos());

        // set new value
        estimator.orientationTimestampNanos = 3000;

        // check correctness
        assertEquals(3000, estimator.getOrientationTimestampNanos());
    }

    @Test
    void testGetAccumulatedAccelerometerSamplesAndIsAccelerometerSampleReceived() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(0, estimator.getAccumulatedAccelerometerSamples());
        assertFalse(estimator.isAccelerometerSampleReceived());

        // set new value
        estimator.accumulatedAccelerometerSamples = 10;

        // check correctness
        assertEquals(10, estimator.getAccumulatedAccelerometerSamples());
        assertTrue(estimator.isAccelerometerSampleReceived());
    }

    @Test
    void testGetAccumulatedGyroscopeSamplesAndIsGyroscopeSampleReceived() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(0, estimator.getAccumulatedGyroscopeSamples());
        assertFalse(estimator.isGyroscopeSampleReceived());

        // set new value
        estimator.accumulatedGyroscopeSamples = 20;

        // check correctness
        assertEquals(20, estimator.getAccumulatedGyroscopeSamples());
        assertTrue(estimator.isGyroscopeSampleReceived());
    }

    @Test
    void testGetAccumulatedOrientationAndIsOrientationSampleReceived() {
        final var randomizer = new UniformRandomizer();

        final var estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(0, estimator.getAccumulatedOrientationSamples());
        assertFalse(estimator.isOrientationSampleReceived());

        // set new value
        final var q = new Quaternion(randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        estimator.accumulatedOrientation = q;

        // check correctness
        final var accumulatedOrientation = (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(q.getA(), accumulatedOrientation.getA(), ABSOLUTE_ERROR);
        assertEquals(q.getB(), accumulatedOrientation.getB(), ABSOLUTE_ERROR);
        assertEquals(q.getC(), accumulatedOrientation.getC(), ABSOLUTE_ERROR);
        assertEquals(q.getD(), accumulatedOrientation.getD(), ABSOLUTE_ERROR);
    }

    @Test
    void testIsFullSampleAvailable() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        assertEquals(0, estimator.getAccumulatedAccelerometerSamples());
        assertEquals(0, estimator.getAccumulatedGyroscopeSamples());
        assertEquals(0, estimator.getAccumulatedOrientationSamples());
        assertFalse(estimator.isAccelerometerSampleReceived());
        assertFalse(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.accumulatedAccelerometerSamples = 1;

        assertEquals(1, estimator.getAccumulatedAccelerometerSamples());
        assertEquals(0, estimator.getAccumulatedGyroscopeSamples());
        assertEquals(0, estimator.getAccumulatedOrientationSamples());
        assertTrue(estimator.isAccelerometerSampleReceived());
        assertFalse(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.accumulatedGyroscopeSamples = 2;

        assertEquals(1, estimator.getAccumulatedAccelerometerSamples());
        assertEquals(2, estimator.getAccumulatedGyroscopeSamples());
        assertEquals(0, estimator.getAccumulatedOrientationSamples());
        assertTrue(estimator.isAccelerometerSampleReceived());
        assertTrue(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.accumulatedOrientationSamples = 3;

        assertEquals(1, estimator.getAccumulatedAccelerometerSamples());
        assertEquals(2, estimator.getAccumulatedGyroscopeSamples());
        assertEquals(3, estimator.getAccumulatedOrientationSamples());
        assertTrue(estimator.isAccelerometerSampleReceived());
        assertTrue(estimator.isGyroscopeSampleReceived());
        assertTrue(estimator.isFullSampleAvailable());
    }

    @Test
    void testGetAccumulatedAccelerationSampleX() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleX(), 0.0);

        final var value = randomizer.nextDouble();
        estimator.accumulatedAccelerationSampleX = value;

        // check correctness
        assertEquals(value, estimator.getAccumulatedAccelerationSampleX(), 0.0);
    }

    @Test
    void testGetAccumulatedAccelerationSampleY() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleY(), 0.0);

        final var value = randomizer.nextDouble();
        estimator.accumulatedAccelerationSampleY = value;

        // check correctness
        assertEquals(value, estimator.getAccumulatedAccelerationSampleY(), 0.0);
    }

    @Test
    void testGetAccumulatedAccelerationSampleZ() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleZ(), 0.0);

        final var value = randomizer.nextDouble();
        estimator.accumulatedAccelerationSampleZ = value;

        // check correctness
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(), value, 0.0);
    }

    @Test
    void testGetAccumulatedAccelerationSample() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getAccumulatedAccelerationSample(), 0.0);

        // set new values
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();
        estimator.accumulatedAccelerationSampleX = valueX;
        estimator.accumulatedAccelerationSampleY = valueY;
        estimator.accumulatedAccelerationSampleZ = valueZ;

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, estimator.getAccumulatedAccelerationSample(),
                0.0);

        final var acceleration = new double[3];
        estimator.getAccumulatedAccelerationSample(acceleration);

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, acceleration, 0.0);

        // Force IllegalArgumentException
        final var wrong = new double[4];
        assertThrows(IllegalArgumentException.class, () -> estimator.getAccumulatedAccelerationSample(wrong));
    }

    @Test
    void testGetAccumulatedAngularSpeedSampleX() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleX(), 0.0);

        // set new value
        final var value = randomizer.nextDouble();
        estimator.accumulatedAngularSpeedSampleX = value;

        // check correctness
        assertEquals(value, estimator.getAccumulatedAngularSpeedSampleX(), 0.0);
    }

    @Test
    void testGetAccumulatedAngularSpeedSampleY() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleY(), 0.0);

        // set new value
        final var value = randomizer.nextDouble();
        estimator.accumulatedAngularSpeedSampleY = value;

        // check correctness
        assertEquals(value, estimator.getAccumulatedAngularSpeedSampleY(), 0.0);
    }

    @Test
    void testGetAccumulatedAngularSpeedSampleZ() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleZ(), 0.0);

        // set new value
        final var value = randomizer.nextDouble();
        estimator.accumulatedAngularSpeedSampleZ = value;

        // check correctness
        assertEquals(value, estimator.getAccumulatedAngularSpeedSampleZ(), 0.0);
    }

    @Test
    void testGetAccumulatedAngularSpeedSample() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var randomizer = new UniformRandomizer();

        assertArrayEquals(new double[]{0.0, 0.0, 0.0}, estimator.getAccumulatedAngularSpeedSample(), 0.0);

        // set new value
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();
        estimator.accumulatedAngularSpeedSampleX = valueX;
        estimator.accumulatedAngularSpeedSampleY = valueY;
        estimator.accumulatedAngularSpeedSampleZ = valueZ;

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, estimator.getAccumulatedAngularSpeedSample(),
                0.0);

        final var speed = new double[3];
        estimator.getAccumulatedAngularSpeedSample(speed);

        // check correctness
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, speed, 0.0);

        // Force IllegalArgumentException
        final var wrong = new double[4];
        assertThrows(IllegalArgumentException.class, () -> estimator.getAccumulatedAngularSpeedSample(wrong));
    }

    @Test
    void testUpdateAccelerometerSampleWithAccumulationDisabled() {
        final var estimator = new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(false);

        final var randomizer = new UniformRandomizer();

        var timestamp = System.currentTimeMillis();
        final var accelerationX = randomizer.nextFloat();
        final var accelerationY = randomizer.nextFloat();
        final var accelerationZ = randomizer.nextFloat();

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
        final var acceleration = new float[3];
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
        final var wrong = new float[4];
        final var finalTimestamp = timestamp;
        assertThrows(IllegalArgumentException.class, () -> estimator.updateAccelerometerSample(finalTimestamp, wrong));
    }

    @Test
    void testUpdateAccelerometerSampleWithAccumulationEnabled() {
        final var estimator = new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(true);

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(-1, estimator.getAccelerometerTimestampNanos());
        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleX(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleY(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAccelerationSampleZ(), 0.0);
        assertEquals(0, estimator.getAccumulatedAccelerometerSamples());
        assertFalse(estimator.isFullSampleAvailable());

        // update with several samples
        var timestamp = System.currentTimeMillis();
        float accelerationX;
        float accelerationY;
        float accelerationZ;
        var avgAccelerationX = 0.0;
        var avgAccelerationY = 0.0;
        var avgAccelerationZ = 0.0;
        for (var i = 0; i < TIMES; i++) {
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
    void testUpdateGyroscopeSampleWithAccumulationDisabled() {
        final var estimator = new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(false);

        final var randomizer = new UniformRandomizer();

        var timestamp = System.currentTimeMillis();
        final var angularSpeedX = randomizer.nextFloat();
        final var angularSpeedY = randomizer.nextFloat();
        final var angularSpeedZ = randomizer.nextFloat();

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
        final var angularSpeed = new float[3];
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
        final var wrong = new float[4];
        final var finalTimestamp = timestamp;
        assertThrows(IllegalArgumentException.class, () -> estimator.updateGyroscopeSample(finalTimestamp, wrong));
    }

    @Test
    void testUpdateGyroscopeSampleWithAccumulationEnabled() {
        final var estimator = new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(true);

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(-1, estimator.getGyroscopeTimestampNanos());
        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleX(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleY(), 0.0);
        assertEquals(0.0, estimator.getAccumulatedAngularSpeedSampleZ(), 0.0);
        assertEquals(0, estimator.getAccumulatedGyroscopeSamples());
        assertFalse(estimator.isFullSampleAvailable());

        // update with several samples
        var timestamp = System.currentTimeMillis();
        float angularSpeedX;
        float angularSpeedY;
        float angularSpeedZ;
        var avgAngularSpeedX = 0.0;
        var avgAngularSpeedY = 0.0;
        var avgAngularSpeedZ = 0.0;
        for (var i = 0; i < TIMES; i++) {
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
    void testUpdateOrientationSampleWithAccumulationDisabled() {
        final var estimator = new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(false);

        final var randomizer = new UniformRandomizer();

        final var timestamp = System.currentTimeMillis();
        final var orientationA = randomizer.nextDouble();
        final var orientationB = randomizer.nextDouble();
        final var orientationC = randomizer.nextDouble();
        final var orientationD = randomizer.nextDouble();
        final var orientation = new Quaternion(orientationA, orientationB, orientationC, orientationD);

        // check initial values
        assertEquals(-1, estimator.getOrientationTimestampNanos());
        var accumulatedOrientation = (Quaternion) estimator.getAccumulatedOrientation();
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
    void testUpdateOrientationSampleWithAccumulationEnabled() {
        final var estimator = new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(true);

        final var randomizer = new UniformRandomizer();

        // check initial values
        assertEquals(-1, estimator.getOrientationTimestampNanos());
        var accumulatedOrientation = (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(1.0, accumulatedOrientation.getA(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getB(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getC(), 0.0);
        assertEquals(0.0, accumulatedOrientation.getD(), 0.0);
        assertEquals(0, estimator.getAccumulatedOrientationSamples());
        assertFalse(estimator.isFullSampleAvailable());

        // update with several samples
        var timestamp = System.currentTimeMillis();
        double orientationA;
        double orientationB;
        double orientationC;
        double orientationD;
        var avgOrientationA = 0.0;
        var avgOrientationB = 0.0;
        var avgOrientationC = 0.0;
        var avgOrientationD = 0.0;
        var orientation = new Quaternion();
        double norm;
        for (var i = 0; i < TIMES; i++) {
            timestamp += 1000;
            orientationA = randomizer.nextDouble();
            orientationB = randomizer.nextDouble();
            orientationC = randomizer.nextDouble();
            orientationD = randomizer.nextDouble();
            norm = Math.sqrt(orientationA * orientationA + orientationB * orientationB + orientationC * orientationC
                    + orientationD * orientationD);
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
    void testGetMostRecentTimestampNanos() {
        final var estimator = new AbsoluteOrientationSlamEstimator();

        final var timestamp = System.currentTimeMillis();
        estimator.accelerometerTimestampNanos = timestamp;
        estimator.gyroscopeTimestampNanos = timestamp + 1000;

        assertEquals(timestamp + 1000, estimator.getMostRecentTimestampNanos());

        estimator.accelerometerTimestampNanos = timestamp + 2000;

        assertEquals(timestamp + 2000, estimator.getMostRecentTimestampNanos());

        estimator.orientationTimestampNanos = timestamp + 3000;

        assertEquals(timestamp + 3000, estimator.getMostRecentTimestampNanos());
    }

    @Test
    void testCorrectWithPositionMeasureCoordinatesAndCovariance() throws AlgebraException {
        for (var t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final var estimator = new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final var randomizer = new UniformRandomizer();

            final var positionX = randomizer.nextDouble();
            final var positionY = randomizer.nextDouble();
            final var positionZ = randomizer.nextDouble();

            final var positionCovariance = Matrix.identity(3, 3).
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
            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            final var orientation = new Quaternion();
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
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
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
            final var vx = VELOCITY_GAIN * positionX;
            final var vy = VELOCITY_GAIN * positionY;
            final var vz = VELOCITY_GAIN * positionZ;

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
            final var stateQuaternion = estimator.getStateQuaternion();
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
            final var wrong = new Matrix(4, 4);
            assertThrows(IllegalArgumentException.class,
                    () -> estimator.correctWithPositionMeasure(positionX, positionY, positionZ, wrong));
        }
    }

    @Test
    void testCorrectWithPositionMeasureArrayAndCovariance() throws AlgebraException {
        for (var t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final var estimator = new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final var randomizer = new UniformRandomizer();

            final var positionX = randomizer.nextDouble();
            final var positionY = randomizer.nextDouble();
            final var positionZ = randomizer.nextDouble();
            final var position = new double[]{positionX, positionY, positionZ};

            final var positionCovariance = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                    ABSOLUTE_ERROR);

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
            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            final var orientation = new Quaternion();
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
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
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
            final var vx = VELOCITY_GAIN * positionX;
            final var vy = VELOCITY_GAIN * positionY;
            final var vz = VELOCITY_GAIN * positionZ;

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
            var stateQuaternion = estimator.getStateQuaternion();
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
            final var wrongCov = new Matrix(4, 4);
            assertThrows(IllegalArgumentException.class,
                    () -> estimator.correctWithPositionMeasure(position, wrongCov));

            // wrong position length
            final var wrongPos = new double[4];
            assertThrows(IllegalArgumentException.class,
                    () -> estimator.correctWithPositionMeasure(wrongPos, positionCovariance));
        }
    }

    @Test
    void testCorrectWithPositionMeasurePointAndCovariance() throws AlgebraException {
        for (var t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final var estimator = new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final var randomizer = new UniformRandomizer();

            final var positionX = randomizer.nextDouble();
            final var positionY = randomizer.nextDouble();
            final var positionZ = randomizer.nextDouble();
            final var position = new InhomogeneousPoint3D(positionX, positionY, positionZ);

            final var positionCovariance = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                    ABSOLUTE_ERROR);

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
            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            final var orientation = new Quaternion();
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
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
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
            final var vx = VELOCITY_GAIN * positionX;
            final var vy = VELOCITY_GAIN * positionY;
            final var vz = VELOCITY_GAIN * positionZ;

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
            final var stateQuaternion = estimator.getStateQuaternion();
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
            final var wrong = new Matrix(4, 4);
            assertThrows(IllegalArgumentException.class, () -> estimator.correctWithPositionMeasure(position, wrong));
        }
    }

    @Test
    void testGetSetPositionCovarianceMatrix() throws AlgebraException {

        final var estimator = new AbsoluteOrientationSlamEstimator();

        // check initial value
        assertEquals(Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                KalmanFilter.DEFAULT_MEASUREMENT_NOISE_VARIANCE), estimator.getPositionCovarianceMatrix());

        // set new value
        final var positionCovariance = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);

        estimator.setPositionCovarianceMatrix(positionCovariance);

        // check correctness
        assertEquals(positionCovariance, estimator.getPositionCovarianceMatrix());

        // Force IllegalArgumentException
        final var wrong = new Matrix(4, 4);
        assertThrows(IllegalArgumentException.class, () -> estimator.setPositionCovarianceMatrix(wrong));
    }

    @Test
    void testCorrectWithPositionMeasureCoordinatesWithoutCovariance() throws AlgebraException {
        for (var t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final var estimator = new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final var randomizer = new UniformRandomizer();

            final var positionX = randomizer.nextDouble();
            final var positionY = randomizer.nextDouble();
            final var positionZ = randomizer.nextDouble();

            final var positionCovariance = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                    ABSOLUTE_ERROR);
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
            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            final var orientation = new Quaternion();
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
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
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
            final var vx = VELOCITY_GAIN * positionX;
            final var vy = VELOCITY_GAIN * positionY;
            final var vz = VELOCITY_GAIN * positionZ;

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
            var stateQuaternion = estimator.getStateQuaternion();
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
    void testCorrectWithPositionMeasureArrayWithoutCovariance() throws AlgebraException {
        for (var t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final var estimator = new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final var randomizer = new UniformRandomizer();

            final var positionX = randomizer.nextDouble();
            final var positionY = randomizer.nextDouble();
            final var positionZ = randomizer.nextDouble();
            final var position = new double[]{positionX, positionY, positionZ};

            final var positionCovariance = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                    ABSOLUTE_ERROR);
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
            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            final var orientation = new Quaternion();
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
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
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
            final var vx = VELOCITY_GAIN * positionX;
            final var vy = VELOCITY_GAIN * positionY;
            final var vz = VELOCITY_GAIN * positionZ;

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
            final var stateQuaternion = estimator.getStateQuaternion();
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
            final var wrong = new double[4];
            assertThrows(IllegalArgumentException.class, () -> estimator.correctWithPositionMeasure(wrong));
        }
    }

    @Test
    void testCorrectWithPositionMeasurePointWithoutCovariance() throws AlgebraException {
        for (var t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final var estimator = new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final var randomizer = new UniformRandomizer();

            final var positionX = randomizer.nextDouble();
            final var positionY = randomizer.nextDouble();
            final var positionZ = randomizer.nextDouble();
            final var position = new InhomogeneousPoint3D(positionX, positionY, positionZ);

            final var positionCovariance = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                    ABSOLUTE_ERROR);
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
            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(0, fullSampleReceived);
            assertEquals(0, fullSampleProcessed);
            assertEquals(0, correctWithPositionMeasure);
            assertEquals(0, correctedWithPositionMeasure);

            final var orientation = new Quaternion();
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
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
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
            final var vx = VELOCITY_GAIN * positionX;
            final var vy = VELOCITY_GAIN * positionY;
            final var vz = VELOCITY_GAIN * positionZ;

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
            final var stateQuaternion = estimator.getStateQuaternion();
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
            final var wrong = new Matrix(4, 4);
            assertThrows(IllegalArgumentException.class, () -> estimator.correctWithPositionMeasure(
                    positionX, positionY, positionZ, wrong));
        }
    }

    @Test
    void testCreateCalibrator() {
        assertInstanceOf(AbsoluteOrientationSlamCalibrator.class, AbsoluteOrientationSlamEstimator.createCalibrator());
    }

    @Test
    void testPredictionNoMotionWithNoise() {
        var numSuccess = 0;
        for (var t = 0; t < REPEAT_TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var gtPositionX = 0.0;
            final var gtPositionY = 0.0;
            final var gtPositionZ = 0.0;
            final var gtSpeedX = 0.0;
            final var gtSpeedY = 0.0;
            final var gtSpeedZ = 0.0;
            final var gtAccelerationX = 0.0f;
            final var gtAccelerationY = 0.0f;
            final var gtAccelerationZ = 0.0f;
            final var gtAngularSpeedX = 0.0f;
            final var gtAngularSpeedY = 0.0f;
            final var gtAngularSpeedZ = 0.0f;
            final var gtQuaternionA = 1.0f;
            final var gtQuaternionB = 0.0f;
            final var gtQuaternionC = 0.0f;
            final var gtQuaternionD = 0.0f;

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
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

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimator.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(),
                    estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    void testPredictionConstantSpeedWithNoise() {
        var numSuccess = 0;
        for (var t = 0; t < 5 * REPEAT_TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var gtPositionX = 0.0;
            final var gtPositionY = 0.0;
            final var gtPositionZ = 0.0;
            final var gtSpeedX = randomizer.nextDouble();
            final var gtSpeedY = randomizer.nextDouble();
            final var gtSpeedZ = randomizer.nextDouble();
            final var gtAccelerationX = 0.0f;
            final var gtAccelerationY = 0.0f;
            final var gtAccelerationZ = 0.0f;
            final var gtAngularSpeedX = 0.0f;
            final var gtAngularSpeedY = 0.0f;
            final var gtAngularSpeedZ = 0.0f;
            final var gtQuaternionA = 1.0f;
            final var gtQuaternionB = 0.0f;
            final var gtQuaternionC = 0.0f;
            final var gtQuaternionD = 0.0f;

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
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

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimator.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(), estimator.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    void testPredictionConstantAccelerationWithNoise() {
        var numSuccess = 0;
        for (var t = 0; t < REPEAT_TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var gtPositionX = 0.0;
            final var gtPositionY = 0.0;
            final var gtPositionZ = 0.0;
            final var gtSpeedX = 0.0;
            final var gtSpeedY = 0.0;
            final var gtSpeedZ = 0.0;
            final var gtAccelerationX = randomizer.nextFloat();
            final var gtAccelerationY = randomizer.nextFloat();
            final var gtAccelerationZ = randomizer.nextFloat();
            final var gtAngularSpeedX = 0.0f;
            final var gtAngularSpeedY = 0.0f;
            final var gtAngularSpeedZ = 0.0f;
            final var gtQuaternionA = 1.0f;
            final var gtQuaternionB = 0.0f;
            final var gtQuaternionC = 0.0f;
            final var gtQuaternionD = 0.0f;

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
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

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimator.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(), estimator.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    void testPredictionRotationOnlyWithNoise() {
        var numSuccess = 0;
        for (var t = 0; t < REPEAT_TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var gtPositionX = 0.0;
            final var gtPositionY = 0.0;
            final var gtPositionZ = 0.0;
            final var gtSpeedX = 0.0;
            final var gtSpeedY = 0.0;
            final var gtSpeedZ = 0.0;
            final var gtAccelerationX = randomizer.nextFloat();
            final var gtAccelerationY = randomizer.nextFloat();
            final var gtAccelerationZ = randomizer.nextFloat();
            final var gtAngularSpeedX = 0.0f;
            final var gtAngularSpeedY = 0.0f;
            final var gtAngularSpeedZ = 0.0f;
            final var gtQuaternionA = 1.0f;
            final var gtQuaternionB = 0.0f;
            final var gtQuaternionC = 0.0f;
            final var gtQuaternionD = 0.0f;

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
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

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimator.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(), estimator.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved = filteredPos.distanceTo(groundTruthPos)
                    < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    void testPredictionVariableAccelerationWithNoise() {
        var numSuccess = 0;
        for (var t = 0; t < REPEAT_TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var gtPositionX = 0.0;
            final var gtPositionY = 0.0;
            final var gtPositionZ = 0.0;
            final var gtSpeedX = randomizer.nextDouble();
            final var gtSpeedY = randomizer.nextDouble();
            final var gtSpeedZ = randomizer.nextDouble();
            final var period = N_PREDICTION_SAMPLES / 2;
            final var offsetAccelerationX = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var offsetAccelerationY = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var offsetAccelerationZ = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var amplitudeAccelerationX = randomizer.nextFloat();
            final var amplitudeAccelerationY = randomizer.nextFloat();
            final var amplitudeAccelerationZ = randomizer.nextFloat();
            var gtAccelerationX = (float) (amplitudeAccelerationX * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAccelerationX)));
            var gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAccelerationY)));
            var gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAccelerationZ)));
            final var gtAngularSpeedX = 0.0f;
            final var gtAngularSpeedY = 0.0f;
            final var gtAngularSpeedZ = 0.0f;
            final var gtQuaternionA = 1.0f;
            final var gtQuaternionB = 0.0f;
            final var gtQuaternionC = 0.0f;
            final var gtQuaternionD = 0.0f;

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                gtAccelerationX = (float) (amplitudeAccelerationX * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAccelerationX)));
                gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAccelerationY)));
                gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAccelerationZ)));

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

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimator.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(), estimator.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    void testPredictionVariableAngularSpeedWithNoise() {
        var numSuccess = 0;
        for (var t = 0; t < REPEAT_TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var gtPositionX = 0.0;
            final var gtPositionY = 0.0;
            final var gtPositionZ = 0.0;
            final var gtSpeedX = 0.0;
            final var gtSpeedY = 0.0;
            final var gtSpeedZ = 0.0;
            final var gtAccelerationX = 0.0f;
            final var gtAccelerationY = 0.0f;
            final var gtAccelerationZ = 0.0f;
            final var period = N_PREDICTION_SAMPLES / 2;
            final var offsetAngularSpeedX = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var offsetAngularSpeedY = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var offsetAngularSpeedZ = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var amplitudeAngularSpeedX = randomizer.nextFloat();
            final var amplitudeAngularSpeedY = randomizer.nextFloat();
            final var amplitudeAngularSpeedZ = randomizer.nextFloat();
            var gtAngularSpeedX = (float) (amplitudeAngularSpeedX * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAngularSpeedX)));
            var gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAngularSpeedY)));
            var gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAngularSpeedZ)));
            final var gtQuaternionA = 1.0f;
            final var gtQuaternionB = 0.0f;
            final var gtQuaternionC = 0.0f;
            final var gtQuaternionD = 0.0f;

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer( 0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                gtAngularSpeedX = (float) (amplitudeAngularSpeedX * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAngularSpeedX)));
                gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAngularSpeedY)));
                gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAngularSpeedZ)));

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

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimator.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(), estimator.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved || positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    void testPredictionWithNoiseAndInitialState() {
        var numSuccess = 0;
        for (var t = 0; t < REPEAT_TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var gtPositionX = randomizer.nextDouble();
            final var gtPositionY = randomizer.nextDouble();
            final var gtPositionZ = randomizer.nextDouble();
            final var gtSpeedX = randomizer.nextDouble();
            final var gtSpeedY = randomizer.nextDouble();
            final var gtSpeedZ = randomizer.nextDouble();
            final var gtAccelerationX = randomizer.nextFloat();
            final var gtAccelerationY = randomizer.nextFloat();
            final var gtAccelerationZ = randomizer.nextFloat();
            final var gtAngularSpeedX = randomizer.nextFloat();
            final var gtAngularSpeedY = randomizer.nextFloat();
            final var gtAngularSpeedZ = randomizer.nextFloat();
            final var gtQuaternion = new Quaternion(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());
            final var gtQuaternionA = (float) gtQuaternion.getA();
            final var gtQuaternionB = (float) gtQuaternion.getB();
            final var gtQuaternionC = (float) gtQuaternion.getC();
            final var gtQuaternionD = (float) gtQuaternion.getD();

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
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

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimator.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(), estimator.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved || positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    void testPredictionVariableAccelerationWithNoiseAndInitialState() {
        var numSuccess = 0;
        for (var t = 0; t < REPEAT_TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var gtPositionX = randomizer.nextDouble();
            final var gtPositionY = randomizer.nextDouble();
            final var gtPositionZ = randomizer.nextDouble();
            final var gtSpeedX = randomizer.nextDouble();
            final var gtSpeedY = randomizer.nextDouble();
            final var gtSpeedZ = randomizer.nextDouble();
            final var period = N_PREDICTION_SAMPLES / 2;
            final var offsetAccelerationX = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var offsetAccelerationY = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var offsetAccelerationZ = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var amplitudeAccelerationX = randomizer.nextFloat();
            final var amplitudeAccelerationY = randomizer.nextFloat();
            final var amplitudeAccelerationZ = randomizer.nextFloat();
            var gtAccelerationX = (float) (amplitudeAccelerationX * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAccelerationX)));
            var gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAccelerationY)));
            var gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAccelerationZ)));
            final var gtAngularSpeedX = randomizer.nextFloat();
            final var gtAngularSpeedY = randomizer.nextFloat();
            final var gtAngularSpeedZ = randomizer.nextFloat();
            final var gtQuaternion = new Quaternion(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());
            final var gtQuaternionA = (float) gtQuaternion.getA();
            final var gtQuaternionB = (float) gtQuaternion.getB();
            final var gtQuaternionC = (float) gtQuaternion.getC();
            final var gtQuaternionD = (float) gtQuaternion.getD();

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                gtAccelerationX = (float) (amplitudeAccelerationX * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAccelerationX)));
                gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAccelerationY)));
                gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAccelerationZ)));

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

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimator.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(), estimator.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved || positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    void testPredictionVariableAngularSpeedWithNoiseAndInitialState() {
        var numSuccess = 0;
        for (var t = 0; t < REPEAT_TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var gtPositionX = randomizer.nextDouble();
            final var gtPositionY = randomizer.nextDouble();
            final var gtPositionZ = randomizer.nextDouble();
            final var gtSpeedX = randomizer.nextDouble();
            final var gtSpeedY = randomizer.nextDouble();
            final var gtSpeedZ = randomizer.nextDouble();
            final var gtAccelerationX = randomizer.nextFloat();
            final var gtAccelerationY = randomizer.nextFloat();
            final var gtAccelerationZ = randomizer.nextFloat();
            final var period = N_PREDICTION_SAMPLES / 2;
            final var offsetAngularSpeedX = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var offsetAngularSpeedY = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var offsetAngularSpeedZ = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var amplitudeAngularSpeedX = randomizer.nextFloat();
            final var amplitudeAngularSpeedY = randomizer.nextFloat();
            final var amplitudeAngularSpeedZ = randomizer.nextFloat();
            var gtAngularSpeedX = (float) (amplitudeAngularSpeedX * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAngularSpeedX)));
            var gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAngularSpeedY)));
            var gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAngularSpeedZ)));
            final var gtQuaternion = new Quaternion(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());
            final var gtQuaternionA = (float) gtQuaternion.getA();
            final var gtQuaternionB = (float) gtQuaternion.getB();
            final var gtQuaternionC = (float) gtQuaternion.getC();
            final var gtQuaternionD = (float) gtQuaternion.getD();

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                gtAngularSpeedX = (float) (amplitudeAngularSpeedX * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAngularSpeedX)));
                gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAngularSpeedY)));
                gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAngularSpeedZ)));

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

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimator.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(), estimator.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved || positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    void testPredictionRandomAccelerationAndAngularSpeedWithNoise() {
        var numSuccess = 0;
        for (var t = 0; t < REPEAT_TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var gtPositionX = randomizer.nextDouble();
            final var gtPositionY = randomizer.nextDouble();
            final var gtPositionZ = randomizer.nextDouble();
            final var gtSpeedX = randomizer.nextDouble();
            final var gtSpeedY = randomizer.nextDouble();
            final var gtSpeedZ = randomizer.nextDouble();
            var gtAccelerationX = randomizer.nextFloat();
            var gtAccelerationY = randomizer.nextFloat();
            var gtAccelerationZ = randomizer.nextFloat();
            var gtAngularSpeedX = randomizer.nextFloat();
            var gtAngularSpeedY = randomizer.nextFloat();
            var gtAngularSpeedZ = randomizer.nextFloat();
            final var gtQuaternion = new Quaternion(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());
            final var gtQuaternionA = (float) gtQuaternion.getA();
            var gtQuaternionB = (float) gtQuaternion.getB();
            var gtQuaternionC = (float) gtQuaternion.getC();
            var gtQuaternionD = (float) gtQuaternion.getD();

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            // set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);

            String msg;
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
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

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimator.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), estimator.getStatePositionY(), estimator.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertFalse(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);
    }

    @Test
    void testPredictionNoMotionWithNoiseAndCalibration() {
        var numSuccess = 0;
        for (var t = 0; t < 5 * REPEAT_TIMES; t++) {
            final var offsetRandomizer = new UniformRandomizer();
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var accelerationOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var angularOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                    angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final var calibration = calibrator.getCalibrationData();

            final var randomizer = new UniformRandomizer();

            final var gtPositionX = 0.0;
            final var gtPositionY = 0.0;
            final var gtPositionZ = 0.0;
            final var gtSpeedX = 0.0;
            final var gtSpeedY = 0.0;
            final var gtSpeedZ = 0.0;
            final var gtAccelerationX = 0.0f;
            final var gtAccelerationY = 0.0f;
            final var gtAccelerationZ = 0.0f;
            final var gtAngularSpeedX = 0.0f;
            final var gtAngularSpeedY = 0.0f;
            final var gtAngularSpeedZ = 0.0f;
            final var gtQuaternionA = 1.0f;
            final var gtQuaternionB = 0.0f;
            final var gtQuaternionC = 0.0f;
            final var gtQuaternionD = 0.0f;

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var estimatorWithCalibration = new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0, ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastAccelerationWithOffsetX = 0.0;
            var lastAccelerationWithOffsetY = 0.0;
            var lastAccelerationWithOffsetZ = 0.0;
            var lastAngularSpeedWithOffsetX = 0.0;
            var lastAngularSpeedWithOffsetY = 0.0;
            var lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX;
            double deltaAccelerationWithOffsetY;
            double deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX;
            double deltaAngularSpeedWithOffsetY;
            double deltaAngularSpeedWithOffsetZ;

            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
            final var uWithOffset = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            final var xWithOffset = Arrays.copyOf(x, x.length);

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
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
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
                    StatePredictor.predictWithRotationAdjustment(xWithOffset, uWithOffset, DELTA_SECONDS, xWithOffset);

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

            msg = "Filtered with calibrator - positionX: " + estimatorWithCalibration.getStatePositionX()
                    + ", positionY: " + estimatorWithCalibration.getStatePositionY()
                    + ", positionZ: " + estimatorWithCalibration.getStatePositionZ()
                    + ", velocityX: " + estimatorWithCalibration.getStateVelocityX()
                    + ", velocityY: " + estimatorWithCalibration.getStateVelocityY()
                    + ", velocityZ: " + estimatorWithCalibration.getStateVelocityZ()
                    + ", accelerationX: " + estimatorWithCalibration.getStateAccelerationX()
                    + ", accelerationY: " + estimatorWithCalibration.getStateAccelerationY()
                    + ", accelerationZ: " + estimatorWithCalibration.getStateAccelerationZ()
                    + ", quaternionA: " + estimatorWithCalibration.getStateQuaternionA()
                    + ", quaternionB: " + estimatorWithCalibration.getStateQuaternionB()
                    + ", quaternionC: " + estimatorWithCalibration.getStateQuaternionC()
                    + ", quaternionD: " + estimatorWithCalibration.getStateQuaternionD()
                    + ", angularSpeedX: " + estimatorWithCalibration.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimatorWithCalibration.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimatorWithCalibration.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimatorWithCalibration.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(),
                    estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
    }

    @Test
    void testPredictionConstantSpeedWithNoiseAndCalibration() {
        var numSuccess = 0;
        for (var t = 0; t < 5 * REPEAT_TIMES; t++) {
            final var offsetRandomizer = new UniformRandomizer();
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var accelerationOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var angularOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                    angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final var calibration = calibrator.getCalibrationData();

            final var randomizer = new UniformRandomizer();

            final var gtPositionX = 0.0;
            final var gtPositionY = 0.0;
            final var gtPositionZ = 0.0;
            final var gtSpeedX = randomizer.nextDouble();
            final var gtSpeedY = randomizer.nextDouble();
            final var gtSpeedZ = randomizer.nextDouble();
            final var gtAccelerationX = 0.0f;
            final var gtAccelerationY = 0.0f;
            final var gtAccelerationZ = 0.0f;
            final var gtAngularSpeedX = 0.0f;
            final var gtAngularSpeedY = 0.0f;
            final var gtAngularSpeedZ = 0.0f;
            final var gtQuaternionA = 1.0f;
            final var gtQuaternionB = 0.0f;
            final var gtQuaternionC = 0.0f;
            final var gtQuaternionD = 0.0f;

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var estimatorWithCalibration = new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastAccelerationWithOffsetX = 0.0;
            var lastAccelerationWithOffsetY = 0.0;
            var lastAccelerationWithOffsetZ = 0.0;
            var lastAngularSpeedWithOffsetX = 0.0;
            var lastAngularSpeedWithOffsetY = 0.0;
            var lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX;
            double deltaAccelerationWithOffsetY;
            double deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX;
            double deltaAngularSpeedWithOffsetY;
            double deltaAngularSpeedWithOffsetZ;

            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
            final var uWithOffset = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            final var xWithOffset = Arrays.copyOf(x, x.length);

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
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
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
                    StatePredictor.predictWithRotationAdjustment(xWithOffset, uWithOffset, DELTA_SECONDS, xWithOffset);

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

            msg = "Filtered with calibrator - positionX: " + estimatorWithCalibration.getStatePositionX()
                    + ", positionY: " + estimatorWithCalibration.getStatePositionY()
                    + ", positionZ: " + estimatorWithCalibration.getStatePositionZ()
                    + ", velocityX: " + estimatorWithCalibration.getStateVelocityX()
                    + ", velocityY: " + estimatorWithCalibration.getStateVelocityY()
                    + ", velocityZ: " + estimatorWithCalibration.getStateVelocityZ()
                    + ", accelerationX: " + estimatorWithCalibration.getStateAccelerationX()
                    + ", accelerationY: " + estimatorWithCalibration.getStateAccelerationY()
                    + ", accelerationZ: " + estimatorWithCalibration.getStateAccelerationZ()
                    + ", quaternionA: " + estimatorWithCalibration.getStateQuaternionA()
                    + ", quaternionB: " + estimatorWithCalibration.getStateQuaternionB()
                    + ", quaternionC: " + estimatorWithCalibration.getStateQuaternionC()
                    + ", quaternionD: " + estimatorWithCalibration.getStateQuaternionD()
                    + ", angularSpeedX: " + estimatorWithCalibration.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimatorWithCalibration.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimatorWithCalibration.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimatorWithCalibration.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(),
                    estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
    }

    @Test
    void testPredictionConstantAccelerationWithNoiseAndCalibration() {
        var numSuccess = 0;
        for (var t = 0; t < 2 * REPEAT_TIMES; t++) {
            final var offsetRandomizer = new UniformRandomizer();
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var accelerationOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var angularOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                    angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final var calibration = calibrator.getCalibrationData();

            final var randomizer = new UniformRandomizer();

            final var gtPositionX = 0.0;
            final var gtPositionY = 0.0;
            final var gtPositionZ = 0.0;
            final var gtSpeedX = 0.0;
            final var gtSpeedY = 0.0;
            final var gtSpeedZ = 0.0;
            final var gtAccelerationX = randomizer.nextFloat();
            final var gtAccelerationY = randomizer.nextFloat();
            final var gtAccelerationZ = randomizer.nextFloat();
            final var gtAngularSpeedX = 0.0f;
            final var gtAngularSpeedY = 0.0f;
            final var gtAngularSpeedZ = 0.0f;
            final var gtQuaternionA = 1.0f;
            final var gtQuaternionB = 0.0f;
            final var gtQuaternionC = 0.0f;
            final var gtQuaternionD = 0.0f;

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var estimatorWithCalibration = new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastAccelerationWithOffsetX = 0.0;
            var lastAccelerationWithOffsetY = 0.0;
            var lastAccelerationWithOffsetZ = 0.0;
            var lastAngularSpeedWithOffsetX = 0.0;
            var lastAngularSpeedWithOffsetY = 0.0;
            var lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX;
            double deltaAccelerationWithOffsetY;
            double deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX;
            double deltaAngularSpeedWithOffsetY;
            double deltaAngularSpeedWithOffsetZ;

            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
            final var uWithOffset = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            final var xWithOffset = Arrays.copyOf(x, x.length);

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
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
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
                    StatePredictor.predictWithRotationAdjustment(xWithOffset, uWithOffset, DELTA_SECONDS, xWithOffset);

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

            msg = "Filtered with calibrator - positionX: " + estimatorWithCalibration.getStatePositionX()
                    + ", positionY: " + estimatorWithCalibration.getStatePositionY()
                    + ", positionZ: " + estimatorWithCalibration.getStatePositionZ()
                    + ", velocityX: " + estimatorWithCalibration.getStateVelocityX()
                    + ", velocityY: " + estimatorWithCalibration.getStateVelocityY()
                    + ", velocityZ: " + estimatorWithCalibration.getStateVelocityZ()
                    + ", accelerationX: " + estimatorWithCalibration.getStateAccelerationX()
                    + ", accelerationY: " + estimatorWithCalibration.getStateAccelerationY()
                    + ", accelerationZ: " + estimatorWithCalibration.getStateAccelerationZ()
                    + ", quaternionA: " + estimatorWithCalibration.getStateQuaternionA()
                    + ", quaternionB: " + estimatorWithCalibration.getStateQuaternionB()
                    + ", quaternionC: " + estimatorWithCalibration.getStateQuaternionC()
                    + ", quaternionD: " + estimatorWithCalibration.getStateQuaternionD()
                    + ", angularSpeedX: " + estimatorWithCalibration.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimatorWithCalibration.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimatorWithCalibration.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimatorWithCalibration.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
                break;
            }
        }

        assertTrue(numSuccess > 0);
    }

    @Test
    void testPredictionRotationOnlyWithNoiseAndCalibration() {
        var numSuccess = 0;
        for (var t = 0; t < 5 * REPEAT_TIMES; t++) {
            final var offsetRandomizer = new UniformRandomizer();
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var accelerationOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var angularOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                    angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final var calibration = calibrator.getCalibrationData();

            final var randomizer = new UniformRandomizer();

            final var gtPositionX = 0.0;
            final var gtPositionY = 0.0;
            final var gtPositionZ = 0.0;
            final var gtSpeedX = 0.0;
            final var gtSpeedY = 0.0;
            final var gtSpeedZ = 0.0;
            final var gtAccelerationX = randomizer.nextFloat();
            final var gtAccelerationY = randomizer.nextFloat();
            final var gtAccelerationZ = randomizer.nextFloat();
            final var gtAngularSpeedX = 0.0f;
            final var gtAngularSpeedY = 0.0f;
            final var gtAngularSpeedZ = 0.0f;
            final var gtQuaternionA = 1.0f;
            final var gtQuaternionB = 0.0f;
            final var gtQuaternionC = 0.0f;
            final var gtQuaternionD = 0.0f;

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var estimatorWithCalibration = new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastAccelerationWithOffsetX = 0.0;
            var lastAccelerationWithOffsetY = 0.0;
            var lastAccelerationWithOffsetZ = 0.0;
            var lastAngularSpeedWithOffsetX = 0.0;
            var lastAngularSpeedWithOffsetY = 0.0;
            var lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX;
            double deltaAccelerationWithOffsetY;
            double deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX;
            double deltaAngularSpeedWithOffsetY;
            double deltaAngularSpeedWithOffsetZ;
            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
            final var uWithOffset = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            final var xWithOffset = Arrays.copyOf(x, x.length);

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
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
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
                    StatePredictor.predictWithRotationAdjustment(xWithOffset, uWithOffset, DELTA_SECONDS, xWithOffset);

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

            msg = "Filtered with calibrator - positionX: " + estimatorWithCalibration.getStatePositionX()
                    + ", positionY: " + estimatorWithCalibration.getStatePositionY()
                    + ", positionZ: " + estimatorWithCalibration.getStatePositionZ()
                    + ", velocityX: " + estimatorWithCalibration.getStateVelocityX()
                    + ", velocityY: " + estimatorWithCalibration.getStateVelocityY()
                    + ", velocityZ: " + estimatorWithCalibration.getStateVelocityZ()
                    + ", accelerationX: " + estimatorWithCalibration.getStateAccelerationX()
                    + ", accelerationY: " + estimatorWithCalibration.getStateAccelerationY()
                    + ", accelerationZ: " + estimatorWithCalibration.getStateAccelerationZ()
                    + ", quaternionA: " + estimatorWithCalibration.getStateQuaternionA()
                    + ", quaternionB: " + estimatorWithCalibration.getStateQuaternionB()
                    + ", quaternionC: " + estimatorWithCalibration.getStateQuaternionC()
                    + ", quaternionD: " + estimatorWithCalibration.getStateQuaternionD()
                    + ", angularSpeedX: " + estimatorWithCalibration.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimatorWithCalibration.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimatorWithCalibration.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimatorWithCalibration.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
    }

    @Test
    void testPredictionVariableAccelerationWithNoiseAndCalibration() {
        var numSuccess = 0;
        for (var t = 0; t < 10 * REPEAT_TIMES; t++) {
            final var offsetRandomizer = new UniformRandomizer();
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var accelerationOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var angularOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                    angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final var calibration = calibrator.getCalibrationData();

            final var randomizer = new UniformRandomizer();

            final var gtPositionX = 0.0;
            final var gtPositionY = 0.0;
            final var gtPositionZ = 0.0;
            final var gtSpeedX = randomizer.nextDouble();
            final var gtSpeedY = randomizer.nextDouble();
            final var gtSpeedZ = randomizer.nextDouble();
            final var period = N_PREDICTION_SAMPLES / 2;
            final var offsetAccelerationX = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var offsetAccelerationY = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var offsetAccelerationZ = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var amplitudeAccelerationX = randomizer.nextFloat();
            final var amplitudeAccelerationY = randomizer.nextFloat();
            final var amplitudeAccelerationZ = randomizer.nextFloat();
            float gtAccelerationX = (float) (amplitudeAccelerationX * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAccelerationX)));
            float gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAccelerationY)));
            float gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAccelerationZ)));
            final var gtAngularSpeedX = 0.0f;
            final var gtAngularSpeedY = 0.0f;
            final var gtAngularSpeedZ = 0.0f;
            final var gtQuaternionA = 1.0f;
            final var gtQuaternionB = 0.0f;
            final var gtQuaternionC = 0.0f;
            final var gtQuaternionD = 0.0f;

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var estimatorWithCalibration = new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastAccelerationWithOffsetX = 0.0;
            var lastAccelerationWithOffsetY = 0.0;
            var lastAccelerationWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX;
            double deltaAccelerationWithOffsetY;
            double deltaAccelerationWithOffsetZ;

            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
            final var uWithOffset = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            final var xWithOffset = Arrays.copyOf(x, x.length);

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
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
                noiseAccelerationX = accelerationRandomizer.nextFloat();
                noiseAccelerationY = accelerationRandomizer.nextFloat();
                noiseAccelerationZ = accelerationRandomizer.nextFloat();

                gtAccelerationX = (float) (amplitudeAccelerationX * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAccelerationX)));
                gtAccelerationY = (float) (amplitudeAccelerationY * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAccelerationY)));
                gtAccelerationZ = (float) (amplitudeAccelerationZ * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAccelerationZ)));

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
                    StatePredictor.predictWithRotationAdjustment(xWithOffset, uWithOffset, DELTA_SECONDS, xWithOffset);

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

            msg = "Filtered with calibrator - positionX: " + estimatorWithCalibration.getStatePositionX()
                    + ", positionY: " + estimatorWithCalibration.getStatePositionY()
                    + ", positionZ: " + estimatorWithCalibration.getStatePositionZ()
                    + ", velocityX: " + estimatorWithCalibration.getStateVelocityX()
                    + ", velocityY: " + estimatorWithCalibration.getStateVelocityY()
                    + ", velocityZ: " + estimatorWithCalibration.getStateVelocityZ()
                    + ", accelerationX: " + estimatorWithCalibration.getStateAccelerationX()
                    + ", accelerationY: " + estimatorWithCalibration.getStateAccelerationY()
                    + ", accelerationZ: " + estimatorWithCalibration.getStateAccelerationZ()
                    + ", quaternionA: " + estimatorWithCalibration.getStateQuaternionA()
                    + ", quaternionB: " + estimatorWithCalibration.getStateQuaternionB()
                    + ", quaternionC: " + estimatorWithCalibration.getStateQuaternionC()
                    + ", quaternionD: " + estimatorWithCalibration.getStateQuaternionD()
                    + ", angularSpeedX: " + estimatorWithCalibration.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimatorWithCalibration.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimatorWithCalibration.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimatorWithCalibration.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated)
                    < predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > 10 * REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
    }

    @Test
    void testPredictionVariableAngularSpeedWithNoiseAndCalibration() {
        var numSuccess = 0;
        for (var t = 0; t < REPEAT_TIMES; t++) {
            final var offsetRandomizer = new UniformRandomizer();
            final var noiseRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            final var accelerationOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var angularOffsetX = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetY = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            final var angularOffsetZ = offsetRandomizer.nextFloat(MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            final var calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ,
                    angularOffsetX, angularOffsetY, angularOffsetZ, noiseRandomizer);
            final var calibration = calibrator.getCalibrationData();

            final var randomizer = new UniformRandomizer();

            final var gtPositionX = 0.0;
            final var gtPositionY = 0.0;
            final var gtPositionZ = 0.0;
            final var gtSpeedX = 0.0;
            final var gtSpeedY = 0.0;
            final var gtSpeedZ = 0.0;
            final var gtAccelerationX = 0.0f;
            final var gtAccelerationY = 0.0f;
            final var gtAccelerationZ = 0.0f;
            final var period = N_PREDICTION_SAMPLES / 2;
            final var offsetAngularSpeedX = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var offsetAngularSpeedY = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var offsetAngularSpeedZ = randomizer.nextInt(0, N_PREDICTION_SAMPLES);
            final var amplitudeAngularSpeedX = randomizer.nextFloat();
            final var amplitudeAngularSpeedY = randomizer.nextFloat();
            final var amplitudeAngularSpeedZ = randomizer.nextFloat();
            var gtAngularSpeedX = (float) (amplitudeAngularSpeedX * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAngularSpeedX)));
            var gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAngularSpeedY)));
            var gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(2.0 * Math.PI / (double) period
                    * (double) (-offsetAngularSpeedZ)));
            final var gtQuaternionA = 1.0f;
            final var gtQuaternionB = 0.0f;
            final var gtQuaternionC = 0.0f;
            final var gtQuaternionD = 0.0f;

            final var estimator = new AbsoluteOrientationSlamEstimator();

            final var estimatorWithCalibration = new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final var accelerationRandomizer = new GaussianRandomizer(0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            final var angularSpeedRandomizer = new GaussianRandomizer(0.0,
                    ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final var orientationRandomizer = new GaussianRandomizer(0.0, ORIENTATION_NOISE_STANDARD_DEVIATION);

            var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
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
            var lastAccelerationX = 0.0;
            var lastAccelerationY = 0.0;
            var lastAccelerationZ = 0.0;
            var lastAngularSpeedX = 0.0;
            var lastAngularSpeedY = 0.0;
            var lastAngularSpeedZ = 0.0;
            double deltaAccelerationX;
            double deltaAccelerationY;
            double deltaAccelerationZ;
            double deltaAngularSpeedX;
            double deltaAngularSpeedY;
            double deltaAngularSpeedZ;
            final var lastOrientation = new Quaternion();
            final var deltaOrientation = new Quaternion();
            var lastAccelerationWithOffsetX = 0.0;
            var lastAccelerationWithOffsetY = 0.0;
            var lastAccelerationWithOffsetZ = 0.0;
            var lastAngularSpeedWithOffsetX = 0.0;
            var lastAngularSpeedWithOffsetY = 0.0;
            var lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX;
            double deltaAccelerationWithOffsetY;
            double deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX;
            double deltaAngularSpeedWithOffsetY;
            double deltaAngularSpeedWithOffsetZ;

            var lastGtAccelerationX = 0.0;
            var lastGtAccelerationY = 0.0;
            var lastGtAccelerationZ = 0.0;
            var lastGtAngularSpeedX = 0.0;
            var lastGtAngularSpeedY = 0.0;
            var lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX;
            double deltaGtAccelerationY;
            double deltaGtAccelerationZ;
            double deltaGtAngularSpeedX;
            double deltaGtAngularSpeedY;
            double deltaGtAngularSpeedZ;
            final var lastGtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var deltaGtOrientation = new Quaternion();
            final var orientation = new Quaternion();
            final var gtOrientation = new Quaternion(gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final var x = new double[16];
            final var u = new double[13];
            final var uWithOffset = new double[13];
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
            final var gtX = Arrays.copyOf(x, x.length);
            final var gtU = new double[13];

            final var xWithOffset = Arrays.copyOf(x, x.length);

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
            for (var i = 0; i < N_PREDICTION_SAMPLES; i++) {
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

                gtAngularSpeedX = (float) (amplitudeAngularSpeedX * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAngularSpeedX)));
                gtAngularSpeedY = (float) (amplitudeAngularSpeedY * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAngularSpeedY)));
                gtAngularSpeedZ = (float) (amplitudeAngularSpeedZ * Math.sin(2.0 * Math.PI / (double) period
                        * (double) (i - offsetAngularSpeedZ)));

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
                    StatePredictor.predictWithRotationAdjustment(xWithOffset, uWithOffset, DELTA_SECONDS, xWithOffset);

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

            msg = "Filtered with calibrator - positionX: " + estimatorWithCalibration.getStatePositionX()
                    + ", positionY: " + estimatorWithCalibration.getStatePositionY()
                    + ", positionZ: " + estimatorWithCalibration.getStatePositionZ()
                    + ", velocityX: " + estimatorWithCalibration.getStateVelocityX()
                    + ", velocityY: " + estimatorWithCalibration.getStateVelocityY()
                    + ", velocityZ: " + estimatorWithCalibration.getStateVelocityZ()
                    + ", accelerationX: " + estimatorWithCalibration.getStateAccelerationX()
                    + ", accelerationY: " + estimatorWithCalibration.getStateAccelerationY()
                    + ", accelerationZ: " + estimatorWithCalibration.getStateAccelerationZ()
                    + ", quaternionA: " + estimatorWithCalibration.getStateQuaternionA()
                    + ", quaternionB: " + estimatorWithCalibration.getStateQuaternionB()
                    + ", quaternionC: " + estimatorWithCalibration.getStateQuaternionC()
                    + ", quaternionD: " + estimatorWithCalibration.getStateQuaternionD()
                    + ", angularSpeedX: " + estimatorWithCalibration.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimatorWithCalibration.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimatorWithCalibration.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Filtered - positionX: " + estimator.getStatePositionX()
                    + ", positionY: " + estimator.getStatePositionY()
                    + ", positionZ: " + estimator.getStatePositionZ()
                    + ", velocityX: " + estimator.getStateVelocityX()
                    + ", velocityY: " + estimator.getStateVelocityY()
                    + ", velocityZ: " + estimator.getStateVelocityZ()
                    + ", accelerationX: " + estimator.getStateAccelerationX()
                    + ", accelerationY: " + estimator.getStateAccelerationY()
                    + ", accelerationZ: " + estimator.getStateAccelerationZ()
                    + ", quaternionA: " + estimator.getStateQuaternionA()
                    + ", quaternionB: " + estimator.getStateQuaternionB()
                    + ", quaternionC: " + estimator.getStateQuaternionC()
                    + ", quaternionD: " + estimator.getStateQuaternionD()
                    + ", angularSpeedX: " + estimator.getStateAngularSpeedX()
                    + ", angularSpeedY: " + estimator.getStateAngularSpeedY()
                    + ", angularSpeedZ: " + estimator.getStateAngularSpeedZ();
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Prediction - positionX: " + x[0]
                    + ", positionY: " + x[1]
                    + ", positionZ: " + x[2]
                    + ", velocityX: " + x[7]
                    + ", velocityY: " + x[8]
                    + ", velocityZ: " + x[9]
                    + ", accelerationX: " + x[10]
                    + ", accelerationY: " + x[11]
                    + ", accelerationZ: " + x[12]
                    + ", quaternionA: " + x[3]
                    + ", quaternionB: " + x[4]
                    + ", quaternionC: " + x[5]
                    + ", quaternionD: " + x[6]
                    + ", angularSpeedX: " + x[13]
                    + ", angularSpeedY: " + x[14]
                    + ", angularSpeedZ: " + x[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            msg = "Ground truth - positionX: " + gtX[0]
                    + ", positionY: " + gtX[1]
                    + ", positionZ: " + gtX[2]
                    + ", velocityX: " + gtX[7]
                    + ", velocityY: " + gtX[8]
                    + ", velocityZ: " + gtX[9]
                    + ", accelerationX: " + gtX[10]
                    + ", accelerationY: " + gtX[11]
                    + ", accelerationZ: " + gtX[12]
                    + ", quaternionA: " + gtX[3]
                    + ", quaternionB: " + gtX[4]
                    + ", quaternionC: " + gtX[5]
                    + ", quaternionD: " + gtX[6]
                    + ", angularSpeedX: " + gtX[13]
                    + ", angularSpeedY: " + gtX[14]
                    + ", angularSpeedZ: " + gtX[15];
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final var randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), randomizer.nextDouble());

            final var filteredQuaternion = estimatorWithCalibration.getStateQuaternion();
            final var predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            final var groundTruthQuaternion = new Quaternion(gtX[3], gtX[4], gtX[5], gtX[6]);

            final var filteredRotated = filteredQuaternion.rotate(randomPoint);
            final var predictedRotated = predictedQuaternion.rotate(randomPoint);
            final var groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            final var rotationImproved = filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final var filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final var predictedPos = new InhomogeneousPoint3D(x[0], x[1], x[2]);

            final var groundTruthPos = new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final var positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved || positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
    }

    @Test
    void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var estimator1 = new AbsoluteOrientationSlamEstimator();

        // set new values
        final var randomizer = new UniformRandomizer();

        final var data = new AbsoluteOrientationSlamCalibrationData();
        estimator1.setCalibrationData(data);
        final var statePositionX = randomizer.nextDouble();
        estimator1.statePositionX = statePositionX;
        final var statePositionY = randomizer.nextDouble();
        estimator1.statePositionY = statePositionY;
        final var statePositionZ = randomizer.nextDouble();
        estimator1.statePositionZ = statePositionZ;
        final var stateVelocityX = randomizer.nextDouble();
        estimator1.stateVelocityX = stateVelocityX;
        final var stateVelocityY = randomizer.nextDouble();
        estimator1.stateVelocityY = stateVelocityY;
        final var stateVelocityZ = randomizer.nextDouble();
        estimator1.stateVelocityZ = stateVelocityZ;
        final var stateAccelerationX = randomizer.nextDouble();
        estimator1.stateAccelerationX = stateAccelerationX;
        final var stateAccelerationY = randomizer.nextDouble();
        estimator1.stateAccelerationY = stateAccelerationY;
        final var stateAccelerationZ = randomizer.nextDouble();
        estimator1.stateAccelerationZ = stateAccelerationZ;
        final var stateQuaternionA = randomizer.nextDouble();
        estimator1.stateQuaternionA = stateQuaternionA;
        final var stateQuaternionB = randomizer.nextDouble();
        estimator1.stateQuaternionB = stateQuaternionB;
        final var stateQuaternionC = randomizer.nextDouble();
        estimator1.stateQuaternionC = stateQuaternionC;
        final var stateQuaternionD = randomizer.nextDouble();
        estimator1.stateQuaternionD = stateQuaternionD;
        final var stateAngularSpeedX = randomizer.nextDouble();
        estimator1.stateAngularSpeedX = stateAngularSpeedX;
        final var stateAngularSpeedY = randomizer.nextDouble();
        estimator1.stateAngularSpeedY = stateAngularSpeedY;
        final var stateAngularSpeedZ = randomizer.nextDouble();
        estimator1.stateAngularSpeedZ = stateAngularSpeedZ;
        estimator1.error = true;
        estimator1.setAccumulationEnabled(false);
        estimator1.accelerometerTimestampNanos = 1000;
        estimator1.gyroscopeTimestampNanos = 2000;
        estimator1.orientationTimestampNanos = 3000;
        estimator1.accumulatedAccelerometerSamples = 10;
        estimator1.accumulatedGyroscopeSamples = 20;
        final var q = new Quaternion(randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble());
        estimator1.accumulatedOrientation = q;
        final var accumulatedAccelerationSampleX = randomizer.nextDouble();
        estimator1.accumulatedAccelerationSampleX = accumulatedAccelerationSampleX;
        final var accumulatedAccelerationSampleY = randomizer.nextDouble();
        estimator1.accumulatedAccelerationSampleY = accumulatedAccelerationSampleY;
        final var accumulatedAccelerationSampleZ = randomizer.nextDouble();
        estimator1.accumulatedAccelerationSampleZ = accumulatedAccelerationSampleZ;
        final var accumulatedAngularSpeedSampleX = randomizer.nextDouble();
        estimator1.accumulatedAngularSpeedSampleX = accumulatedAngularSpeedSampleX;
        final var accumulatedAngularSpeedSampleY = randomizer.nextDouble();
        estimator1.accumulatedAngularSpeedSampleY = accumulatedAngularSpeedSampleY;
        final var accumulatedAngularSpeedSampleZ = randomizer.nextDouble();
        estimator1.accumulatedAngularSpeedSampleZ = accumulatedAngularSpeedSampleZ;
        final var positionCovariance = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);
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
        final var accumulatedOrientation1 = (Quaternion) estimator1.getAccumulatedOrientation();
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

        final var bytes = SerializationHelper.serialize(estimator1);
        final var estimator2 = SerializationHelper.<AbsoluteOrientationSlamEstimator>deserialize(bytes);

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
        assertEquals(estimator1.getAccumulatedAccelerometerSamples(), estimator2.getAccumulatedAccelerometerSamples());
        assertEquals(estimator1.getAccumulatedGyroscopeSamples(), estimator2.getAccumulatedGyroscopeSamples());
        final var accumulatedOrientation2 = (Quaternion) estimator2.getAccumulatedOrientation();
        assertEquals(accumulatedOrientation1, accumulatedOrientation2);
        assertEquals(estimator1.getAccumulatedAccelerationSampleX(), estimator2.getAccumulatedAccelerationSampleX(),
                0.0);
        assertEquals(estimator1.getAccumulatedAccelerationSampleY(), estimator2.getAccumulatedAccelerationSampleY(),
                0.0);
        assertEquals(estimator1.getAccumulatedAccelerationSampleZ(), estimator2.getAccumulatedAccelerationSampleZ(),
                0.0);
        assertEquals(estimator1.getAccumulatedAngularSpeedSampleX(), estimator2.getAccumulatedAngularSpeedSampleX(),
                0.0);
        assertEquals(estimator1.getAccumulatedAngularSpeedSampleY(), estimator2.getAccumulatedAngularSpeedSampleY(),
                0.0);
        assertEquals(estimator1.getAccumulatedAngularSpeedSampleZ(), estimator2.getAccumulatedAngularSpeedSampleZ(),
                0.0);
        assertEquals(estimator1.getPositionCovarianceMatrix(), estimator2.getPositionCovarianceMatrix());
    }

    @Override
    public void onFullSampleReceived(final BaseSlamEstimator<AbsoluteOrientationSlamCalibrationData> estimator) {
        fullSampleReceived++;
    }

    @Override
    public void onFullSampleProcessed(final BaseSlamEstimator<AbsoluteOrientationSlamCalibrationData> estimator) {
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
        fullSampleReceived = fullSampleProcessed = correctWithPositionMeasure = correctedWithPositionMeasure = 0;
    }

    private static AbsoluteOrientationSlamCalibrator createFinishedCalibrator(
            final float accelerationOffsetX, final float accelerationOffsetY, final float accelerationOffsetZ,
            final float angularOffsetX, final float angularOffsetY, final float angularOffsetZ,
            final GaussianRandomizer noiseRandomizer) {
        final var calibrator = AbsoluteOrientationSlamEstimator.createCalibrator();
        calibrator.setConvergenceThreshold(ABSOLUTE_ERROR);
        calibrator.setMaxNumSamples(MAX_CALIBRATION_SAMPLES);

        var timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;

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
        final var orientation = new Quaternion();

        for (var i = 0; i < MAX_CALIBRATION_SAMPLES; i++) {
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

            calibrator.updateAccelerometerSample(timestamp, (float) accelerationX, (float) accelerationY,
                    (float) accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float) angularX, (float) angularY, (float) angularZ);
            calibrator.updateOrientationSample(timestamp, orientation);

            if (calibrator.isFinished()) {
                break;
            }

            timestamp += DELTA_NANOS;
        }

        return calibrator;
    }
}
