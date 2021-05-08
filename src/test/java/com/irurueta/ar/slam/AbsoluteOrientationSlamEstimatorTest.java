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
import com.irurueta.ar.slam.BaseSlamEstimator.BaseSlamEstimatorListener;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.numerical.signal.processing.KalmanFilter;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

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
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        // check initial values
        assertNull(estimator.getListener());
        assertNull(estimator.getCalibrationData());
        assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
        assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
        assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
        assertArrayEquals(estimator.getStatePosition(),
                new double[]{0.0, 0.0, 0.0}, 0.0);
        final double[] position = new double[3];
        estimator.getStatePosition(position);
        assertArrayEquals(position, new double[]{0.0, 0.0, 0.0}, 0.0);

        assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
        assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
        assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
        assertArrayEquals(estimator.getStateVelocity(),
                new double[]{0.0, 0.0, 0.0}, 0.0);
        final double[] velocity = new double[3];
        estimator.getStateVelocity(velocity);
        assertArrayEquals(velocity, new double[]{0.0, 0.0, 0.0}, 0.0);

        assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
        assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
        assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
        assertArrayEquals(estimator.getStateAcceleration(),
                new double[]{0.0, 0.0, 0.0}, 0.0);
        final double[] acceleration = new double[3];
        estimator.getStateAcceleration(acceleration);
        assertArrayEquals(acceleration, new double[]{0.0, 0.0, 0.0}, 0.0);

        assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
        assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
        assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
        assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
        assertArrayEquals(estimator.getStateQuaternionArray(),
                new double[]{1.0, 0.0, 0.0, 0.0}, 0.0);
        final double[] quaternionArray = new double[4];
        estimator.getStateQuaternionArray(quaternionArray);
        assertArrayEquals(quaternionArray, new double[]{1.0, 0.0, 0.0, 0.0}, 0.0);

        Quaternion q = estimator.getStateQuaternion();
        assertEquals(q.getA(), 1.0, 0.0);
        assertEquals(q.getB(), 0.0, 0.0);
        assertEquals(q.getC(), 0.0, 0.0);
        assertEquals(q.getD(), 0.0, 0.0);

        q = new Quaternion();
        estimator.getStateQuaternion(q);
        assertEquals(q.getA(), 1.0, 0.0);
        assertEquals(q.getB(), 0.0, 0.0);
        assertEquals(q.getC(), 0.0, 0.0);
        assertEquals(q.getD(), 0.0, 0.0);

        assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
        assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
        assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
        assertArrayEquals(estimator.getStateAngularSpeed(),
                new double[]{0.0, 0.0, 0.0}, 0.0);
        final double[] angularSpeed = new double[3];
        estimator.getStateAngularSpeed(angularSpeed);
        assertArrayEquals(angularSpeed, new double[]{0.0, 0.0, 0.0}, 0.0);

        assertFalse(estimator.hasError());
        assertTrue(estimator.isAccumulationEnabled());
        assertEquals(estimator.getAccelerometerTimestampNanos(), -1);
        assertEquals(estimator.getGyroscopeTimestampNanos(), -1);
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 0);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(estimator.isAccelerometerSampleReceived());
        assertFalse(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());

        assertEquals(estimator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        assertArrayEquals(estimator.getAccumulatedAccelerationSample(),
                new double[]{0.0, 0.0, 0.0}, 0.0);
        final double[] accumAcc = new double[3];
        estimator.getAccumulatedAccelerationSample(accumAcc);
        assertArrayEquals(accumAcc, new double[]{0.0, 0.0, 0.0}, 0.0);

        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertArrayEquals(estimator.getAccumulatedAngularSpeedSample(),
                new double[]{0.0, 0.0, 0.0}, 0.0);
        final double[] accumAngularSpeed = new double[3];
        estimator.getAccumulatedAngularSpeedSample(accumAngularSpeed);
        assertArrayEquals(accumAngularSpeed, new double[]{0.0, 0.0, 0.0}, 0.0);

        assertEquals(estimator.getMostRecentTimestampNanos(), -1);

        assertEquals(estimator.getPositionCovarianceMatrix(),
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        KalmanFilter.DEFAULT_MEASUREMENT_NOISE_VARIANCE));

        assertEquals(estimator.getOrientationTimestampNanos(), -1);
        assertTrue(estimator.getAccumulatedOrientation() instanceof Quaternion);
        Quaternion accumulatedOrientation =
                (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(accumulatedOrientation.getA(), 1.0, ABSOLUTE_ERROR);
        assertEquals(accumulatedOrientation.getB(), 0.0, ABSOLUTE_ERROR);
        assertEquals(accumulatedOrientation.getC(), 0.0, ABSOLUTE_ERROR);
        assertEquals(accumulatedOrientation.getD(), 0.0, ABSOLUTE_ERROR);
        accumulatedOrientation = new Quaternion();
        estimator.getAccumulatedOrientation(accumulatedOrientation);
        assertEquals(accumulatedOrientation.getA(), 1.0, ABSOLUTE_ERROR);
        assertEquals(accumulatedOrientation.getB(), 0.0, ABSOLUTE_ERROR);
        assertEquals(accumulatedOrientation.getC(), 0.0, ABSOLUTE_ERROR);
        assertEquals(accumulatedOrientation.getD(), 0.0, ABSOLUTE_ERROR);

        assertEquals(estimator.getAccumulatedOrientationSamples(), 0);
        assertFalse(estimator.isOrientationSampleReceived());
        assertNotNull(estimator.getStateCovariance());
    }

    @Test
    public void testGetSetListener() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        // initial value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetCalibrationData() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        // initial value
        assertNull(estimator.getCalibrationData());

        // set new value
        final AbsoluteOrientationSlamCalibrationData data =
                new AbsoluteOrientationSlamCalibrationData();
        estimator.setCalibrationData(data);

        // check correctness
        assertSame(estimator.getCalibrationData(), data);
    }

    @Test
    public void testResetPosition() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        estimator.resetPosition();

        assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
        assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
        assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);

        assertEquals(estimator.getMostRecentTimestampNanos(), -1);
    }

    @Test
    public void testResetVelocity() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        estimator.resetVelocity();

        assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
        assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
        assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);

        assertEquals(estimator.getMostRecentTimestampNanos(), -1);
    }

    @Test
    public void testResetPositionAndVelocity() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        estimator.resetPositionAndVelocity();

        assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
        assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
        assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);

        assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
        assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
        assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);

        assertEquals(estimator.getMostRecentTimestampNanos(), -1);
    }

    @Test
    public void testResetAcceleration() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        estimator.resetAcceleration();

        assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
        assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
        assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);

        assertEquals(estimator.getMostRecentTimestampNanos(), -1);
    }

    @Test
    public void testResetOrientation() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        estimator.resetOrientation();

        assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
        assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
        assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
        assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);

        assertEquals(estimator.getMostRecentTimestampNanos(), -1);
    }

    @Test
    public void testResetAngularSpeed() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        estimator.resetAngularSpeed();

        assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
        assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
        assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);

        assertEquals(estimator.getMostRecentTimestampNanos(), -1);
    }

    @Test
    public void testReset() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        estimator.reset();

        assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
        assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
        assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);

        assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
        assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
        assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);

        assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
        assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
        assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);

        assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
        assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
        assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
        assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);

        assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
        assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
        assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);

        assertEquals(estimator.getMostRecentTimestampNanos(), -1);
    }

    @Test
    public void testGetStatePositionX() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(estimator.getStatePositionX(), 0.0, 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStatePositionX = value;

        // check correctness
        assertEquals(estimator.getStatePositionX(), value, 0.0);
    }

    @Test
    public void testGetStatePositionY() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(estimator.getStatePositionY(), 0.0, 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStatePositionY = value;

        // check correctness
        assertEquals(estimator.getStatePositionY(), value, 0.0);
    }

    @Test
    public void testGetStatePositionZ() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStatePositionZ = value;

        // check correctness
        assertEquals(estimator.getStatePositionZ(), value, 0.0);
    }

    @Test
    public void testGetStatePosition() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        estimator.mStatePositionX = valueX;
        estimator.mStatePositionY = valueY;
        estimator.mStatePositionZ = valueZ;

        // check correctness
        assertArrayEquals(estimator.getStatePosition(),
                new double[]{valueX, valueY, valueZ}, 0.0);

        final double[] position = new double[3];
        estimator.getStatePosition(position);

        // check correctness
        assertArrayEquals(position, new double[]{valueX, valueY, valueZ}, 0.0);

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
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStateVelocityX = value;

        // check correctness
        assertEquals(estimator.getStateVelocityX(), value, 0.0);
    }

    @Test
    public void testGetStateVelocityY() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStateVelocityY = value;

        // check correctness
        assertEquals(estimator.getStateVelocityY(), value, 0.0);
    }

    @Test
    public void testGetStateVelocityZ() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStateVelocityZ = value;

        // check correctness
        assertEquals(estimator.getStateVelocityZ(), value, 0.0);
    }

    @Test
    public void testGetStateVelocity() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        estimator.mStateVelocityX = valueX;
        estimator.mStateVelocityY = valueY;
        estimator.mStateVelocityZ = valueZ;

        // check correctness
        assertArrayEquals(estimator.getStateVelocity(),
                new double[]{valueX, valueY, valueZ}, 0.0);

        final double[] velocity = new double[3];
        estimator.getStateVelocity(velocity);

        // check correctness
        assertArrayEquals(velocity, new double[]{valueX, valueY, valueZ}, 0.0);

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
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStateAccelerationX = value;

        // check correctness
        assertEquals(estimator.getStateAccelerationX(), value, 0.0);
    }

    @Test
    public void testGetStateAccelerationY() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStateAccelerationY = value;

        // check correctness
        assertEquals(estimator.getStateAccelerationY(), value, 0.0);
    }

    @Test
    public void testGetStateAccelerationZ() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);

        final double value = randomizer.nextDouble();
        estimator.mStateAccelerationZ = value;

        // check correctness
        assertEquals(estimator.getStateAccelerationZ(), value, 0.0);
    }

    @Test
    public void testGetStateAcceleration() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        estimator.mStateAccelerationX = valueX;
        estimator.mStateAccelerationY = valueY;
        estimator.mStateAccelerationZ = valueZ;

        // check correctness
        assertArrayEquals(estimator.getStateAcceleration(),
                new double[]{valueX, valueY, valueZ}, 0.0);

        final double[] acceleration = new double[3];
        estimator.getStateAcceleration(acceleration);

        // check correctness
        assertArrayEquals(acceleration, new double[]{valueX, valueY, valueZ},
                0.0);

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
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateQuaternionA = value;

        // check correctness
        assertEquals(estimator.getStateQuaternionA(), value, 0.0);
    }

    @Test
    public void testGetStateQuaternionB() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateQuaternionB = value;

        // check correctness
        assertEquals(estimator.getStateQuaternionB(), value, 0.0);
    }

    @Test
    public void testGetStateQuaternionC() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateQuaternionC = value;

        // check correctness
        assertEquals(estimator.getStateQuaternionC(), value, 0.0);
    }

    @Test
    public void testGetStateQuaternionD() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateQuaternionD = value;

        // check correctness
        assertEquals(estimator.getStateQuaternionD(), value, 0.0);
    }

    @Test
    public void testGetStateQuaternionArray() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertArrayEquals(estimator.getStateQuaternionArray(),
                new double[]{1.0, 0.0, 0.0, 0.0}, 0.0);

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
        assertArrayEquals(estimator.getStateQuaternionArray(),
                new double[]{valueA, valueB, valueC, valueD}, 0.0);

        final double[] quaternion = new double[4];
        estimator.getStateQuaternionArray(quaternion);

        assertArrayEquals(quaternion,
                new double[]{valueA, valueB, valueC, valueD}, 0.0);

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
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        Quaternion q = estimator.getStateQuaternion();

        assertEquals(q.getA(), 1.0, 0.0);
        assertEquals(q.getB(), 0.0, 0.0);
        assertEquals(q.getC(), 0.0, 0.0);
        assertEquals(q.getD(), 0.0, 0.0);

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

        assertEquals(q.getA(), valueA, 0.0);
        assertEquals(q.getB(), valueB, 0.0);
        assertEquals(q.getC(), valueC, 0.0);
        assertEquals(q.getD(), valueD, 0.0);

        q = new Quaternion();
        estimator.getStateQuaternion(q);

        assertEquals(q.getA(), valueA, 0.0);
        assertEquals(q.getB(), valueB, 0.0);
        assertEquals(q.getC(), valueC, 0.0);
        assertEquals(q.getD(), valueD, 0.0);
    }

    @Test
    public void testGetStateAngularSpeedX() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateAngularSpeedX = value;

        // check correctness
        assertEquals(estimator.getStateAngularSpeedX(), value, 0.0);
    }

    @Test
    public void testGetStateAngularSpeedY() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateAngularSpeedY = value;

        // check correctness
        assertEquals(estimator.getStateAngularSpeedY(), value, 0.0);
    }

    @Test
    public void testGetStateAngularSpeedZ() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mStateAngularSpeedZ = value;

        // check correctness
        assertEquals(estimator.getStateAngularSpeedZ(), value, 0.0);
    }

    @Test
    public void testGetStateAngularSpeed() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial value
        assertArrayEquals(estimator.getStateAngularSpeed(),
                new double[]{0.0, 0.0, 0.0}, 0.0);

        // set new value
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        estimator.mStateAngularSpeedX = valueX;
        estimator.mStateAngularSpeedY = valueY;
        estimator.mStateAngularSpeedZ = valueZ;

        // check correctness
        assertArrayEquals(estimator.getStateAngularSpeed(),
                new double[]{valueX, valueY, valueZ}, 0.0);

        final double[] angularSpeed = new double[3];
        estimator.getStateAngularSpeed(angularSpeed);

        // check correctness
        assertArrayEquals(angularSpeed, new double[]{valueX, valueY, valueZ},
                0.0);

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
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        assertFalse(estimator.hasError());

        estimator.mError = true;

        assertTrue(estimator.hasError());
    }

    @Test
    public void testIsAccumulationEnabled() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        assertTrue(estimator.isAccumulationEnabled());

        estimator.setAccumulationEnabled(false);

        assertFalse(estimator.isAccumulationEnabled());
    }

    @Test
    public void testGetAccelerometerTimestampNanos() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        assertEquals(estimator.getAccelerometerTimestampNanos(), -1);

        // set new value
        estimator.mAccelerometerTimestampNanos = 1000;

        // check correctness
        assertEquals(estimator.getAccelerometerTimestampNanos(), 1000);
    }

    @Test
    public void testGetGyroscopeTimestampNanos() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        assertEquals(estimator.getGyroscopeTimestampNanos(), -1);

        // set new value
        estimator.mGyroscopeTimestampNanos = 2000;

        // check correctness
        assertEquals(estimator.getGyroscopeTimestampNanos(), 2000);
    }

    @Test
    public void testGetOrientationTimestampNanos() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        assertEquals(estimator.getOrientationTimestampNanos(), -1);

        // set new value
        estimator.mOrientationTimestampNanos = 3000;

        // check correctness
        assertEquals(estimator.getOrientationTimestampNanos(), 3000);
    }

    @Test
    public void testGetAccumulatedAccelerometerSamplesAndIsAccelerometerSampleReceived() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 0);
        assertFalse(estimator.isAccelerometerSampleReceived());

        // set new value
        estimator.mAccumulatedAccelerometerSamples = 10;

        // check correctness
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 10);
        assertTrue(estimator.isAccelerometerSampleReceived());
    }

    @Test
    public void testGetAccumulatedGyroscopeSamplesAndIsGyroscopeSampleReceived() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(estimator.isGyroscopeSampleReceived());

        // set new value
        estimator.mAccumulatedGyroscopeSamples = 20;

        // check correctness
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 20);
        assertTrue(estimator.isGyroscopeSampleReceived());
    }

    @Test
    public void testGetAccumulatedOrientationAndIsOrientationSampleReceived() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        assertEquals(estimator.getAccumulatedOrientationSamples(), 0);
        assertFalse(estimator.isOrientationSampleReceived());

        // set new value
        final Quaternion q = new Quaternion(randomizer.nextDouble(),
                randomizer.nextDouble(), randomizer.nextDouble(),
                randomizer.nextDouble());
        estimator.mAccumulatedOrientation = q;

        // check correctness
        final Quaternion accumulatedOrientation =
                (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(accumulatedOrientation.getA(), q.getA(), ABSOLUTE_ERROR);
        assertEquals(accumulatedOrientation.getB(), q.getB(), ABSOLUTE_ERROR);
        assertEquals(accumulatedOrientation.getC(), q.getC(), ABSOLUTE_ERROR);
        assertEquals(accumulatedOrientation.getD(), q.getD(), ABSOLUTE_ERROR);
    }

    @Test
    public void testIsFullSampleAvailable() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 0);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 0);
        assertEquals(estimator.getAccumulatedOrientationSamples(), 0);
        assertFalse(estimator.isAccelerometerSampleReceived());
        assertFalse(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.mAccumulatedAccelerometerSamples = 1;

        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 1);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 0);
        assertEquals(estimator.getAccumulatedOrientationSamples(), 0);
        assertTrue(estimator.isAccelerometerSampleReceived());
        assertFalse(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.mAccumulatedGyroscopeSamples = 2;

        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 1);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 2);
        assertEquals(estimator.getAccumulatedOrientationSamples(), 0);
        assertTrue(estimator.isAccelerometerSampleReceived());
        assertTrue(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());

        estimator.mAccumulatedOrientationSamples = 3;

        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 1);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 2);
        assertEquals(estimator.getAccumulatedOrientationSamples(), 3);
        assertTrue(estimator.isAccelerometerSampleReceived());
        assertTrue(estimator.isGyroscopeSampleReceived());
        assertTrue(estimator.isFullSampleAvailable());
    }

    @Test
    public void testGetAccumulatedAccelerationSampleX() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(estimator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);

        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleX = value;

        // check correctness
        assertEquals(estimator.getAccumulatedAccelerationSampleX(), value, 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSampleY() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(estimator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);

        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleY = value;

        // check correctness
        assertEquals(estimator.getAccumulatedAccelerationSampleY(), value, 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSampleZ() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(estimator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);

        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleZ = value;

        // check correctness
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(), value, 0.0);
    }

    @Test
    public void testGetAccumulatedAccelerationSample() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertArrayEquals(estimator.getAccumulatedAccelerationSample(),
                new double[]{0.0, 0.0, 0.0}, 0.0);

        // set new values
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleX = valueX;
        estimator.mAccumulatedAccelerationSampleY = valueY;
        estimator.mAccumulatedAccelerationSampleZ = valueZ;

        // check correctness
        assertArrayEquals(estimator.getAccumulatedAccelerationSample(),
                new double[]{valueX, valueY, valueZ}, 0.0);

        final double[] acceleration = new double[3];
        estimator.getAccumulatedAccelerationSample(acceleration);

        // check correctness
        assertArrayEquals(acceleration, new double[]{valueX, valueY, valueZ},
                0.0);

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
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAngularSpeedSampleX = value;

        // check correctness
        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(), value, 0.0);
    }

    @Test
    public void testGetAccumulatedAngularSpeedSampleY() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAngularSpeedSampleY = value;

        // check correctness
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(), value, 0.0);
    }

    @Test
    public void testGetAccumulatedAngularSpeedSampleZ() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);

        // set new value
        final double value = randomizer.nextDouble();
        estimator.mAccumulatedAngularSpeedSampleZ = value;

        // check correctness
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(), value, 0.0);
    }

    @Test
    public void testGetAccumulatedAngularSpeedSample() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        assertArrayEquals(estimator.getAccumulatedAngularSpeedSample(),
                new double[]{0.0, 0.0, 0.0}, 0.0);

        // set new value
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        estimator.mAccumulatedAngularSpeedSampleX = valueX;
        estimator.mAccumulatedAngularSpeedSampleY = valueY;
        estimator.mAccumulatedAngularSpeedSampleZ = valueZ;

        // check correctness
        assertArrayEquals(estimator.getAccumulatedAngularSpeedSample(),
                new double[]{valueX, valueY, valueZ}, 0.0);

        final double[] speed = new double[3];
        estimator.getAccumulatedAngularSpeedSample(speed);

        // check correctness
        assertArrayEquals(speed, new double[]{valueX, valueY, valueZ}, 0.0);

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
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(false);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        long timestamp = System.currentTimeMillis();
        final float accelerationX = randomizer.nextFloat();
        final float accelerationY = randomizer.nextFloat();
        final float accelerationZ = randomizer.nextFloat();

        // check initial values
        assertEquals(estimator.getAccelerometerTimestampNanos(), -1);
        assertEquals(estimator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 0);
        assertFalse(estimator.isFullSampleAvailable());

        estimator.updateAccelerometerSample(timestamp, accelerationX,
                accelerationY, accelerationZ);

        // check correctness
        assertEquals(estimator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(estimator.getAccumulatedAccelerationSampleX(),
                accelerationX, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleY(),
                accelerationY, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(),
                accelerationZ, 0.0);
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 1);
        assertFalse(estimator.isFullSampleAvailable());

        // test again but using an array
        final float[] acceleration = new float[3];
        randomizer.fill(acceleration);
        timestamp += 1000;

        estimator.updateAccelerometerSample(timestamp, acceleration);

        // check correctness
        assertEquals(estimator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(estimator.getAccumulatedAccelerationSampleX(),
                acceleration[0], 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleY(),
                acceleration[1], 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(),
                acceleration[2], 0.0);
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 2);
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
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(true);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(estimator.getAccelerometerTimestampNanos(), -1);
        assertEquals(estimator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 0);
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

            estimator.updateAccelerometerSample(timestamp, accelerationX,
                    accelerationY, accelerationZ);
        }

        // check correctness
        assertEquals(estimator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(estimator.getAccumulatedAccelerationSampleX(),
                avgAccelerationX, ABSOLUTE_ERROR);
        assertEquals(estimator.getAccumulatedAccelerationSampleY(),
                avgAccelerationY, ABSOLUTE_ERROR);
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(),
                avgAccelerationZ, ABSOLUTE_ERROR);
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), TIMES);
        assertFalse(estimator.isFullSampleAvailable());
    }

    @Test
    public void testUpdateGyroscopeSampleWithAccumulationDisabled() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(false);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        long timestamp = System.currentTimeMillis();
        final float angularSpeedX = randomizer.nextFloat();
        final float angularSpeedY = randomizer.nextFloat();
        final float angularSpeedZ = randomizer.nextFloat();

        // check initial values
        assertEquals(estimator.getGyroscopeTimestampNanos(), -1);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(estimator.isFullSampleAvailable());

        estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY,
                angularSpeedZ);

        // check correctness
        assertEquals(estimator.getGyroscopeTimestampNanos(), timestamp);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(),
                angularSpeedX, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(),
                angularSpeedY, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(),
                angularSpeedZ, 0.0);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 1);
        assertFalse(estimator.isFullSampleAvailable());

        // test again but using an array
        final float[] angularSpeed = new float[3];
        randomizer.fill(angularSpeed);
        timestamp += 100;

        estimator.updateGyroscopeSample(timestamp, angularSpeed);

        // check correctness
        assertEquals(estimator.getGyroscopeTimestampNanos(), timestamp);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(),
                angularSpeed[0], 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(),
                angularSpeed[1], 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(),
                angularSpeed[2], 0.0);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 2);
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
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(true);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(estimator.getGyroscopeTimestampNanos(), -1);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 0);
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

            estimator.updateGyroscopeSample(timestamp, angularSpeedX,
                    angularSpeedY, angularSpeedZ);
        }

        // check correctness
        assertEquals(estimator.getGyroscopeTimestampNanos(), timestamp);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(),
                avgAngularSpeedX, ABSOLUTE_ERROR);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(),
                avgAngularSpeedY, ABSOLUTE_ERROR);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(),
                avgAngularSpeedZ, ABSOLUTE_ERROR);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), TIMES);
        assertFalse(estimator.isFullSampleAvailable());
    }

    @Test
    public void testUpdateOrientationSampleWithAccumulationDisabled() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(false);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final long timestamp = System.currentTimeMillis();
        final double orientationA = randomizer.nextDouble();
        final double orientationB = randomizer.nextDouble();
        final double orientationC = randomizer.nextDouble();
        final double orientationD = randomizer.nextDouble();
        final Quaternion orientation = new Quaternion(orientationA, orientationB,
                orientationC, orientationD);

        // check initial values
        assertEquals(estimator.getOrientationTimestampNanos(), -1);
        Quaternion accumulatedOrientation =
                (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(accumulatedOrientation.getA(), 1.0, 0.0);
        assertEquals(accumulatedOrientation.getB(), 0.0, 0.0);
        assertEquals(accumulatedOrientation.getC(), 0.0, 0.0);
        assertEquals(accumulatedOrientation.getD(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedOrientationSamples(), 0);
        assertFalse(estimator.isFullSampleAvailable());

        estimator.updateOrientationSample(timestamp, orientation);

        // check correctness
        assertEquals(estimator.getOrientationTimestampNanos(), timestamp);
        accumulatedOrientation =
                (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(accumulatedOrientation.getA(), orientationA, 0.0);
        assertEquals(accumulatedOrientation.getB(), orientationB, 0.0);
        assertEquals(accumulatedOrientation.getC(), orientationC, 0.0);
        assertEquals(accumulatedOrientation.getD(), orientationD, 0.0);
    }

    @Test
    public void testUpdateOrientationSampleWithAccumulationEnabled() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();
        estimator.setAccumulationEnabled(true);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // check initial values
        assertEquals(estimator.getOrientationTimestampNanos(), -1);
        Quaternion accumulatedOrientation =
                (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(accumulatedOrientation.getA(), 1.0, 0.0);
        assertEquals(accumulatedOrientation.getB(), 0.0, 0.0);
        assertEquals(accumulatedOrientation.getC(), 0.0, 0.0);
        assertEquals(accumulatedOrientation.getD(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedOrientationSamples(), 0);
        assertFalse(estimator.isFullSampleAvailable());

        // update with several samples
        long timestamp = System.currentTimeMillis();
        double orientationA;
        double orientationB;
        double orientationC;
        double orientationD;
        double avgOrientationA = 0.0, avgOrientationB = 0.0,
                avgOrientationC = 0.0, avgOrientationD = 0.0;
        Quaternion orientation = new Quaternion();
        double norm;
        for (int i = 0; i < TIMES; i++) {
            timestamp += 1000;
            orientationA = randomizer.nextDouble();
            orientationB = randomizer.nextDouble();
            orientationC = randomizer.nextDouble();
            orientationD = randomizer.nextDouble();
            norm = Math.sqrt(orientationA * orientationA +
                    orientationB * orientationB +
                    orientationC * orientationC +
                    orientationD * orientationD);
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
        accumulatedOrientation =
                (Quaternion) estimator.getAccumulatedOrientation();
        assertEquals(accumulatedOrientation.getA(), avgOrientationA,
                ABSOLUTE_ERROR);
        assertEquals(accumulatedOrientation.getB(), avgOrientationB,
                ABSOLUTE_ERROR);
        assertEquals(accumulatedOrientation.getC(), avgOrientationC,
                ABSOLUTE_ERROR);
        assertEquals(accumulatedOrientation.getD(), avgOrientationD,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetMostRecentTimestampNanos() {
        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        final long timestamp = System.currentTimeMillis();
        estimator.mAccelerometerTimestampNanos = timestamp;
        estimator.mGyroscopeTimestampNanos = timestamp + 1000;

        assertEquals(estimator.getMostRecentTimestampNanos(), timestamp + 1000);

        estimator.mAccelerometerTimestampNanos = timestamp + 2000;

        assertEquals(estimator.getMostRecentTimestampNanos(), timestamp + 2000);

        estimator.mOrientationTimestampNanos = timestamp + 3000;

        assertEquals(estimator.getMostRecentTimestampNanos(), timestamp + 3000);
    }

    @Test
    public void testCorrectWithPositionMeasureCoordinatesAndCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double positionX = randomizer.nextDouble();
            final double positionY = randomizer.nextDouble();
            final double positionZ = randomizer.nextDouble();

            final Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);

            // check initial value
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            final Quaternion orientation = new Quaternion();
            estimator.updateOrientationSample(timestamp, orientation);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // correct has no effect until we predict again
            estimator.correctWithPositionMeasure(
                    positionX, positionY, positionZ, positionCovariance);

            // check correctness
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateOrientationSample(timestamp, orientation);

            // check correctness
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(fullSampleReceived, 2);
            assertEquals(fullSampleProcessed, 2);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(
                    positionX, positionY, positionZ, positionCovariance);

            // expected velocities
            final double vx = VELOCITY_GAIN * positionX;
            final double vy = VELOCITY_GAIN * positionY;
            final double vz = VELOCITY_GAIN * positionZ;

            // check correctness
            assertEquals(estimator.getStatePositionX(), positionX,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionY(), positionY,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionZ(), positionZ,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityX(), vx, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityY(), vy, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityZ(), vz, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            Quaternion stateQuaternion = estimator.getStateQuaternion();
            stateQuaternion.normalize();
            assertEquals(stateQuaternion.getA(), 1.0, 0.0);
            assertEquals(stateQuaternion.getB(), 0.0, 0.0);
            assertEquals(stateQuaternion.getC(), 0.0, 0.0);
            assertEquals(stateQuaternion.getD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(correctWithPositionMeasure, 1);
            assertEquals(correctedWithPositionMeasure, 1);

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
    public void testCorrectWithPositionMeasureArrayAndCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double positionX = randomizer.nextDouble();
            final double positionY = randomizer.nextDouble();
            final double positionZ = randomizer.nextDouble();
            final double[] position = new double[]{positionX, positionY, positionZ};

            final Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);

            // check initial value
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            final Quaternion orientation = new Quaternion();
            estimator.updateOrientationSample(timestamp, orientation);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // correct has no effect until we predict again
            estimator.correctWithPositionMeasure(position, positionCovariance);

            // check correctness
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateOrientationSample(timestamp, orientation);

            // check correctness
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(fullSampleReceived, 2);
            assertEquals(fullSampleProcessed, 2);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(position, positionCovariance);

            // expected velocities
            final double vx = VELOCITY_GAIN * positionX;
            final double vy = VELOCITY_GAIN * positionY;
            final double vz = VELOCITY_GAIN * positionZ;

            // check correctness
            assertEquals(estimator.getStatePositionX(), positionX,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionY(), positionY,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionZ(), positionZ,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityX(), vx, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityY(), vy, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityZ(), vz, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            Quaternion stateQuaternion = estimator.getStateQuaternion();
            stateQuaternion.normalize();
            assertEquals(stateQuaternion.getA(), 1.0, 0.0);
            assertEquals(stateQuaternion.getB(), 0.0, 0.0);
            assertEquals(stateQuaternion.getC(), 0.0, 0.0);
            assertEquals(stateQuaternion.getD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(correctWithPositionMeasure, 1);
            assertEquals(correctedWithPositionMeasure, 1);

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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double positionX = randomizer.nextDouble();
            final double positionY = randomizer.nextDouble();
            final double positionZ = randomizer.nextDouble();
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(positionX,
                    positionY, positionZ);

            final Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);

            // check initial value
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            final Quaternion orientation = new Quaternion();
            estimator.updateOrientationSample(timestamp, orientation);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // correct has no effect until we predict again
            estimator.correctWithPositionMeasure(position, positionCovariance);

            // check correctness
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateOrientationSample(timestamp, orientation);

            // check correctness
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(fullSampleReceived, 2);
            assertEquals(fullSampleProcessed, 2);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(position, positionCovariance);

            // expected velocities
            final double vx = VELOCITY_GAIN * positionX;
            final double vy = VELOCITY_GAIN * positionY;
            final double vz = VELOCITY_GAIN * positionZ;

            // check correctness
            assertEquals(estimator.getStatePositionX(), positionX,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionY(), positionY,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionZ(), positionZ,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityX(), vx, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityY(), vy, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityZ(), vz, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            final Quaternion stateQuaternion = estimator.getStateQuaternion();
            stateQuaternion.normalize();
            assertEquals(stateQuaternion.getA(), 1.0, 0.0);
            assertEquals(stateQuaternion.getB(), 0.0, 0.0);
            assertEquals(stateQuaternion.getC(), 0.0, 0.0);
            assertEquals(stateQuaternion.getD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(correctWithPositionMeasure, 1);
            assertEquals(correctedWithPositionMeasure, 1);

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

        final AbsoluteOrientationSlamEstimator estimator =
                new AbsoluteOrientationSlamEstimator();

        // check initial value
        assertEquals(estimator.getPositionCovarianceMatrix(),
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        KalmanFilter.DEFAULT_MEASUREMENT_NOISE_VARIANCE));

        // set new value
        final Matrix positionCovariance = Matrix.identity(3, 3).
                multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);

        estimator.setPositionCovarianceMatrix(positionCovariance);

        // check correctness
        assertEquals(estimator.getPositionCovarianceMatrix(),
                positionCovariance);

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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double positionX = randomizer.nextDouble();
            final double positionY = randomizer.nextDouble();
            final double positionZ = randomizer.nextDouble();

            final Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);
            estimator.setPositionCovarianceMatrix(positionCovariance);

            // check initial value
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            final Quaternion orientation = new Quaternion();
            estimator.updateOrientationSample(timestamp, orientation);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // correct has no effect until we predict again
            estimator.correctWithPositionMeasure(positionX, positionY,
                    positionZ);

            // check correctness
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateOrientationSample(timestamp, orientation);

            // check correctness
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(fullSampleReceived, 2);
            assertEquals(fullSampleProcessed, 2);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(positionX, positionY,
                    positionZ);

            // expected velocities
            final double vx = VELOCITY_GAIN * positionX;
            final double vy = VELOCITY_GAIN * positionY;
            final double vz = VELOCITY_GAIN * positionZ;

            // check correctness
            assertEquals(estimator.getStatePositionX(), positionX,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionY(), positionY,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionZ(), positionZ,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityX(), vx, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityY(), vy, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityZ(), vz, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            Quaternion stateQuaternion = estimator.getStateQuaternion();
            stateQuaternion.normalize();
            assertEquals(stateQuaternion.getA(), 1.0, 0.0);
            assertEquals(stateQuaternion.getB(), 0.0, 0.0);
            assertEquals(stateQuaternion.getC(), 0.0, 0.0);
            assertEquals(stateQuaternion.getD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(correctWithPositionMeasure, 1);
            assertEquals(correctedWithPositionMeasure, 1);
        }
    }

    @Test
    public void testCorrectWithPositionMeasureArrayWithoutCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();
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
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            final Quaternion orientation = new Quaternion();
            estimator.updateOrientationSample(timestamp, orientation);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // correct has no effect until we predict again
            estimator.correctWithPositionMeasure(position);

            // check correctness
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateOrientationSample(timestamp, orientation);

            // check correctness
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(fullSampleReceived, 2);
            assertEquals(fullSampleProcessed, 2);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(position);

            // expected velocities
            final double vx = VELOCITY_GAIN * positionX;
            final double vy = VELOCITY_GAIN * positionY;
            final double vz = VELOCITY_GAIN * positionZ;

            // check correctness
            assertEquals(estimator.getStatePositionX(), positionX,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionY(), positionY,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionZ(), positionZ,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityX(), vx, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityY(), vy, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityZ(), vz, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            final Quaternion stateQuaternion = estimator.getStateQuaternion();
            stateQuaternion.normalize();
            assertEquals(stateQuaternion.getA(), 1.0, 0.0);
            assertEquals(stateQuaternion.getB(), 0.0, 0.0);
            assertEquals(stateQuaternion.getC(), 0.0, 0.0);
            assertEquals(stateQuaternion.getD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(correctWithPositionMeasure, 1);
            assertEquals(correctedWithPositionMeasure, 1);

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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();
            estimator.setListener(this);

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double positionX = randomizer.nextDouble();
            final double positionY = randomizer.nextDouble();
            final double positionZ = randomizer.nextDouble();
            final InhomogeneousPoint3D position = new InhomogeneousPoint3D(positionX,
                    positionY, positionZ);

            final Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);
            estimator.setPositionCovarianceMatrix(positionCovariance);

            // check initial value
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertTrue(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            final Quaternion orientation = new Quaternion();
            estimator.updateOrientationSample(timestamp, orientation);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertFalse(estimator.isOrientationSampleReceived());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // correct has no effect until we predict again
            estimator.correctWithPositionMeasure(position);

            // check correctness
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateOrientationSample(timestamp, orientation);

            // check correctness
            assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
            assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
            assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(fullSampleReceived, 2);
            assertEquals(fullSampleProcessed, 2);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

            // correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(position);

            // expected velocities
            final double vx = VELOCITY_GAIN * positionX;
            final double vy = VELOCITY_GAIN * positionY;
            final double vz = VELOCITY_GAIN * positionZ;

            // check correctness
            assertEquals(estimator.getStatePositionX(), positionX,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionY(), positionY,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionZ(), positionZ,
                    ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityX(), vx, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityY(), vy, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityZ(), vz, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
            assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
            final Quaternion stateQuaternion = estimator.getStateQuaternion();
            stateQuaternion.normalize();
            assertEquals(stateQuaternion.getA(), 1.0, 0.0);
            assertEquals(stateQuaternion.getB(), 0.0, 0.0);
            assertEquals(stateQuaternion.getC(), 0.0, 0.0);
            assertEquals(stateQuaternion.getD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(correctWithPositionMeasure, 1);
            assertEquals(correctedWithPositionMeasure, 1);

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
        //noinspection ConstantConditions
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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
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

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(),
                    estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
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

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(),
                    estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
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

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(),
                    estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
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

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(),
                    estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

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
            final int offsetAccelerationX = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
            final int offsetAccelerationY = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
            final int offsetAccelerationZ = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
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

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(),
                    estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

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
            final int offsetAngularSpeedX = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
            final int offsetAngularSpeedY = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
            final int offsetAngularSpeedZ = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
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

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(),
                    estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) <
                            predictedPos.distanceTo(groundTruthPos);

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
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());
            final float gtQuaternionA = (float) gtQuaternion.getA();
            final float gtQuaternionB = (float) gtQuaternion.getB();
            final float gtQuaternionC = (float) gtQuaternion.getC();
            final float gtQuaternionD = (float) gtQuaternion.getD();

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
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

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(),
                    estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) <
                            predictedPos.distanceTo(groundTruthPos);

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
            final int offsetAccelerationX = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
            final int offsetAccelerationY = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
            final int offsetAccelerationZ = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
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
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());
            final float gtQuaternionA = (float) gtQuaternion.getA();
            final float gtQuaternionB = (float) gtQuaternion.getB();
            final float gtQuaternionC = (float) gtQuaternion.getC();
            final float gtQuaternionD = (float) gtQuaternion.getD();

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
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

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(),
                    estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) <
                            predictedPos.distanceTo(groundTruthPos);

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
            final int offsetAngularSpeedX = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
            final int offsetAngularSpeedY = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
            final int offsetAngularSpeedZ = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
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
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());
            final float gtQuaternionA = (float) gtQuaternion.getA();
            final float gtQuaternionB = (float) gtQuaternion.getB();
            final float gtQuaternionC = (float) gtQuaternion.getC();
            final float gtQuaternionD = (float) gtQuaternion.getD();

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
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

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(),
                    estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) <
                            predictedPos.distanceTo(groundTruthPos);

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
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());
            final float gtQuaternionA = (float) gtQuaternion.getA();
            float gtQuaternionB = (float) gtQuaternion.getB();
            float gtQuaternionC = (float) gtQuaternion.getC();
            float gtQuaternionD = (float) gtQuaternion.getD();

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
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

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion = estimator.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(),
                    estimator.getStatePositionY(),
                    estimator.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) <
                            predictedPos.distanceTo(groundTruthPos);

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
                    createFinishedCalibrator(accelerationOffsetX,
                            accelerationOffsetY, accelerationOffsetZ,
                            angularOffsetX, angularOffsetY, angularOffsetZ,
                            noiseRandomizer);
            final AbsoluteOrientationSlamCalibrationData calibration =
                    calibrator.getCalibrationData();

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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final AbsoluteOrientationSlamEstimator estimatorWithCalibration =
                    new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);
            estimatorWithCalibration.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY,
                        accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY,
                        angularSpeedWithOffsetZ);
                estimatorWithCalibration.updateOrientationSample(timestamp,
                        orientation);

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

                    deltaAccelerationWithOffsetX = accelerationWithOffsetX -
                            lastAccelerationWithOffsetX;
                    deltaAccelerationWithOffsetY = accelerationWithOffsetY -
                            lastAccelerationWithOffsetY;
                    deltaAccelerationWithOffsetZ = accelerationWithOffsetZ -
                            lastAccelerationWithOffsetZ;

                    deltaAngularSpeedWithOffsetX = angularSpeedWithOffsetX -
                            lastAngularSpeedWithOffsetX;
                    deltaAngularSpeedWithOffsetY = angularSpeedWithOffsetY -
                            lastAngularSpeedWithOffsetY;
                    deltaAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ -
                            lastAngularSpeedWithOffsetZ;

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion =
                    estimatorWithCalibration.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(),
                    estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) <
                            predictedPos.distanceTo(groundTruthPos);

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
                    createFinishedCalibrator(accelerationOffsetX,
                            accelerationOffsetY, accelerationOffsetZ,
                            angularOffsetX, angularOffsetY, angularOffsetZ,
                            noiseRandomizer);
            final AbsoluteOrientationSlamCalibrationData calibration =
                    calibrator.getCalibrationData();

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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final AbsoluteOrientationSlamEstimator estimatorWithCalibration =
                    new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            double deltaAngularSpeedWithOffsetX, deltaAngularSpeedWithOffsetY,
                    deltaAngularSpeedWithOffsetZ;

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);
            estimatorWithCalibration.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY,
                        accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY,
                        angularSpeedWithOffsetZ);
                estimatorWithCalibration.updateOrientationSample(timestamp,
                        orientation);

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

                    deltaAccelerationWithOffsetX = accelerationWithOffsetX -
                            lastAccelerationWithOffsetX;
                    deltaAccelerationWithOffsetY = accelerationWithOffsetY -
                            lastAccelerationWithOffsetY;
                    deltaAccelerationWithOffsetZ = accelerationWithOffsetZ -
                            lastAccelerationWithOffsetZ;

                    deltaAngularSpeedWithOffsetX = angularSpeedWithOffsetX -
                            lastAngularSpeedWithOffsetX;
                    deltaAngularSpeedWithOffsetY = angularSpeedWithOffsetY -
                            lastAngularSpeedWithOffsetY;
                    deltaAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ -
                            lastAngularSpeedWithOffsetZ;

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion =
                    estimatorWithCalibration.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(),
                    estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) <
                            predictedPos.distanceTo(groundTruthPos);

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
                    createFinishedCalibrator(accelerationOffsetX,
                            accelerationOffsetY, accelerationOffsetZ,
                            angularOffsetX, angularOffsetY, angularOffsetZ,
                            noiseRandomizer);
            final AbsoluteOrientationSlamCalibrationData calibration =
                    calibrator.getCalibrationData();

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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final AbsoluteOrientationSlamEstimator estimatorWithCalibration =
                    new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);
            estimatorWithCalibration.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY,
                        accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY,
                        angularSpeedWithOffsetZ);
                estimatorWithCalibration.updateOrientationSample(timestamp,
                        orientation);

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

                    deltaAccelerationWithOffsetX = accelerationWithOffsetX -
                            lastAccelerationWithOffsetX;
                    deltaAccelerationWithOffsetY = accelerationWithOffsetY -
                            lastAccelerationWithOffsetY;
                    deltaAccelerationWithOffsetZ = accelerationWithOffsetZ -
                            lastAccelerationWithOffsetZ;

                    deltaAngularSpeedWithOffsetX = angularSpeedWithOffsetX -
                            lastAngularSpeedWithOffsetX;
                    deltaAngularSpeedWithOffsetY = angularSpeedWithOffsetY -
                            lastAngularSpeedWithOffsetY;
                    deltaAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ -
                            lastAngularSpeedWithOffsetZ;

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion =
                    estimatorWithCalibration.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(),
                    estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) <
                            predictedPos.distanceTo(groundTruthPos);

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
                    createFinishedCalibrator(accelerationOffsetX,
                            accelerationOffsetY, accelerationOffsetZ,
                            angularOffsetX, angularOffsetY, angularOffsetZ,
                            noiseRandomizer);
            final AbsoluteOrientationSlamCalibrationData calibration =
                    calibrator.getCalibrationData();

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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final AbsoluteOrientationSlamEstimator estimatorWithCalibration =
                    new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);
            estimatorWithCalibration.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY,
                        accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY,
                        angularSpeedWithOffsetZ);
                estimatorWithCalibration.updateOrientationSample(timestamp,
                        orientation);

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

                    deltaAccelerationWithOffsetX = accelerationWithOffsetX -
                            lastAccelerationWithOffsetX;
                    deltaAccelerationWithOffsetY = accelerationWithOffsetY -
                            lastAccelerationWithOffsetY;
                    deltaAccelerationWithOffsetZ = accelerationWithOffsetZ -
                            lastAccelerationWithOffsetZ;

                    deltaAngularSpeedWithOffsetX = angularSpeedWithOffsetX -
                            lastAngularSpeedWithOffsetX;
                    deltaAngularSpeedWithOffsetY = angularSpeedWithOffsetY -
                            lastAngularSpeedWithOffsetY;
                    deltaAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ -
                            lastAngularSpeedWithOffsetZ;

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion =
                    estimatorWithCalibration.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(),
                    estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) <
                            predictedPos.distanceTo(groundTruthPos);

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
                    createFinishedCalibrator(accelerationOffsetX,
                            accelerationOffsetY, accelerationOffsetZ,
                            angularOffsetX, angularOffsetY, angularOffsetZ,
                            noiseRandomizer);
            final AbsoluteOrientationSlamCalibrationData calibration =
                    calibrator.getCalibrationData();

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double gtPositionX = 0.0;
            final double gtPositionY = 0.0;
            final double gtPositionZ = 0.0;
            final double gtSpeedX = randomizer.nextDouble();
            final double gtSpeedY = randomizer.nextDouble();
            final double gtSpeedZ = randomizer.nextDouble();
            final int period = N_PREDICTION_SAMPLES / 2;
            final int offsetAccelerationX = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
            final int offsetAccelerationY = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
            final int offsetAccelerationZ = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final AbsoluteOrientationSlamEstimator estimatorWithCalibration =
                    new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);
            estimatorWithCalibration.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY,
                        accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY,
                        angularSpeedWithOffsetZ);
                estimatorWithCalibration.updateOrientationSample(timestamp,
                        orientation);

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

                    deltaAccelerationWithOffsetX = accelerationWithOffsetX -
                            lastAccelerationWithOffsetX;
                    deltaAccelerationWithOffsetY = accelerationWithOffsetY -
                            lastAccelerationWithOffsetY;
                    deltaAccelerationWithOffsetZ = accelerationWithOffsetZ -
                            lastAccelerationWithOffsetZ;

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion =
                    estimatorWithCalibration.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(),
                    estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) <
                            predictedPos.distanceTo(groundTruthPos);

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
                    createFinishedCalibrator(accelerationOffsetX,
                            accelerationOffsetY, accelerationOffsetZ,
                            angularOffsetX, angularOffsetY, angularOffsetZ,
                            noiseRandomizer);
            final AbsoluteOrientationSlamCalibrationData calibration =
                    calibrator.getCalibrationData();

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
            final int offsetAngularSpeedX = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
            final int offsetAngularSpeedY = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
            final int offsetAngularSpeedZ = randomizer.nextInt(0,
                    N_PREDICTION_SAMPLES);
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

            final AbsoluteOrientationSlamEstimator estimator =
                    new AbsoluteOrientationSlamEstimator();

            final AbsoluteOrientationSlamEstimator estimatorWithCalibration =
                    new AbsoluteOrientationSlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            final GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
            final GaussianRandomizer orientationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ORIENTATION_NOISE_STANDARD_DEVIATION);

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
            final Quaternion lastGtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
            final Quaternion deltaGtOrientation = new Quaternion();
            final Quaternion orientation = new Quaternion();
            final Quaternion gtOrientation = new Quaternion(gtQuaternionA,
                    gtQuaternionB, gtQuaternionC, gtQuaternionD);
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
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
                    gtAccelerationX, gtAccelerationY, gtAccelerationZ,
                    gtQuaternionA, gtQuaternionB, gtQuaternionC, gtQuaternionD,
                    gtAngularSpeedX, gtAngularSpeedY, gtAngularSpeedZ);
            estimatorWithCalibration.reset(gtPositionX, gtPositionY, gtPositionZ,
                    gtSpeedX, gtSpeedY, gtSpeedZ,
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

                estimator.updateAccelerometerSample(timestamp,
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp,
                        angularSpeedX, angularSpeedY, angularSpeedZ);
                estimator.updateOrientationSample(timestamp, orientation);

                estimatorWithCalibration.updateAccelerometerSample(timestamp,
                        accelerationWithOffsetX, accelerationWithOffsetY,
                        accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp,
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY,
                        angularSpeedWithOffsetZ);
                estimatorWithCalibration.updateOrientationSample(timestamp,
                        orientation);

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

                    deltaAccelerationWithOffsetX = accelerationWithOffsetX -
                            lastAccelerationWithOffsetX;
                    deltaAccelerationWithOffsetY = accelerationWithOffsetY -
                            lastAccelerationWithOffsetY;
                    deltaAccelerationWithOffsetZ = accelerationWithOffsetZ -
                            lastAccelerationWithOffsetZ;

                    deltaAngularSpeedWithOffsetX = angularSpeedWithOffsetX -
                            lastAngularSpeedWithOffsetX;
                    deltaAngularSpeedWithOffsetY = angularSpeedWithOffsetY -
                            lastAngularSpeedWithOffsetY;
                    deltaAngularSpeedWithOffsetZ = angularSpeedWithOffsetZ -
                            lastAngularSpeedWithOffsetZ;

                    deltaGtAccelerationX = gtAccelerationX -
                            lastGtAccelerationX;
                    deltaGtAccelerationY = gtAccelerationY -
                            lastGtAccelerationY;
                    deltaGtAccelerationZ = gtAccelerationZ -
                            lastGtAccelerationZ;

                    deltaGtAngularSpeedX = gtAngularSpeedX -
                            lastGtAngularSpeedX;
                    deltaGtAngularSpeedY = gtAngularSpeedY -
                            lastGtAngularSpeedY;
                    deltaGtAngularSpeedZ = gtAngularSpeedZ -
                            lastGtAngularSpeedZ;

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
                    StatePredictor.predictWithRotationAdjustment(x, u,
                            DELTA_SECONDS, x);

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
                    StatePredictor.predictWithRotationAdjustment(gtX, gtU,
                            DELTA_SECONDS, gtX);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

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
            Logger.getLogger(SlamEstimatorTest.class.getSimpleName()).log(
                    Level.INFO, msg);

            // rotate random point with quaternions and check that filtered
            // quaternion is closer to ground truth than predicted quaternion
            final InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(),
                    randomizer.nextDouble());

            final Quaternion filteredQuaternion =
                    estimatorWithCalibration.getStateQuaternion();
            final Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            final Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            final Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            final Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            final Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            final boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                            predictedRotated.distanceTo(groundTruthRotated);

            // check that filtered position is closer to ground truth than
            // predicted position, and hence Kalman filter improves results
            final InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(),
                    estimatorWithCalibration.getStatePositionY(),
                    estimatorWithCalibration.getStatePositionZ());

            final InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            final InhomogeneousPoint3D groundTruthPos =
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            final boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) <
                            predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved || positionImproved) {
                numSuccess++;
            }
        }

        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);
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
