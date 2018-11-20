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
import org.junit.*;

import java.util.Arrays;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class SlamEstimatorTest implements BaseSlamEstimatorListener {
    
    private static final int TIMES = 50;
    private static final double ABSOLUTE_ERROR = 1e-8;
    
    //conversion from milliseconds to nanoseconds
    private static final int MILLIS_TO_NANOS = 1000000;
    
    //time between samples expressed in nanoseconds (a typical sensor in Android 
    //delivers a sample every 20ms)
    private static final int DELTA_NANOS = 20000000; //0.02 seconds
    private static final double DELTA_SECONDS = 0.02; //0.02 seconds;
    
    //gain below is only valid when measure variance is 1e-8 and process noise
    //variance is 1e-6. This is the gain component of the Kalman filter for 
    //those error covariances. This gain will change for other values
    private static final double VELOCITY_GAIN = 0.019991983014891204; //0.1923075055475187;
    
    private static final int REPEAT_TIMES = 5;
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
    
    public SlamEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() throws WrongSizeException {
        SlamEstimator estimator = new SlamEstimator();
        
        //check initial values
        assertNull(estimator.getListener());
        assertNull(estimator.getCalibrationData());
        assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
        assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
        assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
        assertArrayEquals(estimator.getStatePosition(), 
                new double[]{0.0,0.0,0.0}, 0.0);
        double[] position = new double[3];
        estimator.getStatePosition(position);
        assertArrayEquals(position, new double[]{0.0,0.0,0.0}, 0.0);
        
        assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
        assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
        assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
        assertArrayEquals(estimator.getStateVelocity(), 
                new double[]{0.0,0.0,0.0},0.0);
        double[] velocity = new double[3];
        estimator.getStateVelocity(velocity);
        assertArrayEquals(velocity, new double[]{0.0,0.0,0.0}, 0.0);
        
        assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
        assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
        assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
        assertArrayEquals(estimator.getStateAcceleration(), 
                new double[]{0.0,0.0,0.0}, 0.0);
        double[] acceleration = new double[3];
        estimator.getStateAcceleration(acceleration);
        assertArrayEquals(acceleration, new double[]{0.0,0.0,0.0}, 0.0);
        
        assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
        assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
        assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
        assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
        assertArrayEquals(estimator.getStateQuaternionArray(),
                new double[]{1.0,0.0,0.0,0.0}, 0.0);
        double[] quaternionArray = new double[4];
        estimator.getStateQuaternionArray(quaternionArray);
        assertArrayEquals(quaternionArray, new double[]{1.0,0.0,0.0,0.0}, 0.0);
        
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
                new double[]{0.0,0.0,0.0}, 0.0);
        double[] angularSpeed = new double[3];
        estimator.getStateAngularSpeed(angularSpeed);
        assertArrayEquals(angularSpeed, new double[]{0.0,0.0,0.0}, 0.0);
        
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
                new double[]{0.0,0.0,0.0}, 0.0);
        double[] accumAcc = new double[3];
        estimator.getAccumulatedAccelerationSample(accumAcc);
        assertArrayEquals(accumAcc, new double[]{0.0,0.0,0.0}, 0.0);
        
        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertArrayEquals(estimator.getAccumulatedAngularSpeedSample(),
                new double[]{0.0,0.0,0.0}, 0.0);
        double[] accumAngularSpeed = new double[3];
        estimator.getAccumulatedAngularSpeedSample(accumAngularSpeed);
        assertArrayEquals(accumAngularSpeed, new double[]{0.0,0.0,0.0}, 0.0);
        
        assertEquals(estimator.getMostRecentTimestampNanos(), -1);
        
        assertEquals(estimator.getPositionCovarianceMatrix(), 
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        KalmanFilter.DEFAULT_MEASUREMENT_NOISE_VARIANCE));
        assertNotNull(estimator.getStateCovariance());
    }
    
    @Test
    public void testGetSetListener() {
        SlamEstimator estimator = new SlamEstimator();
        
        //initial value
        assertNull(estimator.getListener());
        
        //set new value
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
    }
    
    @Test
    public void testGetSetCalibrationData() {                
        SlamEstimator estimator = new SlamEstimator();
        
        //initial value
        assertNull(estimator.getCalibrationData());
        
        //set new value
        SlamCalibrationData data = new SlamCalibrationData();
        estimator.setCalibrationData(data);
        
        //check correctness
        assertSame(estimator.getCalibrationData(), data);
    }

    @Test
    public void testResetPosition() {
        SlamEstimator estimator = new SlamEstimator();

        estimator.resetPosition();

        assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
        assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
        assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);

        assertEquals(estimator.getMostRecentTimestampNanos(), -1);
    }

    @Test
    public void testResetVelocity() {
        SlamEstimator estimator = new SlamEstimator();

        estimator.resetVelocity();

        assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
        assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
        assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);

        assertEquals(estimator.getMostRecentTimestampNanos(), -1);
    }

    @Test
    public void testResetPositionAndVelocity() {
        SlamEstimator estimator = new SlamEstimator();

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
        SlamEstimator estimator = new SlamEstimator();

        estimator.resetAcceleration();

        assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
        assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
        assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);

        assertEquals(estimator.getMostRecentTimestampNanos(), -1);
    }

    @Test
    public void testResetOrientation() {
        SlamEstimator estimator = new SlamEstimator();

        estimator.resetOrientation();

        assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
        assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
        assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
        assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);

        assertEquals(estimator.getMostRecentTimestampNanos(), -1);
    }

    @Test
    public void testResetAngularSpeed() {
        SlamEstimator estimator = new SlamEstimator();

        estimator.resetAngularSpeed();

        assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
        assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
        assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);

        assertEquals(estimator.getMostRecentTimestampNanos(), -1);
    }

    @Test
    public void testReset() {
        SlamEstimator estimator = new SlamEstimator();
        
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
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(estimator.getStatePositionX(), 0.0, 0.0);
        
        double value = randomizer.nextDouble();
        estimator.mStatePositionX = value;
        
        //check correctness
        assertEquals(estimator.getStatePositionX(), value, 0.0);
    }
    
    @Test
    public void testGetStatePositionY() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(estimator.getStatePositionY(), 0.0, 0.0);
        
        double value = randomizer.nextDouble();
        estimator.mStatePositionY = value;
        
        //check correctness
        assertEquals(estimator.getStatePositionY(), value, 0.0);
    }
    
    @Test
    public void testGetStatePositionZ() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(estimator.getStatePositionZ(), 0.0, 0.0);
        
        double value = randomizer.nextDouble();
        estimator.mStatePositionZ = value;
        
        //check correctness
        assertEquals(estimator.getStatePositionZ(), value, 0.0);
    }
    
    @Test
    public void testGetStatePosition() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double valueX = randomizer.nextDouble();
        double valueY = randomizer.nextDouble();
        double valueZ = randomizer.nextDouble();
        estimator.mStatePositionX = valueX;
        estimator.mStatePositionY = valueY;
        estimator.mStatePositionZ = valueZ;
        
        //check correctness
        assertArrayEquals(estimator.getStatePosition(), 
                new double[]{valueX,valueY,valueZ}, 0.0);
        
        double[] position = new double[3];
        estimator.getStatePosition(position);
        
        //check correctness
        assertArrayEquals(position, new double[]{valueX,valueY,valueZ}, 0.0);
        
        //Force IllegalArgumentException
        double[] wrong = new double[4];
        try {
            estimator.getStatePosition(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetStateVelocityX() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(estimator.getStateVelocityX(), 0.0, 0.0);
        
        double value = randomizer.nextDouble();
        estimator.mStateVelocityX = value;
        
        //check correctness
        assertEquals(estimator.getStateVelocityX(), value, 0.0);        
    }
    
    @Test
    public void testGetStateVelocityY() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(estimator.getStateVelocityY(), 0.0, 0.0);
        
        double value = randomizer.nextDouble();
        estimator.mStateVelocityY = value;
        
        //check correctness
        assertEquals(estimator.getStateVelocityY(), value, 0.0);
    }
    
    @Test
    public void testGetStateVelocityZ() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(estimator.getStateVelocityZ(), 0.0, 0.0);
        
        double value = randomizer.nextDouble();
        estimator.mStateVelocityZ = value;
        
        //check correctness
        assertEquals(estimator.getStateVelocityZ(), value, 0.0);
    }
    
    @Test
    public void testGetStateVelocity() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double valueX = randomizer.nextDouble();
        double valueY = randomizer.nextDouble();
        double valueZ = randomizer.nextDouble();
        estimator.mStateVelocityX = valueX;
        estimator.mStateVelocityY = valueY;
        estimator.mStateVelocityZ = valueZ;
        
        //check correctness
        assertArrayEquals(estimator.getStateVelocity(),
                new double[]{valueX, valueY, valueZ}, 0.0);
        
        double[] velocity = new double[3];
        estimator.getStateVelocity(velocity);
        
        //check correctness
        assertArrayEquals(velocity, new double[]{valueX,valueY,valueZ}, 0.0);        
        
        //Force IllegalArgumentException
        double[] wrong = new double[4];
        try {
            estimator.getStateVelocity(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetStateAccelerationX() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(estimator.getStateAccelerationX(), 0.0, 0.0);
        
        double value = randomizer.nextDouble();
        estimator.mStateAccelerationX = value;
        
        //check correctness
        assertEquals(estimator.getStateAccelerationX(), value, 0.0);    
    }
    
    @Test
    public void testGetStateAccelerationY() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(estimator.getStateAccelerationY(), 0.0, 0.0);
        
        double value = randomizer.nextDouble();
        estimator.mStateAccelerationY = value;
        
        //check correctness
        assertEquals(estimator.getStateAccelerationY(), value, 0.0);
    }
    
    @Test
    public void testGetStateAccelerationZ() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(estimator.getStateAccelerationZ(), 0.0, 0.0);
        
        double value = randomizer.nextDouble();
        estimator.mStateAccelerationZ = value;
        
        //check correctness
        assertEquals(estimator.getStateAccelerationZ(), value, 0.0);
    }
    
    @Test
    public void testGetStateAcceleration() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double valueX = randomizer.nextDouble();
        double valueY = randomizer.nextDouble();
        double valueZ = randomizer.nextDouble();
        estimator.mStateAccelerationX = valueX;
        estimator.mStateAccelerationY = valueY;
        estimator.mStateAccelerationZ = valueZ;
        
        //check correctness
        assertArrayEquals(estimator.getStateAcceleration(),
                new double[]{valueX, valueY, valueZ}, 0.0);
        
        double[] acceleration = new double[3];
        estimator.getStateAcceleration(acceleration);
        
        //check correctness
        assertArrayEquals(acceleration, new double[]{valueX,valueY,valueZ}, 
                0.0);        
        
        //Force IllegalArgumentException
        double[] wrong = new double[4];
        try {
            estimator.getStateAcceleration(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetStateQuaternionA() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial value
        assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
        
        //set new value
        double value = randomizer.nextDouble();
        estimator.mStateQuaternionA = value;
        
        //check correctness
        assertEquals(estimator.getStateQuaternionA(), value, 0.0);
    }
    
    @Test
    public void testGetStateQuaternionB() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial value
        assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
        
        //set new value
        double value = randomizer.nextDouble();
        estimator.mStateQuaternionB = value;
        
        //check correctness
        assertEquals(estimator.getStateQuaternionB(), value, 0.0);
    }
    
    @Test
    public void testGetStateQuaternionC() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial value
        assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
        
        //set new value
        double value = randomizer.nextDouble();
        estimator.mStateQuaternionC = value;
        
        //check correctness
        assertEquals(estimator.getStateQuaternionC(), value, 0.0);
    }
    
    @Test
    public void testGetStateQuaternionD() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial value
        assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
        
        //set new value
        double value = randomizer.nextDouble();
        estimator.mStateQuaternionD = value;
        
        //check correctness
        assertEquals(estimator.getStateQuaternionD(), value, 0.0);
    }
    
    @Test
    public void testGetStateQuaternionArray() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial value
        assertArrayEquals(estimator.getStateQuaternionArray(), 
                new double[]{1.0,0.0,0.0,0.0}, 0.0);
        
        //set new values
        double valueA = randomizer.nextDouble();
        double valueB = randomizer.nextDouble();
        double valueC = randomizer.nextDouble();
        double valueD = randomizer.nextDouble();
        estimator.mStateQuaternionA = valueA;
        estimator.mStateQuaternionB = valueB;
        estimator.mStateQuaternionC = valueC;
        estimator.mStateQuaternionD = valueD;
        
        //check correctness
        assertArrayEquals(estimator.getStateQuaternionArray(),
                new double[]{valueA, valueB, valueC, valueD}, 0.0);
        
        double[] quaternion = new double[4];
        estimator.getStateQuaternionArray(quaternion);
        
        assertArrayEquals(quaternion, 
                new double[]{valueA, valueB, valueC, valueD}, 0.0);
        
        //Force IllegalArgumentException
        double[] wrong = new double[5];
        try {
            estimator.getStateQuaternionArray(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetStateQuaternion() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial value
        Quaternion q = estimator.getStateQuaternion();
        
        assertEquals(q.getA(), 1.0, 0.0);
        assertEquals(q.getB(), 0.0, 0.0);
        assertEquals(q.getC(), 0.0, 0.0);
        assertEquals(q.getD(), 0.0, 0.0);
        
        //set new values
        double valueA = randomizer.nextDouble();
        double valueB = randomizer.nextDouble();
        double valueC = randomizer.nextDouble();
        double valueD = randomizer.nextDouble();
        estimator.mStateQuaternionA = valueA;
        estimator.mStateQuaternionB = valueB;
        estimator.mStateQuaternionC = valueC;
        estimator.mStateQuaternionD = valueD;
        
        //check correctness
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
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial value
        assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
        
        //set new value
        double value = randomizer.nextDouble();
        estimator.mStateAngularSpeedX = value;
        
        //check correctness
        assertEquals(estimator.getStateAngularSpeedX(), value, 0.0);
    }
    
    @Test
    public void testGetStateAngularSpeedY() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial value
        assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
        
        //set new value
        double value = randomizer.nextDouble();
        estimator.mStateAngularSpeedY = value;
        
        //check correctness
        assertEquals(estimator.getStateAngularSpeedY(), value, 0.0);
    }
    
    @Test
    public void testGetStateAngularSpeedZ() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial value
        assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
        
        //set new value
        double value = randomizer.nextDouble();
        estimator.mStateAngularSpeedZ = value;
        
        //check correctness
        assertEquals(estimator.getStateAngularSpeedZ(), value, 0.0);
    }
    
    @Test
    public void testGetStateAngularSpeed() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial value
        assertArrayEquals(estimator.getStateAngularSpeed(), 
                new double[]{0.0,0.0,0.0}, 0.0);
        
        //set new value
        double valueX = randomizer.nextDouble();
        double valueY = randomizer.nextDouble();
        double valueZ = randomizer.nextDouble();
        estimator.mStateAngularSpeedX = valueX;
        estimator.mStateAngularSpeedY = valueY;
        estimator.mStateAngularSpeedZ = valueZ;
        
        //check correctness
        assertArrayEquals(estimator.getStateAngularSpeed(),
                new double[]{valueX,valueY,valueZ}, 0.0);
        
        double[] angularSpeed = new double[3];
        estimator.getStateAngularSpeed(angularSpeed);
        
        //check correctness
        assertArrayEquals(angularSpeed, new double[]{valueX,valueY,valueZ}, 
                0.0);
        
        //Force IllegalArgumentException
        double[] wrong = new double[4];
        try {
            estimator.getStateAngularSpeed(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testHasError() {
        SlamEstimator estimator = new SlamEstimator();
        
        assertFalse(estimator.hasError());
        
        estimator.mError = true;
        
        assertTrue(estimator.hasError());
    }
    
    @Test
    public void testIsAccumulationEnabled() {
        SlamEstimator estimator = new SlamEstimator();
        
        assertTrue(estimator.isAccumulationEnabled());
        
        estimator.setAccumulationEnabled(false);
        
        assertFalse(estimator.isAccumulationEnabled());
    }
    
    @Test
    public void testGetAccelerometerTimestampNanos() {
        SlamEstimator estimator = new SlamEstimator();
        
        assertEquals(estimator.getAccelerometerTimestampNanos(), -1);
        
        //set new vlaue
        estimator.mAccelerometerTimestampNanos = 1000;
        
        //check correctness
        assertEquals(estimator.getAccelerometerTimestampNanos(), 1000);
    }
    
    @Test
    public void testGetGyroscopeTimestampNanos() {
        SlamEstimator estimator = new SlamEstimator();
        
        assertEquals(estimator.getGyroscopeTimestampNanos(), -1);
        
        //set new value
        estimator.mGyroscopeTimestampNanos = 2000;
        
        //check correctness
        assertEquals(estimator.getGyroscopeTimestampNanos(), 2000);
    }
    
    @Test
    public void testGetAccumulatedAccelerometerSamplesAndIsAccelerometerSampleReceived() {
        SlamEstimator estimator = new SlamEstimator();
        
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 0);
        assertFalse(estimator.isAccelerometerSampleReceived());
        
        //set new value
        estimator.mAccumulatedAccelerometerSamples = 10;
        
        //check correctness
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 10);
        assertTrue(estimator.isAccelerometerSampleReceived());
    }
    
    @Test
    public void testGetAccumulatedGyroscopeSamplesAndIsGyroscopeSampleReceived() {
        SlamEstimator estimator = new SlamEstimator();
        
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(estimator.isGyroscopeSampleReceived());
        
        //set new value
        estimator.mAccumulatedGyroscopeSamples = 20;
        
        //check correctness
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 20);
        assertTrue(estimator.isGyroscopeSampleReceived());
    }
    
    @Test
    public void testIsFullSampleAvailable() {
        SlamEstimator estimator = new SlamEstimator();
        
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 0);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(estimator.isAccelerometerSampleReceived());
        assertFalse(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());
        
        estimator.mAccumulatedAccelerometerSamples = 1;
        
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 1);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 0);
        assertTrue(estimator.isAccelerometerSampleReceived());
        assertFalse(estimator.isGyroscopeSampleReceived());
        assertFalse(estimator.isFullSampleAvailable());
        
        estimator.mAccumulatedGyroscopeSamples = 2;
        
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 1);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 2);
        assertTrue(estimator.isAccelerometerSampleReceived());
        assertTrue(estimator.isGyroscopeSampleReceived());
        assertTrue(estimator.isFullSampleAvailable());
    }
    
    @Test
    public void testGetAccumulatedAccelerationSampleX() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        assertEquals(estimator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        
        double value = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleX = value;
        
        //check correctness
        assertEquals(estimator.getAccumulatedAccelerationSampleX(), value, 0.0);
    }
    
    @Test
    public void testGetAccumulatedAccelerationSampleY() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        assertEquals(estimator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        
        double value = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleY = value;
        
        //check correctness
        assertEquals(estimator.getAccumulatedAccelerationSampleY(), value, 0.0);
    }
    
    @Test
    public void testGetAccumulatedAccelerationSampleZ() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        
        double value = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleZ = value;
        
        //check correctness
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(), value, 0.0);
    }
    
    @Test
    public void testGetAccumulatedAccelerationSample() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        assertArrayEquals(estimator.getAccumulatedAccelerationSample(),
                new double[]{0.0,0.0,0.0}, 0.0);
        
        //set new values
        double valueX = randomizer.nextDouble();
        double valueY = randomizer.nextDouble();
        double valueZ = randomizer.nextDouble();
        estimator.mAccumulatedAccelerationSampleX = valueX;
        estimator.mAccumulatedAccelerationSampleY = valueY;
        estimator.mAccumulatedAccelerationSampleZ = valueZ;
        
        //check correctness
        assertArrayEquals(estimator.getAccumulatedAccelerationSample(),
                new double[]{valueX,valueY,valueZ}, 0.0);
        
        double[] acceleration = new double[3];
        estimator.getAccumulatedAccelerationSample(acceleration);
        
        //check correctness
        assertArrayEquals(acceleration, new double[]{valueX,valueY,valueZ}, 
                0.0);
        
        //Force IllegalArgumentException
        double[] wrong = new double[4];
        try {
            estimator.getAccumulatedAccelerationSample(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetAccumulatedAngularSpeedSampleX() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        
        //set new value
        double value = randomizer.nextDouble();
        estimator.mAccumulatedAngularSpeedSampleX = value;
        
        //check correctness
        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(), value, 0.0);
    }
    
    @Test
    public void testGetAccumulatedAngularSpeedSampleY() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        
        //set new value
        double value = randomizer.nextDouble();
        estimator.mAccumulatedAngularSpeedSampleY = value;
        
        //check correctness
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(), value, 0.0);
    }
    
    @Test
    public void testGetAccumulatedAngularSpeedSampleZ() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        
        //set new value
        double value = randomizer.nextDouble();
        estimator.mAccumulatedAngularSpeedSampleZ = value;
        
        //check correctness
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(), value, 0.0);
    }
    
    @Test
    public void testGetAccumulatedAngularSpeedSample() {
        SlamEstimator estimator = new SlamEstimator();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        assertArrayEquals(estimator.getAccumulatedAngularSpeedSample(), 
                new double[]{0.0,0.0,0.0}, 0.0);
        
        //set new value
        double valueX = randomizer.nextDouble();
        double valueY = randomizer.nextDouble();
        double valueZ = randomizer.nextDouble();
        estimator.mAccumulatedAngularSpeedSampleX = valueX;
        estimator.mAccumulatedAngularSpeedSampleY = valueY;
        estimator.mAccumulatedAngularSpeedSampleZ = valueZ;
        
        //check correctness
        assertArrayEquals(estimator.getAccumulatedAngularSpeedSample(),
                new double[]{valueX,valueY,valueZ}, 0.0);
        
        double[] speed = new double[3];
        estimator.getAccumulatedAngularSpeedSample(speed);
        
        //check correctness
        assertArrayEquals(speed, new double[]{valueX,valueY,valueZ}, 0.0);
        
        //Force IllegalArgumentException
        double[] wrong = new double[4];
        try {
            estimator.getAccumulatedAngularSpeedSample(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testUpdateAccelerometerSampleWithAccumulationDisabled() {
        SlamEstimator estimator = new SlamEstimator();
        estimator.setAccumulationEnabled(false);
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        long timestamp = System.currentTimeMillis();
        float accelerationX = randomizer.nextFloat();
        float accelerationY = randomizer.nextFloat();
        float accelerationZ = randomizer.nextFloat();
        
        //check initial values
        assertEquals(estimator.getAccelerometerTimestampNanos(), -1);
        assertEquals(estimator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 0);        
        assertFalse(estimator.isFullSampleAvailable());
        
        estimator.updateAccelerometerSample(timestamp, accelerationX, 
                accelerationY, accelerationZ);
        
        //check correctness
        assertEquals(estimator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(estimator.getAccumulatedAccelerationSampleX(), 
                accelerationX, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleY(),
                accelerationY, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(),
                accelerationZ, 0.0);
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 1);
        assertFalse(estimator.isFullSampleAvailable());
        
        //test again but using an array
        float[] acceleration = new float[3];
        randomizer.fill(acceleration);
        timestamp += 1000;
        
        estimator.updateAccelerometerSample(timestamp, acceleration);
        
        //check correctness
        assertEquals(estimator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(estimator.getAccumulatedAccelerationSampleX(), 
                acceleration[0], 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleY(),
                acceleration[1], 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(),
                acceleration[2], 0.0);
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 2);
        assertFalse(estimator.isFullSampleAvailable());
        
        //Force IllegalArgumentException
        float[] wrong = new float[4];
        try {
            estimator.updateAccelerometerSample(timestamp, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testUpdateAccelerometerSampleWithAccumulationEnabled() {
        SlamEstimator estimator = new SlamEstimator();
        estimator.setAccumulationEnabled(true);
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(estimator.getAccelerometerTimestampNanos(), -1);
        assertEquals(estimator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAccelerometerSamples(), 0);        
        assertFalse(estimator.isFullSampleAvailable());
        
        //update with several samples
        long timestamp = System.currentTimeMillis();
        float accelerationX, accelerationY, accelerationZ;
        double avgAccelerationX = 0.0, avgAccelerationY = 0.0, 
                avgAccelerationZ = 0.0;
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
        
        //check correctness
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
        SlamEstimator estimator = new SlamEstimator();
        estimator.setAccumulationEnabled(false);
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        long timestamp = System.currentTimeMillis();
        float angularSpeedX = randomizer.nextFloat();
        float angularSpeedY = randomizer.nextFloat();
        float angularSpeedZ = randomizer.nextFloat();
        
        //check initial values
        assertEquals(estimator.getGyroscopeTimestampNanos(), -1);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(estimator.isFullSampleAvailable());
        
        estimator.updateGyroscopeSample(timestamp, angularSpeedX, angularSpeedY, 
                angularSpeedZ);
        
        //check correctness
        assertEquals(estimator.getGyroscopeTimestampNanos(), timestamp);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(), 
                angularSpeedX, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(), 
                angularSpeedY, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(), 
                angularSpeedZ, 0.0);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 1);
        assertFalse(estimator.isFullSampleAvailable());
        
        //test again but using an array
        float[] angularSpeed = new float[3];
        randomizer.fill(angularSpeed);
        timestamp += 100;
        
        estimator.updateGyroscopeSample(timestamp, angularSpeed);
        
        //check correctness
        assertEquals(estimator.getGyroscopeTimestampNanos(), timestamp);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(), 
                angularSpeed[0], 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(), 
                angularSpeed[1], 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(),
                angularSpeed[2], 0.0);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 2);
        assertFalse(estimator.isFullSampleAvailable());
        
        //Force IllegalArgumentException
        float[] wrong = new float[4];
        try {
            estimator.updateGyroscopeSample(timestamp, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testUpdateGyroscopeSampleWithAccumulationEnabled() {
        SlamEstimator estimator = new SlamEstimator();
        estimator.setAccumulationEnabled(true);
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(estimator.getGyroscopeTimestampNanos(), -1);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertEquals(estimator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(estimator.isFullSampleAvailable());
        
        //update with several samples
        long timestamp = System.currentTimeMillis();
        float angularSpeedX, angularSpeedY, angularSpeedZ;
        double avgAngularSpeedX = 0.0, avgAngularSpeedY = 0.0, 
                avgAngularSpeedZ = 0.0;
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
        
        //check correctness
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
    public void testGetMostRecentTimestampNanos() {
        SlamEstimator estimator = new SlamEstimator();
        
        long timestamp = System.currentTimeMillis();
        estimator.mAccelerometerTimestampNanos = timestamp;
        estimator.mGyroscopeTimestampNanos = timestamp + 1000;
        
        assertEquals(estimator.getMostRecentTimestampNanos(), timestamp + 1000);
        
        estimator.mAccelerometerTimestampNanos = timestamp + 2000;
        
        assertEquals(estimator.getMostRecentTimestampNanos(), timestamp + 2000);
    }
    
    @Test
    public void testCorrectWithPositionMeasureCoordinatesAndCovariance() 
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();
            
            SlamEstimator estimator = new SlamEstimator();
            estimator.setListener(this);
        
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            double positionX = randomizer.nextDouble();
            double positionY = randomizer.nextDouble();
            double positionZ = randomizer.nextDouble();
        
            Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);
        
            //check initial value
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
        
        
            //predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
        
            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0); 
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);            
        
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
        
            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);            
                            
            //correct has no effect until we predict again
            estimator.correctWithPositionMeasure(
                    positionX, positionY, positionZ, positionCovariance);
        
            //check correctness
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

            //predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
        
            //check correctness
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
            
        
            //correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(
                    positionX, positionY, positionZ, positionCovariance);   
                        
            //expected velocities
            double vx = VELOCITY_GAIN * positionX;
            double vy = VELOCITY_GAIN * positionY;
            double vz = VELOCITY_GAIN * positionZ;
        
            //check correctness
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
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());  
            assertEquals(correctWithPositionMeasure, 1);
            assertEquals(correctedWithPositionMeasure, 1);

        
            //Force IllegalArgumentException (wrong covariance matrix size)
            Matrix wrong = new Matrix(4,4);
            try {
                estimator.correctWithPositionMeasure(
                        positionX, positionY, positionZ, wrong);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
        }
    }
    
    @Test
    public void testCorrectWithPositionMeasureArrayAndCovariance() 
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();
            
            SlamEstimator estimator = new SlamEstimator();
            estimator.setListener(this);
        
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            double positionX = randomizer.nextDouble();
            double positionY = randomizer.nextDouble();
            double positionZ = randomizer.nextDouble();
            double[] position = new double[]{ positionX, positionY, positionZ };
        
            Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);
        
            //check initial value
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
        
            //predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
        
            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);

        
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
        
            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1); 
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);            

        
            //correct has no effect until we predict again
            estimator.correctWithPositionMeasure(position, positionCovariance);
        
            //check correctness
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
        
            //predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
        
            //check correctness
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

            
            //correct providing a certain position (it is not ignored this time
            estimator.correctWithPositionMeasure(position, positionCovariance);
        
            //expected velocities
            double vx = VELOCITY_GAIN * positionX;
            double vy = VELOCITY_GAIN * positionY;
            double vz = VELOCITY_GAIN * positionZ;
        
            //check correctness
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
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());
            assertEquals(correctWithPositionMeasure, 1);
            assertEquals(correctedWithPositionMeasure, 1);

            
            //Force IllegalArgumentException 
            //(wrong covariance matrix size)
            Matrix wrongCov = new Matrix(4,4);
            try {
                estimator.correctWithPositionMeasure(position, wrongCov);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
        
            //(wrong position length)
            double[] wrongPos = new double[4];
            try {
                estimator.correctWithPositionMeasure(wrongPos, positionCovariance);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
        }
    }
    
    @Test
    public void testCorrectWithPositionMeasurePointAndCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();
            
            SlamEstimator estimator = new SlamEstimator();
            estimator.setListener(this);
        
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            double positionX = randomizer.nextDouble();
            double positionY = randomizer.nextDouble();
            double positionZ = randomizer.nextDouble();
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(positionX, 
                    positionY, positionZ);
        
            Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);
        
            //check initial value
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
        
        
            //predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
        
            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);
            
        
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
        
            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1); 
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);            

            
            //correct has no effect until we predict again
            estimator.correctWithPositionMeasure(position, positionCovariance);
        
            //check correctness
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
        
            //predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
        
            //check correctness
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

            
            //correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(position, positionCovariance);

            //expected velocities
            double vx = VELOCITY_GAIN * positionX;
            double vy = VELOCITY_GAIN * positionY;
            double vz = VELOCITY_GAIN * positionZ;

            //check correctness
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
            assertEquals(estimator.getStateQuaternionA(), 1.0, 0.0);
            assertEquals(estimator.getStateQuaternionB(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionC(), 0.0, 0.0);
            assertEquals(estimator.getStateQuaternionD(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedX(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedY(), 0.0, 0.0);
            assertEquals(estimator.getStateAngularSpeedZ(), 0.0, 0.0);
            assertFalse(estimator.hasError());  
            assertEquals(correctWithPositionMeasure, 1);
            assertEquals(correctedWithPositionMeasure, 1);

            
            //Force IllegalArgumentException (wrong covariance matrix size)
            Matrix wrong = new Matrix(4,4);
            try {
                estimator.correctWithPositionMeasure(
                        positionX, positionY, positionZ, wrong);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
        }
    }
    
    @Test
    public void testGetSetPositionCovarianceMatrix() throws AlgebraException {
        
        SlamEstimator estimator = new SlamEstimator();
        
        //check initial value
        assertEquals(estimator.getPositionCovarianceMatrix(), 
                Matrix.identity(3, 3).multiplyByScalarAndReturnNew(
                        KalmanFilter.DEFAULT_MEASUREMENT_NOISE_VARIANCE));
        
        //set new value
        Matrix positionCovariance = Matrix.identity(3, 3).
                multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);        
        
        estimator.setPositionCovarianceMatrix(positionCovariance);
        
        //check correctness
        assertEquals(estimator.getPositionCovarianceMatrix(), 
                positionCovariance);
        
        //Force IllegalArgumentException
        Matrix wrong = new Matrix(4,4);
        try {
            estimator.setPositionCovarianceMatrix(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testCorrectWithPositionMeasureCoordinatesWithoutCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();
            
            SlamEstimator estimator = new SlamEstimator();
            estimator.setListener(this);
        
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            double positionX = randomizer.nextDouble();
            double positionY = randomizer.nextDouble();
            double positionZ = randomizer.nextDouble();
        
            Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);
            estimator.setPositionCovarianceMatrix(positionCovariance);
        
            //check initial value
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
        
        
            //predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0); 
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);            

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);            

            //correct has no effect until we predict again
            estimator.correctWithPositionMeasure(positionX, positionY, positionZ);

            //check correctness
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

            //predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            //check correctness
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

            //correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(positionX, positionY, positionZ);   

            //expected velocities
            double vx = VELOCITY_GAIN * positionX;
            double vy = VELOCITY_GAIN * positionY;
            double vz = VELOCITY_GAIN * positionZ;

            //check correctness
            assertEquals(estimator.getStatePositionX(), positionX, ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionY(), positionY, ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionZ(), positionZ, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityX(), vx, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityY(), vy, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityZ(), vz, ABSOLUTE_ERROR);
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
            assertEquals(correctWithPositionMeasure, 1);
            assertEquals(correctedWithPositionMeasure, 1);            
        }
    }
    
    @Test
    public void testCorrectWithPositionMeasureArrayWithoutCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();
            
            SlamEstimator estimator = new SlamEstimator();
            estimator.setListener(this);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double positionX = randomizer.nextDouble();
            double positionY = randomizer.nextDouble();
            double positionZ = randomizer.nextDouble();
            double[] position = new double[]{ positionX, positionY, positionZ };

            Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);
            estimator.setPositionCovarianceMatrix(positionCovariance);

            //check initial value
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


            //predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0); 
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);                        

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);                        

            //correct has no effect until we predict again
            estimator.correctWithPositionMeasure(position);
        
            //check correctness
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

            //predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
        
            //check correctness
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

            
            //correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(position);   
        
            //expected velocities
            double vx = VELOCITY_GAIN * positionX;
            double vy = VELOCITY_GAIN * positionY;
            double vz = VELOCITY_GAIN * positionZ;

            //check correctness
            assertEquals(estimator.getStatePositionX(), positionX, ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionY(), positionY, ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionZ(), positionZ, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityX(), vx, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityY(), vy, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityZ(), vz, ABSOLUTE_ERROR);
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
            assertEquals(correctWithPositionMeasure, 1);
            assertEquals(correctedWithPositionMeasure, 1);
        
            //Force IllegalArgumentException
            double[] wrong = new double[4];
            try {
                estimator.correctWithPositionMeasure(wrong);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
        }
    }
    
    @Test
    public void testCorrectWithPositionMeasurePointWithoutCovariance()
            throws AlgebraException {
        for (int t = 0; t < REPEAT_TIMES; t++) {
            reset();
            
            SlamEstimator estimator = new SlamEstimator();
            estimator.setListener(this);

            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double positionX = randomizer.nextDouble();
            double positionY = randomizer.nextDouble();
            double positionZ = randomizer.nextDouble();
            InhomogeneousPoint3D position = new InhomogeneousPoint3D(positionX, 
                    positionY, positionZ);

            Matrix positionCovariance = Matrix.identity(3, 3).
                    multiplyByScalarAndReturnNew(ABSOLUTE_ERROR);
            estimator.setPositionCovarianceMatrix(positionCovariance);
        
            //check initial value
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
        
        
            //predict
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertTrue(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 0);
            assertEquals(fullSampleProcessed, 0); 
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);            

            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);

            assertFalse(estimator.isAccelerometerSampleReceived());
            assertFalse(estimator.isGyroscopeSampleReceived());
            assertEquals(fullSampleReceived, 1);
            assertEquals(fullSampleProcessed, 1);
            assertEquals(correctWithPositionMeasure, 0);
            assertEquals(correctedWithPositionMeasure, 0);            

            //correct has no effect until we predict again
            estimator.correctWithPositionMeasure(position);
        
            //check correctness
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

            //predict again
            timestamp += DELTA_NANOS;
            estimator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
            estimator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
        
            //check correctness
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

            //correct providing a certain position (it is not ignored this time)
            estimator.correctWithPositionMeasure(position);   
        
            //expected velocities
            double vx = VELOCITY_GAIN * positionX;
            double vy = VELOCITY_GAIN * positionY;
            double vz = VELOCITY_GAIN * positionZ;

            //check correctness
            assertEquals(estimator.getStatePositionX(), positionX, ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionY(), positionY, ABSOLUTE_ERROR);
            assertEquals(estimator.getStatePositionZ(), positionZ, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityX(), vx, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityY(), vy, ABSOLUTE_ERROR);
            assertEquals(estimator.getStateVelocityZ(), vz, ABSOLUTE_ERROR);
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
            assertEquals(correctWithPositionMeasure, 1);
            assertEquals(correctedWithPositionMeasure, 1);            
        }
    }
    
    @Test
    public void testCreateCalibrator() {
        //noinspection all
        assertTrue(SlamEstimator.createCalibrator() instanceof SlamCalibrator);
    }
    
    @Test
    public void testPredictionNoMotionWithNoise() {
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double gtPositionX = 0.0, gtPositionY = 0.0, gtPositionZ = 0.0;
            double gtSpeedX = 0.0, gtSpeedY = 0.0, gtSpeedZ = 0.0;
            float gtAccelerationX = 0.0f, gtAccelerationY = 0.0f, 
                    gtAccelerationZ = 0.0f;
            float gtAngularSpeedX = 0.0f, gtAngularSpeedY = 0.0f, 
                    gtAngularSpeedZ = 0.0f;
            float gtQuaternionA = 1.0f, gtQuaternionB = 0.0f, 
                    gtQuaternionC = 0.0f, gtQuaternionD = 0.0f;

            SlamEstimator estimator = new SlamEstimator();
        
            GaussianRandomizer accelerationRandomizer = 
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer = 
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0, 
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0, 
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;            
            double[] x = new double[16];
            double[] u = new double[9];
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
        
            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];

            //set initial state
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

                estimator.updateAccelerometerSample(timestamp, 
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i == 0) {
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
                    
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

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
                    
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
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

            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = estimator.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            boolean rotationImproved = 
                    filteredRotated.distanceTo(groundTruthRotated) < 
                    predictedRotated.distanceTo(groundTruthRotated);

            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), 
                    estimator.getStatePositionY(), 
                    estimator.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = 
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            boolean positionImproved = 
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
        for (int t = 0; t < REPEAT_TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double gtPositionX = 0.0, gtPositionY = 0.0, gtPositionZ = 0.0;
            double gtSpeedX = randomizer.nextDouble();
            double gtSpeedY = randomizer.nextDouble();
            double gtSpeedZ = randomizer.nextDouble();
            float gtAccelerationX = 0.0f, gtAccelerationY = 0.0f, 
                    gtAccelerationZ = 0.0f;
            float gtAngularSpeedX = 0.0f, gtAngularSpeedY = 0.0f, 
                    gtAngularSpeedZ = 0.0f;
            float gtQuaternionA = 1.0f, gtQuaternionB = 0.0f, 
                    gtQuaternionC = 0.0f, gtQuaternionD = 0.0f;

            SlamEstimator estimator = new SlamEstimator();
        
            GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0,
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0, 
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;            
            double[] x = new double[16];
            double[] u = new double[9];
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

            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];

            //set initial state
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

                estimator.updateAccelerometerSample(timestamp, accelerationX, 
                        accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i == 0) {
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
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

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
                    
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
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

            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = estimator.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), 
                    estimator.getStatePositionY(), 
                    estimator.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = 
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            boolean positionImproved =
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
        for (int t = 0; t < REPEAT_TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double gtPositionX = 0.0, gtPositionY = 0.0, gtPositionZ = 0.0;
            double gtSpeedX = 0.0, gtSpeedY = 0.0, gtSpeedZ = 0.0;
            float gtAccelerationX = randomizer.nextFloat();
            float gtAccelerationY = randomizer.nextFloat();
            float gtAccelerationZ = randomizer.nextFloat();
            float gtAngularSpeedX = 0.0f, gtAngularSpeedY = 0.0f, 
                    gtAngularSpeedZ = 0.0f;
            float gtQuaternionA = 1.0f, gtQuaternionB = 0.0f, 
                    gtQuaternionC = 0.0f, gtQuaternionD = 0.0f;

            SlamEstimator estimator = new SlamEstimator();

            GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
        
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0,
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0,
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;            
            double[] x = new double[16];
            double[] u = new double[9];
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
        
            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];

            //set initial state
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
            
                estimator.updateAccelerometerSample(timestamp, accelerationX, 
                        accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i == 0) {
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
                    
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;
                
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
                    
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
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
        
            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = estimator.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            assertTrue(filteredRotated.distanceTo(groundTruthRotated) <=
                    predictedRotated.distanceTo(groundTruthRotated));
        
            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), 
                    estimator.getStatePositionY(), 
                    estimator.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(
                    gtX[0], gtX[1], gtX[2]);

            assertTrue(filteredPos.distanceTo(groundTruthPos) < 
                    predictedPos.distanceTo(groundTruthPos));
        }
    }    
    
    @Test
    public void testPredictionRotationOnlyWithNoise() {
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            double gtPositionX = 0.0, gtPositionY = 0.0, gtPositionZ = 0.0;
            double gtSpeedX = 0.0, gtSpeedY = 0.0, gtSpeedZ = 0.0;
            float gtAccelerationX = 0.0f, gtAccelerationY = 0.0f,
                    gtAccelerationZ = 0.0f;
            float gtAngularSpeedX = randomizer.nextFloat();
            float gtAngularSpeedY = randomizer.nextFloat();
            float gtAngularSpeedZ = randomizer.nextFloat();
            float gtQuaternionA = 1.0f, gtQuaternionB = 0.0f, 
                    gtQuaternionC = 0.0f, gtQuaternionD = 0.0f;

            SlamEstimator estimator = new SlamEstimator();

            GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
        
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX , accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0,
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0,
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;            
            double[] x = new double[16];
            double[] u = new double[9];
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
        
            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];

            //set initial state
            estimator.reset(gtPositionX, gtPositionY, gtPositionZ, 
                    gtSpeedX, gtSpeedY, gtSpeedZ, 
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
            
                estimator.updateAccelerometerSample(timestamp, accelerationX, 
                        accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i == 0) {
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
                    
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;
                
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
                    
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a constant 
                    //speed movement
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
        
            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = estimator.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            boolean rotationImproved = 
                    filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);
        
            //check that filtered position is closer to ground truth than predicted 
            //position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), 
                    estimator.getStatePositionY(), 
                    estimator.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], 
                    gtX[1], gtX[2]);

            boolean positionImproved = 
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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double gtPositionX = 0.0, gtPositionY = 0.0, gtPositionZ = 0.0;            
            double gtSpeedX = randomizer.nextDouble();
            double gtSpeedY = randomizer.nextDouble();
            double gtSpeedZ = randomizer.nextDouble();
            int period = N_PREDICTION_SAMPLES / 2;
            int offsetAccelerationX = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            int offsetAccelerationY = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            int offsetAccelerationZ = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            float amplitudeAccelerationX = randomizer.nextFloat();
            float amplitudeAccelerationY = randomizer.nextFloat();
            float amplitudeAccelerationZ = randomizer.nextFloat();
            float gtAccelerationX = (float)(amplitudeAccelerationX * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAccelerationX)));
            float gtAccelerationY = (float)(amplitudeAccelerationY * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAccelerationY)));
            float gtAccelerationZ = (float)(amplitudeAccelerationZ * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAccelerationZ)));
            float gtAngularSpeedX = 0.0f, gtAngularSpeedY = 0.0f, 
                    gtAngularSpeedZ = 0.0f;
            float gtQuaternionA = 1.0f, gtQuaternionB = 0.0f, 
                    gtQuaternionC = 0.0f, gtQuaternionD = 0.0f;

            SlamEstimator estimator = new SlamEstimator();

            GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
        
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0,
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0,
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;
            double[] x = new double[16];
            double[] u = new double[9];
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
            
            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];

            //set initial state
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
                
                gtAccelerationX = (float)(amplitudeAccelerationX * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAccelerationX)));
                gtAccelerationY = (float)(amplitudeAccelerationY * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAccelerationY)));
                gtAccelerationZ = (float)(amplitudeAccelerationZ * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAccelerationZ)));
                
                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;
            
                estimator.updateAccelerometerSample(timestamp, accelerationX, 
                        accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;
                
                if (i == 0) {
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
                    
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;
                    
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
                
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
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
        
            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = estimator.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            boolean rotationImproved = 
                    filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);
        
            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), 
                    estimator.getStatePositionY(), 
                    estimator.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(
                    gtX[0], gtX[1], gtX[2]);

            boolean positionImproved = 
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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            double gtPositionX = 0.0, gtPositionY = 0.0, gtPositionZ = 0.0;
            double gtSpeedX = 0.0, gtSpeedY = 0.0, gtSpeedZ = 0.0;
            float gtAccelerationX = 0.0f, gtAccelerationY = 0.0f,
                    gtAccelerationZ = 0.0f;
            int period = N_PREDICTION_SAMPLES / 2;
            int offsetAngularSpeedX = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            int offsetAngularSpeedY = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            int offsetAngularSpeedZ = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            float amplitudeAngularSpeedX = randomizer.nextFloat();
            float amplitudeAngularSpeedY = randomizer.nextFloat();
            float amplitudeAngularSpeedZ = randomizer.nextFloat();
            float gtAngularSpeedX = (float)(amplitudeAngularSpeedX * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAngularSpeedX)));
            float gtAngularSpeedY = (float)(amplitudeAngularSpeedY * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAngularSpeedY)));
            float gtAngularSpeedZ = (float)(amplitudeAngularSpeedZ * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAngularSpeedZ)));
            float gtQuaternionA = 1.0f, gtQuaternionB = 0.0f, 
                    gtQuaternionC = 0.0f, gtQuaternionD = 0.0f;

            SlamEstimator estimator = new SlamEstimator();

            GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
        
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0,
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0,
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;
            double[] x = new double[16];
            double[] u = new double[9];
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
            
            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];

            //set initial state
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

                gtAngularSpeedX = (float)(amplitudeAngularSpeedX * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAngularSpeedX)));
                gtAngularSpeedY = (float)(amplitudeAngularSpeedY * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAngularSpeedY)));
                gtAngularSpeedZ = (float)(amplitudeAngularSpeedZ * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAngularSpeedZ)));
                
                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;
            
                estimator.updateAccelerometerSample(timestamp, accelerationX, 
                        accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;
                
                if (i == 0) {
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
                    
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;
                    
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
                
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
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
        
            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = estimator.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            boolean rotationImproved = 
                    filteredRotated.distanceTo(groundTruthRotated) < 
                    predictedRotated.distanceTo(groundTruthRotated);
        
            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), 
                    estimator.getStatePositionY(), 
                    estimator.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(
                    gtX[0], gtX[1], gtX[2]);

            boolean positionImproved = 
                    filteredPos.distanceTo(groundTruthPos) < 
                    predictedPos.distanceTo(groundTruthPos);
            
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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double gtPositionX = randomizer.nextDouble();
            double gtPositionY = randomizer.nextDouble();
            double gtPositionZ = randomizer.nextDouble();
            double gtSpeedX = randomizer.nextDouble();
            double gtSpeedY = randomizer.nextDouble();
            double gtSpeedZ = randomizer.nextDouble();
            float gtAccelerationX = randomizer.nextFloat();
            float gtAccelerationY = randomizer.nextFloat();
            float gtAccelerationZ = randomizer.nextFloat();
            float gtAngularSpeedX = randomizer.nextFloat();
            float gtAngularSpeedY = randomizer.nextFloat();
            float gtAngularSpeedZ = randomizer.nextFloat();
            Quaternion gtQuaternion = new Quaternion(
                    randomizer.nextDouble(),randomizer.nextDouble(), 
                    randomizer.nextDouble());
            float gtQuaternionA = (float)gtQuaternion.getA();
            float gtQuaternionB = (float)gtQuaternion.getB();
            float gtQuaternionC = (float)gtQuaternion.getC();
            float gtQuaternionD = (float)gtQuaternion.getD();

            SlamEstimator estimator = new SlamEstimator();
        
            GaussianRandomizer accelerationRandomizer = 
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer = 
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0, 
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0, 
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;            
            double[] x = new double[16];
            double[] u = new double[9];
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
        
            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];

            //set initial state
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

                estimator.updateAccelerometerSample(timestamp, 
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i == 0) {
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
                    
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

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
                    
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
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

            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = estimator.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            boolean rotationImproved = 
                    filteredRotated.distanceTo(groundTruthRotated) < 
                    predictedRotated.distanceTo(groundTruthRotated);

            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), 
                    estimator.getStatePositionY(), 
                    estimator.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = 
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            boolean positionImproved = 
                    filteredPos.distanceTo(groundTruthPos) < 
                    predictedPos.distanceTo(groundTruthPos);
            
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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double gtPositionX = randomizer.nextDouble();
            double gtPositionY = randomizer.nextDouble();
            double gtPositionZ = randomizer.nextDouble();
            double gtSpeedX = randomizer.nextDouble();
            double gtSpeedY = randomizer.nextDouble();
            double gtSpeedZ = randomizer.nextDouble();
            int period = N_PREDICTION_SAMPLES / 2;
            int offsetAccelerationX = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            int offsetAccelerationY = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            int offsetAccelerationZ = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            float amplitudeAccelerationX = randomizer.nextFloat();
            float amplitudeAccelerationY = randomizer.nextFloat();
            float amplitudeAccelerationZ = randomizer.nextFloat();
            float gtAccelerationX = (float)(amplitudeAccelerationX * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAccelerationX)));
            float gtAccelerationY = (float)(amplitudeAccelerationY * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAccelerationY)));
            float gtAccelerationZ = (float)(amplitudeAccelerationZ * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAccelerationZ)));
            float gtAngularSpeedX = randomizer.nextFloat();
            float gtAngularSpeedY = randomizer.nextFloat();
            float gtAngularSpeedZ = randomizer.nextFloat();
            Quaternion gtQuaternion = new Quaternion(
                    randomizer.nextDouble(),randomizer.nextDouble(), 
                    randomizer.nextDouble());
            float gtQuaternionA = (float)gtQuaternion.getA();
            float gtQuaternionB = (float)gtQuaternion.getB();
            float gtQuaternionC = (float)gtQuaternion.getC();
            float gtQuaternionD = (float)gtQuaternion.getD();

            SlamEstimator estimator = new SlamEstimator();

            GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
        
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0,
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0,
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;
            double[] x = new double[16];
            double[] u = new double[9];
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
            
            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];

            //set initial state
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
                
                gtAccelerationX = (float)(amplitudeAccelerationX * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAccelerationX)));
                gtAccelerationY = (float)(amplitudeAccelerationY * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAccelerationY)));
                gtAccelerationZ = (float)(amplitudeAccelerationZ * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAccelerationZ)));
                
                accelerationX = gtAccelerationX + noiseAccelerationX;
                accelerationY = gtAccelerationY + noiseAccelerationY;
                accelerationZ = gtAccelerationZ + noiseAccelerationZ;

                noiseAngularSpeedX = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedY = angularSpeedRandomizer.nextFloat();
                noiseAngularSpeedZ = angularSpeedRandomizer.nextFloat();

                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;
            
                estimator.updateAccelerometerSample(timestamp, accelerationX, 
                        accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;
                
                if (i == 0) {
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
                    
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;
                    
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
                
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
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
        
            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = estimator.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            boolean rotationImproved = 
                    filteredRotated.distanceTo(groundTruthRotated) < 
                    predictedRotated.distanceTo(groundTruthRotated);
        
            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), 
                    estimator.getStatePositionY(), 
                    estimator.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(
                    gtX[0], gtX[1], gtX[2]);

            boolean positionImproved = 
                    filteredPos.distanceTo(groundTruthPos) < 
                    predictedPos.distanceTo(groundTruthPos);
            
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
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            double gtPositionX = randomizer.nextDouble();
            double gtPositionY = randomizer.nextDouble();
            double gtPositionZ = randomizer.nextDouble();
            double gtSpeedX = randomizer.nextDouble();
            double gtSpeedY = randomizer.nextDouble();
            double gtSpeedZ = randomizer.nextDouble();
            float gtAccelerationX = randomizer.nextFloat();
            float gtAccelerationY = randomizer.nextFloat();
            float gtAccelerationZ = randomizer.nextFloat();
            int period = N_PREDICTION_SAMPLES / 2;
            int offsetAngularSpeedX = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            int offsetAngularSpeedY = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            int offsetAngularSpeedZ = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            float amplitudeAngularSpeedX = randomizer.nextFloat();
            float amplitudeAngularSpeedY = randomizer.nextFloat();
            float amplitudeAngularSpeedZ = randomizer.nextFloat();
            float gtAngularSpeedX = (float)(amplitudeAngularSpeedX * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAngularSpeedX)));
            float gtAngularSpeedY = (float)(amplitudeAngularSpeedY * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAngularSpeedY)));
            float gtAngularSpeedZ = (float)(amplitudeAngularSpeedZ * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAngularSpeedZ)));
            Quaternion gtQuaternion = new Quaternion(
                    randomizer.nextDouble(),randomizer.nextDouble(), 
                    randomizer.nextDouble());
            float gtQuaternionA = (float)gtQuaternion.getA();
            float gtQuaternionB = (float)gtQuaternion.getB();
            float gtQuaternionC = (float)gtQuaternion.getC();
            float gtQuaternionD = (float)gtQuaternion.getD();

            SlamEstimator estimator = new SlamEstimator();

            GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
        
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0,
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0,
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;
            double[] x = new double[16];
            double[] u = new double[9];
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
            
            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];

            //set initial state
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

                gtAngularSpeedX = (float)(amplitudeAngularSpeedX * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAngularSpeedX)));
                gtAngularSpeedY = (float)(amplitudeAngularSpeedY * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAngularSpeedY)));
                gtAngularSpeedZ = (float)(amplitudeAngularSpeedZ * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAngularSpeedZ)));
                
                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;
            
                estimator.updateAccelerometerSample(timestamp, accelerationX, 
                        accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;
                
                if (i == 0) {
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
                    
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;
                    
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
                
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
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
        
            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = estimator.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            boolean rotationImproved = 
                    filteredRotated.distanceTo(groundTruthRotated) < 
                    predictedRotated.distanceTo(groundTruthRotated);
        
            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), 
                    estimator.getStatePositionY(), 
                    estimator.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(
                    gtX[0], gtX[1], gtX[2]);

            boolean positionImproved = 
                    filteredPos.distanceTo(groundTruthPos) < 
                    predictedPos.distanceTo(groundTruthPos);
            
            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }
        
        assertTrue(numSuccess > REPEAT_TIMES * REQUIRED_PREDICTION_SUCCESS_RATE);        
    }
    
    @Test
    public void testPredictionRandomAccelerationAndAngularSpeedWithNoise() {
        //when acceleration or angular speed has random values (abrupt changes)
        //Kalman Filter might obtain worse results because those changes are 
        //treated as noise
        int numSuccess = 0;
        for (int t = 0; t < REPEAT_TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            double gtPositionX = randomizer.nextDouble();
            double gtPositionY = randomizer.nextDouble();
            double gtPositionZ = randomizer.nextDouble();
            double gtSpeedX = randomizer.nextDouble();
            double gtSpeedY = randomizer.nextDouble();
            double gtSpeedZ = randomizer.nextDouble();
            float gtAccelerationX = randomizer.nextFloat();
            float gtAccelerationY = randomizer.nextFloat();
            float gtAccelerationZ = randomizer.nextFloat();
            float gtAngularSpeedX = randomizer.nextFloat();
            float gtAngularSpeedY = randomizer.nextFloat();
            float gtAngularSpeedZ = randomizer.nextFloat();
            Quaternion gtQuaternion = new Quaternion(
                    randomizer.nextDouble(),randomizer.nextDouble(), 
                    randomizer.nextDouble());
            float gtQuaternionA = (float)gtQuaternion.getA();
            float gtQuaternionB = (float)gtQuaternion.getB();
            float gtQuaternionC = (float)gtQuaternion.getC();
            float gtQuaternionD = (float)gtQuaternion.getD();

            SlamEstimator estimator = new SlamEstimator();
        
            GaussianRandomizer accelerationRandomizer = 
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer = 
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0, 
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0, 
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;            
            double[] x = new double[16];
            double[] u = new double[9];
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
        
            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];

            //set initial state
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

                estimator.updateAccelerometerSample(timestamp, 
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                timestamp += DELTA_NANOS;

                if (i == 0) {
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
                    
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

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
                    
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
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

            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = estimator.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            boolean rotationImproved = 
                    filteredRotated.distanceTo(groundTruthRotated) < 
                    predictedRotated.distanceTo(groundTruthRotated);

            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimator.getStatePositionX(), 
                    estimator.getStatePositionY(), 
                    estimator.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = 
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            boolean positionImproved = 
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
        for (int t = 0; t < REPEAT_TIMES; t++) {        
            UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
        
            float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ, 
                    angularOffsetX, angularOffsetY, angularOffsetZ, 
                    noiseRandomizer);
            SlamCalibrationData calibration = calibrator.getCalibrationData();

            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double gtPositionX = 0.0, gtPositionY = 0.0, gtPositionZ = 0.0;
            double gtSpeedX = 0.0, gtSpeedY = 0.0, gtSpeedZ = 0.0;
            float gtAccelerationX = 0.0f, gtAccelerationY = 0.0f, 
                    gtAccelerationZ = 0.0f;
            float gtAngularSpeedX = 0.0f, gtAngularSpeedY = 0.0f, 
                    gtAngularSpeedZ = 0.0f;
            float gtQuaternionA = 1.0f, gtQuaternionB = 0.0f, 
                    gtQuaternionC = 0.0f, gtQuaternionD = 0.0f;

            SlamEstimator estimator = new SlamEstimator();
        
            SlamEstimator estimatorWithCalibration = new SlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);

            GaussianRandomizer accelerationRandomizer = 
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer = 
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float accelerationWithOffsetX, accelerationWithOffsetY, 
                    accelerationWithOffsetZ;
            float angularSpeedWithOffsetX, angularSpeedWithOffsetY, 
                    angularSpeedWithOffsetZ;        
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0, 
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0, 
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastAccelerationWithOffsetX = 0.0, 
                    lastAccelerationWithOffsetY = 0.0, 
                    lastAccelerationWithOffsetZ = 0.0;
            double lastAngularSpeedWithOffsetX = 0.0, 
                    lastAngularSpeedWithOffsetY = 0.0, 
                    lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX, deltaAccelerationWithOffsetY, 
                    deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX, deltaAngularSpeedWithOffsetY, 
                    deltaAngularSpeedWithOffsetZ;
        
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;            
            double[] x = new double[16];
            double[] u = new double[9];
            double[] uWithOffset = new double[9];
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

            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];
        
            double[] xWithOffset = Arrays.copyOf(x, x.length);

            //set initial state
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

                estimator.updateAccelerometerSample(timestamp, 
                        accelerationX, accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                estimatorWithCalibration.updateAccelerometerSample(timestamp, 
                        accelerationWithOffsetX, accelerationWithOffsetY, 
                        accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp, 
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, 
                        angularSpeedWithOffsetZ);

                timestamp += DELTA_NANOS;

                if (i == 0) {
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

                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

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

                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
                    u[0] = u[1] = u[2] = 0.0;                
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    //we also take into account control signals with offset
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

            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = 
                    estimatorWithCalibration.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            boolean rotationImproved = 
                    filteredRotated.distanceTo(groundTruthRotated) < 
                    predictedRotated.distanceTo(groundTruthRotated);

            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), 
                    estimatorWithCalibration.getStatePositionY(), 
                    estimatorWithCalibration.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = 
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            boolean positionImproved = 
                    filteredPos.distanceTo(groundTruthPos) < 
                    predictedPos.distanceTo(groundTruthPos);
            
            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }
        
        assertTrue(numSuccess >= REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);        
    }
    
    @Test
    public void testPredictionConstantSpeedWithNoiseAndCalibration() {
        int numSuccess = 0;
        for (int t = 0; t < 5*REPEAT_TIMES; t++) {
            UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
        
            float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ, 
                    angularOffsetX, angularOffsetY, angularOffsetZ, 
                    noiseRandomizer);
            SlamCalibrationData calibration = calibrator.getCalibrationData();
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double gtPositionX = 0.0, gtPositionY = 0.0, gtPositionZ = 0.0;
            double gtSpeedX = randomizer.nextDouble();
            double gtSpeedY = randomizer.nextDouble();
            double gtSpeedZ = randomizer.nextDouble();
            float gtAccelerationX = 0.0f, gtAccelerationY = 0.0f, 
                    gtAccelerationZ = 0.0f;
            float gtAngularSpeedX = 0.0f, gtAngularSpeedY = 0.0f, 
                    gtAngularSpeedZ = 0.0f;
            float gtQuaternionA = 1.0f, gtQuaternionB = 0.0f, 
                    gtQuaternionC = 0.0f, gtQuaternionD = 0.0f;

            SlamEstimator estimator = new SlamEstimator();

            SlamEstimator estimatorWithCalibration = new SlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);
            
            GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);

            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float accelerationWithOffsetX, accelerationWithOffsetY, 
                    accelerationWithOffsetZ;
            float angularSpeedWithOffsetX, angularSpeedWithOffsetY, 
                    angularSpeedWithOffsetZ;                    
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0,
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0, 
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastAccelerationWithOffsetX = 0.0, 
                    lastAccelerationWithOffsetY = 0.0, 
                    lastAccelerationWithOffsetZ = 0.0;
            double lastAngularSpeedWithOffsetX = 0.0, 
                    lastAngularSpeedWithOffsetY = 0.0, 
                    lastAngularSpeedWithOffsetZ = 0.0;    
            double deltaAccelerationWithOffsetX, deltaAccelerationWithOffsetY, 
                    deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX, deltaAngularSpeedWithOffsetY, 
                    deltaAngularSpeedWithOffsetZ;
            
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;            
            double[] x = new double[16];
            double[] u = new double[9];
            double[] uWithOffset = new double[9];
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

            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];
            
            double[] xWithOffset = Arrays.copyOf(x, x.length);

            //set initial state
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
                
                estimator.updateAccelerometerSample(timestamp, accelerationX, 
                        accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                estimatorWithCalibration.updateAccelerometerSample(timestamp, 
                        accelerationWithOffsetX, accelerationWithOffsetY, 
                        accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp, 
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, 
                        angularSpeedWithOffsetZ);
                
                timestamp += DELTA_NANOS;

                if (i == 0) {
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
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;

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
                    
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    //we also take into account control signals with offset
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

            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = 
                    estimatorWithCalibration.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);

            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), 
                    estimatorWithCalibration.getStatePositionY(), 
                    estimatorWithCalibration.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = 
                    new InhomogeneousPoint3D(gtX[0], gtX[1], gtX[2]);

            boolean positionImproved =
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
            UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
        
            float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            
            SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ, 
                    angularOffsetX, angularOffsetY, angularOffsetZ, 
                    noiseRandomizer);
            SlamCalibrationData calibration = calibrator.getCalibrationData();
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double gtPositionX = 0.0, gtPositionY = 0.0, gtPositionZ = 0.0;
            double gtSpeedX = 0.0, gtSpeedY = 0.0, gtSpeedZ = 0.0;
            float gtAccelerationX = randomizer.nextFloat();
            float gtAccelerationY = randomizer.nextFloat();
            float gtAccelerationZ = randomizer.nextFloat();
            float gtAngularSpeedX = 0.0f, gtAngularSpeedY = 0.0f, 
                    gtAngularSpeedZ = 0.0f;
            float gtQuaternionA = 1.0f, gtQuaternionB = 0.0f, 
                    gtQuaternionC = 0.0f, gtQuaternionD = 0.0f;

            SlamEstimator estimator = new SlamEstimator();

            SlamEstimator estimatorWithCalibration = new SlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);
            
            GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
        
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float accelerationWithOffsetX, accelerationWithOffsetY, 
                    accelerationWithOffsetZ;
            float angularSpeedWithOffsetX, angularSpeedWithOffsetY, 
                    angularSpeedWithOffsetZ;                                
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0,
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0,
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastAccelerationWithOffsetX = 0.0, 
                    lastAccelerationWithOffsetY = 0.0, 
                    lastAccelerationWithOffsetZ = 0.0;
            double lastAngularSpeedWithOffsetX = 0.0, 
                    lastAngularSpeedWithOffsetY = 0.0, 
                    lastAngularSpeedWithOffsetZ = 0.0;    
            double deltaAccelerationWithOffsetX, deltaAccelerationWithOffsetY, 
                    deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX, deltaAngularSpeedWithOffsetY, 
                    deltaAngularSpeedWithOffsetZ;
            
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;            
            double[] x = new double[16];
            double[] u = new double[9];
            double[] uWithOffset = new double[9];
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
        
            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];

            double[] xWithOffset = Arrays.copyOf(x, x.length);
            
            //set initial state
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
                
                estimator.updateAccelerometerSample(timestamp, accelerationX, 
                        accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                estimatorWithCalibration.updateAccelerometerSample(timestamp, 
                        accelerationWithOffsetX, accelerationWithOffsetY, 
                        accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp, 
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, 
                        angularSpeedWithOffsetZ);
                
                timestamp += DELTA_NANOS;

                if (i == 0) {
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
                    
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;
                
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
                    
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    //we also take into account control signals with offset
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
        
            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = 
                    estimatorWithCalibration.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            boolean rotationImproved =
                    filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);
            
        
            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), 
                    estimatorWithCalibration.getStatePositionY(), 
                    estimatorWithCalibration.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(
                    gtX[0], gtX[1], gtX[2]);

            boolean positionImproved =
                    filteredPos.distanceTo(groundTruthPos) < 
                    predictedPos.distanceTo(groundTruthPos);
            
            if(rotationImproved && positionImproved) {
                numSuccess++;
            }                        
        }
        
        assertTrue(numSuccess >= REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);        
    }    
    
    @Test
    public void testPredictionRotationOnlyWithNoiseAndCalibration() {
        int numSuccess = 0;
        for (int t = 0; t < 5*REPEAT_TIMES; t++) {
            UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            
            float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
        
            float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            
            SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ, 
                    angularOffsetX, angularOffsetY, angularOffsetZ, 
                    noiseRandomizer);
            SlamCalibrationData calibration = calibrator.getCalibrationData();
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
            double gtPositionX = 0.0, gtPositionY = 0.0, gtPositionZ = 0.0;
            double gtSpeedX = 0.0, gtSpeedY = 0.0, gtSpeedZ = 0.0;
            float gtAccelerationX = 0.0f, gtAccelerationY = 0.0f,
                    gtAccelerationZ = 0.0f;
            float gtAngularSpeedX = randomizer.nextFloat();
            float gtAngularSpeedY = randomizer.nextFloat();
            float gtAngularSpeedZ = randomizer.nextFloat();
            float gtQuaternionA = 1.0f, gtQuaternionB = 0.0f, 
                    gtQuaternionC = 0.0f, gtQuaternionD = 0.0f;

            SlamEstimator estimator = new SlamEstimator();

            SlamEstimator estimatorWithCalibration = new SlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);
            
            GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
        
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX , accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float accelerationWithOffsetX, accelerationWithOffsetY, 
                    accelerationWithOffsetZ;
            float angularSpeedWithOffsetX, angularSpeedWithOffsetY, 
                    angularSpeedWithOffsetZ;                    
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0,
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0,
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastAccelerationWithOffsetX = 0.0, 
                    lastAccelerationWithOffsetY = 0.0, 
                    lastAccelerationWithOffsetZ = 0.0;
            double lastAngularSpeedWithOffsetX = 0.0, 
                    lastAngularSpeedWithOffsetY = 0.0, 
                    lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX, deltaAccelerationWithOffsetY, 
                    deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX, deltaAngularSpeedWithOffsetY, 
                    deltaAngularSpeedWithOffsetZ;
            
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;            
            double[] x = new double[16];
            double[] u = new double[9];
            double[] uWithOffset = new double[9];
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
        
            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];

            double[] xWithOffset = Arrays.copyOf(x, x.length);
            
            //set initial state
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
                
                estimator.updateAccelerometerSample(timestamp, accelerationX, 
                        accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                estimatorWithCalibration.updateAccelerometerSample(timestamp, 
                        accelerationWithOffsetX, accelerationWithOffsetY, 
                        accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp, 
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, 
                        angularSpeedWithOffsetZ);
                
                timestamp += DELTA_NANOS;

                if (i == 0) {
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
                    
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;
                
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
                    
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a constant 
                    //speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    //we also take into account control signals with offset
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
        
            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = 
                    estimatorWithCalibration.getStateQuaternion();
            Quaternion predictedQuaternion = 
                    new Quaternion(x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(randomPoint);

            boolean rotationImproved = 
                    filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);
        
            //check that filtered position is closer to ground truth than predicted 
            //position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), 
                    estimatorWithCalibration.getStatePositionY(), 
                    estimatorWithCalibration.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(gtX[0], 
                    gtX[1], gtX[2]);

            boolean positionImproved = 
                    filteredPos.distanceTo(groundTruthPos) < 
                    predictedPos.distanceTo(groundTruthPos);
            
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
            UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);
            
            float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
        
            float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ, 
                    angularOffsetX, angularOffsetY, angularOffsetZ, 
                    noiseRandomizer);
            SlamCalibrationData calibration = calibrator.getCalibrationData();
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double gtPositionX = 0.0, gtPositionY = 0.0, gtPositionZ = 0.0;            
            double gtSpeedX = randomizer.nextDouble();
            double gtSpeedY = randomizer.nextDouble();
            double gtSpeedZ = randomizer.nextDouble();
            int period = N_PREDICTION_SAMPLES / 2;
            int offsetAccelerationX = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            int offsetAccelerationY = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            int offsetAccelerationZ = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            float amplitudeAccelerationX = randomizer.nextFloat();
            float amplitudeAccelerationY = randomizer.nextFloat();
            float amplitudeAccelerationZ = randomizer.nextFloat();
            float gtAccelerationX = (float)(amplitudeAccelerationX * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAccelerationX)));
            float gtAccelerationY = (float)(amplitudeAccelerationY * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAccelerationY)));
            float gtAccelerationZ = (float)(amplitudeAccelerationZ * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAccelerationZ)));
            float gtAngularSpeedX = 0.0f, gtAngularSpeedY = 0.0f, 
                    gtAngularSpeedZ = 0.0f;
            float gtQuaternionA = 1.0f, gtQuaternionB = 0.0f, 
                    gtQuaternionC = 0.0f, gtQuaternionD = 0.0f;

            SlamEstimator estimator = new SlamEstimator();

            SlamEstimator estimatorWithCalibration = new SlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);
            
            GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
        
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float accelerationWithOffsetX, accelerationWithOffsetY, 
                    accelerationWithOffsetZ;
            float angularSpeedWithOffsetX, angularSpeedWithOffsetY, 
                    angularSpeedWithOffsetZ;                    
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0,
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0,
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastAccelerationWithOffsetX = 0.0, 
                    lastAccelerationWithOffsetY = 0.0, 
                    lastAccelerationWithOffsetZ = 0.0;
            double lastAngularSpeedWithOffsetX = 0.0, 
                    lastAngularSpeedWithOffsetY = 0.0, 
                    lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX, deltaAccelerationWithOffsetY, 
                    deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX, deltaAngularSpeedWithOffsetY, 
                    deltaAngularSpeedWithOffsetZ;
            
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;
            double[] x = new double[16];
            double[] u = new double[9];
            double[] uWithOffset = new double[9];
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
            
            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];
            
            double[] xWithOffset = Arrays.copyOf(x, x.length);

            //set initial state
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
                
                gtAccelerationX = (float)(amplitudeAccelerationX * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAccelerationX)));
                gtAccelerationY = (float)(amplitudeAccelerationY * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAccelerationY)));
                gtAccelerationZ = (float)(amplitudeAccelerationZ * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAccelerationZ)));
                
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

                estimator.updateAccelerometerSample(timestamp, accelerationX, 
                        accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                estimatorWithCalibration.updateAccelerometerSample(timestamp, 
                        accelerationWithOffsetX, accelerationWithOffsetY, 
                        accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp, 
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, 
                        angularSpeedWithOffsetZ);
                
                timestamp += DELTA_NANOS;
                
                if (i == 0) {
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
                    
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;
                    
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
                
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    //we also take into account control signals with offset
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
        
            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = 
                    estimatorWithCalibration.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            boolean rotationImproved = 
                    filteredRotated.distanceTo(groundTruthRotated) <
                    predictedRotated.distanceTo(groundTruthRotated);
        
            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), 
                    estimatorWithCalibration.getStatePositionY(), 
                    estimatorWithCalibration.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(
                    gtX[0], gtX[1], gtX[2]);

            boolean positionImproved = 
                    filteredPos.distanceTo(groundTruthPos) < 
                    predictedPos.distanceTo(groundTruthPos);

            if (rotationImproved && positionImproved) {
                numSuccess++;
            }            
        }
        
        assertTrue(numSuccess >= REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);        
    }
    
    @Test
    public void testPredictionVariableAngularSpeedWithNoiseAndCalibration() {
        int numSuccess = 0;
        for(int t = 0; t < 2*REPEAT_TIMES; t++) {
            UniformRandomizer offsetRandomizer = new UniformRandomizer(
                    new Random());
            GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ACCELERATION_NOISE_STANDARD_DEVIATION);

            float accelerationOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float accelerationOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
        
            float angularOffsetX = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetY = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);
            float angularOffsetZ = offsetRandomizer.nextFloat(
                    MIN_CALIBRATION_OFFSET, MAX_CALIBRATION_OFFSET);

            SlamCalibrator calibrator = createFinishedCalibrator(
                    accelerationOffsetX, accelerationOffsetY, accelerationOffsetZ, 
                    angularOffsetX, angularOffsetY, angularOffsetZ, 
                    noiseRandomizer);
            SlamCalibrationData calibration = calibrator.getCalibrationData();
            
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            double gtPositionX = 0.0, gtPositionY = 0.0, gtPositionZ = 0.0;
            double gtSpeedX = 0.0, gtSpeedY = 0.0, gtSpeedZ = 0.0;
            float gtAccelerationX = 0.0f, gtAccelerationY = 0.0f,
                    gtAccelerationZ = 0.0f;
            int period = N_PREDICTION_SAMPLES / 2;
            int offsetAngularSpeedX = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            int offsetAngularSpeedY = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            int offsetAngularSpeedZ = randomizer.nextInt(0, 
                    N_PREDICTION_SAMPLES);
            float amplitudeAngularSpeedX = randomizer.nextFloat();
            float amplitudeAngularSpeedY = randomizer.nextFloat();
            float amplitudeAngularSpeedZ = randomizer.nextFloat();
            float gtAngularSpeedX = (float)(amplitudeAngularSpeedX * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAngularSpeedX)));
            float gtAngularSpeedY = (float)(amplitudeAngularSpeedY * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAngularSpeedY)));
            float gtAngularSpeedZ = (float)(amplitudeAngularSpeedZ * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(0 - offsetAngularSpeedZ)));
            float gtQuaternionA = 1.0f, gtQuaternionB = 0.0f, 
                    gtQuaternionC = 0.0f, gtQuaternionD = 0.0f;

            SlamEstimator estimator = new SlamEstimator();

            SlamEstimator estimatorWithCalibration = new SlamEstimator();
            estimatorWithCalibration.setCalibrationData(calibration);
            
            GaussianRandomizer accelerationRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ACCELERATION_NOISE_STANDARD_DEVIATION);
            GaussianRandomizer angularSpeedRandomizer =
                    new GaussianRandomizer(new Random(), 0.0,
                            ANGULAR_SPEED_NOISE_STANDARD_DEVIATION);
        
            long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
            float accelerationX, accelerationY, accelerationZ;
            float angularSpeedX, angularSpeedY, angularSpeedZ;
            float accelerationWithOffsetX, accelerationWithOffsetY, 
                    accelerationWithOffsetZ;
            float angularSpeedWithOffsetX, angularSpeedWithOffsetY, 
                    angularSpeedWithOffsetZ;                    
            float noiseAccelerationX, noiseAccelerationY, noiseAccelerationZ;
            float noiseAngularSpeedX, noiseAngularSpeedY, noiseAngularSpeedZ;
            double lastAccelerationX = 0.0, lastAccelerationY = 0.0,
                    lastAccelerationZ = 0.0;
            double lastAngularSpeedX = 0.0, lastAngularSpeedY = 0.0,
                    lastAngularSpeedZ = 0.0;
            double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
            double deltaAngularSpeedX, deltaAngularSpeedY, deltaAngularSpeedZ;
            double lastAccelerationWithOffsetX = 0.0, 
                    lastAccelerationWithOffsetY = 0.0, 
                    lastAccelerationWithOffsetZ = 0.0;
            double lastAngularSpeedWithOffsetX = 0.0, 
                    lastAngularSpeedWithOffsetY = 0.0, 
                    lastAngularSpeedWithOffsetZ = 0.0;
            double deltaAccelerationWithOffsetX, deltaAccelerationWithOffsetY, 
                    deltaAccelerationWithOffsetZ;
            double deltaAngularSpeedWithOffsetX, deltaAngularSpeedWithOffsetY, 
                    deltaAngularSpeedWithOffsetZ;
            
            double lastGtAccelerationX = 0.0, lastGtAccelerationY = 0.0,
                    lastGtAccelerationZ = 0.0;
            double lastGtAngularSpeedX = 0.0, lastGtAngularSpeedY = 0.0,
                    lastGtAngularSpeedZ = 0.0;
            double deltaGtAccelerationX, deltaGtAccelerationY, 
                    deltaGtAccelerationZ;
            double deltaGtAngularSpeedX, deltaGtAngularSpeedY, 
                    deltaGtAngularSpeedZ;
            double[] x = new double[16];
            double[] u = new double[9];
            double[] uWithOffset = new double[9];
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
            
            //ground truth state and control
            double[] gtX = Arrays.copyOf(x, x.length);
            double[] gtU = new double[9];
            
            double[] xWithOffset = Arrays.copyOf(x, x.length);

            //set initial state
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

                gtAngularSpeedX = (float)(amplitudeAngularSpeedX * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAngularSpeedX)));
                gtAngularSpeedY = (float)(amplitudeAngularSpeedY * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAngularSpeedY)));
                gtAngularSpeedZ = (float)(amplitudeAngularSpeedZ * Math.sin(
                    2.0 * Math.PI / (double)period * 
                    (double)(i - offsetAngularSpeedZ)));
                
                angularSpeedX = gtAngularSpeedX + noiseAngularSpeedX;
                angularSpeedY = gtAngularSpeedY + noiseAngularSpeedY;
                angularSpeedZ = gtAngularSpeedZ + noiseAngularSpeedZ;
            
                angularSpeedWithOffsetX = angularSpeedX + angularOffsetX;
                angularSpeedWithOffsetY = angularSpeedY + angularOffsetY;
                angularSpeedWithOffsetZ = angularSpeedZ + angularOffsetZ;
                
                estimator.updateAccelerometerSample(timestamp, accelerationX, 
                        accelerationY, accelerationZ);
                estimator.updateGyroscopeSample(timestamp, 
                        angularSpeedX, angularSpeedY, angularSpeedZ);

                estimatorWithCalibration.updateAccelerometerSample(timestamp, 
                        accelerationWithOffsetX, accelerationWithOffsetY, 
                        accelerationWithOffsetZ);
                estimatorWithCalibration.updateGyroscopeSample(timestamp, 
                        angularSpeedWithOffsetX, angularSpeedWithOffsetY, 
                        angularSpeedWithOffsetZ);
                
                timestamp += DELTA_NANOS;
                
                if (i == 0) {
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
                    
                } else {
                    deltaAccelerationX = accelerationX - lastAccelerationX;
                    deltaAccelerationY = accelerationY - lastAccelerationY;
                    deltaAccelerationZ = accelerationZ - lastAccelerationZ;

                    deltaAngularSpeedX = angularSpeedX - lastAngularSpeedX;
                    deltaAngularSpeedY = angularSpeedY - lastAngularSpeedY;
                    deltaAngularSpeedZ = angularSpeedZ - lastAngularSpeedZ;
                    
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
                
                    //delta acceleration and angular speed only cointain noise, 
                    //since no real change in those values is present on a 
                    //constant speed movement
                    u[0] = u[1] = u[2] = 0.0;
                    u[3] = deltaAccelerationX;
                    u[4] = deltaAccelerationY;
                    u[5] = deltaAccelerationZ;
                    u[6] = deltaAngularSpeedX;
                    u[7] = deltaAngularSpeedY;
                    u[8] = deltaAngularSpeedZ;
                    StatePredictor.predict(x, u, DELTA_SECONDS, x);

                    //we also take into account control signals with offset
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
        
            //rotate random point with quaternions and check that filtered 
            //quaternion is closer to ground truth than predicted quaternion
            InhomogeneousPoint3D randomPoint = new InhomogeneousPoint3D(
                    randomizer.nextDouble(), randomizer.nextDouble(), 
                    randomizer.nextDouble());

            Quaternion filteredQuaternion = 
                    estimatorWithCalibration.getStateQuaternion();
            Quaternion predictedQuaternion = new Quaternion(
                    x[3], x[4], x[5], x[6]);
            Quaternion groundTruthQuaternion = new Quaternion(
                    gtX[3], gtX[4], gtX[5], gtX[6]);

            Point3D filteredRotated = filteredQuaternion.rotate(randomPoint);
            Point3D predictedRotated = predictedQuaternion.rotate(randomPoint);
            Point3D groundTruthRotated = groundTruthQuaternion.rotate(
                    randomPoint);

            boolean rotationImproved = 
                    filteredRotated.distanceTo(groundTruthRotated) < 
                    predictedRotated.distanceTo(groundTruthRotated);
        
            //check that filtered position is closer to ground truth than 
            //predicted position, and hence Kalman filter improves results
            InhomogeneousPoint3D filteredPos = new InhomogeneousPoint3D(
                    estimatorWithCalibration.getStatePositionX(), 
                    estimatorWithCalibration.getStatePositionY(), 
                    estimatorWithCalibration.getStatePositionZ());

            InhomogeneousPoint3D predictedPos = new InhomogeneousPoint3D(
                    x[0], x[1], x[2]);

            InhomogeneousPoint3D groundTruthPos = new InhomogeneousPoint3D(
                    gtX[0], gtX[1], gtX[2]);

            boolean positionImproved = 
                    filteredPos.distanceTo(groundTruthPos) < 
                    predictedPos.distanceTo(groundTruthPos);
            
            if (rotationImproved && positionImproved) {
                numSuccess++;
            }
        }
        
        assertTrue(numSuccess >= REPEAT_TIMES * REQUIRED_PREDICTION_WITH_CALIBRATION_SUCCESS_RATE);        
    }    

    @Override
    public void onFullSampleReceived(BaseSlamEstimator estimator) {
        fullSampleReceived++;
    }

    @Override
    public void onFullSampleProcessed(BaseSlamEstimator estimator) {
        fullSampleProcessed++;
    }

    @Override
    public void onCorrectWithPositionMeasure(BaseSlamEstimator estimator) {
        correctWithPositionMeasure++;
    }

    @Override
    public void onCorrectedWithPositionMeasure(BaseSlamEstimator estimator) {
        correctedWithPositionMeasure++;
    }

    private void reset() {
        fullSampleReceived = fullSampleProcessed = correctWithPositionMeasure =
                correctedWithPositionMeasure = 0;
    }

    private SlamCalibrator createFinishedCalibrator(float accelerationOffsetX,
            float accelerationOffsetY, float accelerationOffsetZ, 
            float angularOffsetX, float angularOffsetY, float angularOffsetZ,
            GaussianRandomizer noiseRandomizer) {
        SlamCalibrator calibrator = SlamEstimator.createCalibrator();
        calibrator.setConvergenceThreshold(ABSOLUTE_ERROR);
        calibrator.setMaxNumSamples(MAX_CALIBRATION_SAMPLES);
        
        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
        
        float accelerationNoiseX, accelerationNoiseY, accelerationNoiseZ;
        float angularNoiseX, angularNoiseY, angularNoiseZ;
        
        double accelerationX, accelerationY, accelerationZ;
        double angularX, angularY, angularZ;

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
            
            calibrator.updateAccelerometerSample(timestamp, (float)accelerationX, 
                    (float)accelerationY, (float)accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float)angularX, (float)angularY, 
                    (float)angularZ);
            
            if (calibrator.isFinished()) {
                break;
            }
            
            timestamp += DELTA_NANOS;
        }
        
        return calibrator;
    }    
}
