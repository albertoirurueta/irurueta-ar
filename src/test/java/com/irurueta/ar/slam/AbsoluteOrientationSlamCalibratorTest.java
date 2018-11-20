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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.slam.BaseSlamCalibrator.BaseSlamCalibratorListener;
import com.irurueta.geometry.Quaternion;
import com.irurueta.numerical.signal.processing.MeasurementNoiseCovarianceEstimator;
import com.irurueta.numerical.signal.processing.SignalProcessingException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.InvalidCovarianceMatrixException;
import com.irurueta.statistics.MultivariateNormalDist;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class AbsoluteOrientationSlamCalibratorTest implements 
        BaseSlamCalibratorListener {
    
    private static final int TIMES = 50;
    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-2;
    
    private static final float MIN_OFFSET = -10.0f;
    private static final float MAX_OFFSET = 10.0f;
    
    private static final float NOISE_DEVIATION = 1e-5f;
    
    private static final int N_SAMPLES = 10000;
    
    //conversion from milliseconds to nanoseconds
    private static final int MILLIS_TO_NANOS = 1000000;
    
    //time between samples expressed in nanoseconds (a typical sensor in Android 
    //delivers a sample every 20ms)
    private static final int DELTA_NANOS = 20000000; //0.02 seconds
    
    private int fullSampleReceived;
    private int fullSampleProcessed;
    private int calibratorFinished;    
    
    
    public AbsoluteOrientationSlamCalibratorTest() { }
    
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
        AbsoluteOrientationSlamCalibrator calibrator =
                new AbsoluteOrientationSlamCalibrator();
        
        //check initial values
        assertEquals(calibrator.getSampleLength(),
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH);
        assertEquals(calibrator.getEstimatorStateLength(),
                AbsoluteOrientationSlamEstimator.STATE_LENGTH);
        assertFalse(calibrator.isConverged());
        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertEquals(calibrator.getSampleCount(), 0);
        assertEquals(calibrator.getMinNumSamples(), 
                SlamCalibrator.DEFAULT_MIN_NUM_SAMPLES);
        assertEquals(calibrator.getMaxNumSamples(),
                SlamCalibrator.DEFAULT_MAX_NUM_SAMPLES);
        assertEquals(calibrator.getConvergenceThreshold(),
                SlamCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, 0.0);
        assertEquals(calibrator.isAccumulationEnabled(),
                SlamCalibrator.DEFAULT_ENABLE_SAMPLE_ACCUMULATION);
        assertEquals(calibrator.getAccelerometerTimestampNanos(), -1);
        assertEquals(calibrator.getGyroscopeTimestampNanos(), -1);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        assertArrayEquals(calibrator.getAccumulatedAccelerationSample(),
                new double[]{0.0,0.0,0.0}, 0.0);
        double[] accumAcceleration = new double[3];
        calibrator.getAccumulatedAccelerationSample(accumAcceleration);
        assertArrayEquals(accumAcceleration, new double[]{0.0,0.0,0.0}, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertArrayEquals(calibrator.getAccumulatedAngularSpeedSample(),
                new double[]{0.0,0.0,0.0}, 0.0);
        double[] accumAngularSpeed = new double[3];
        calibrator.getAccumulatedAngularSpeedSample(accumAngularSpeed);
        assertArrayEquals(accumAngularSpeed, new double[]{0.0,0.0,0.0}, 0.0);
        assertNull(calibrator.getListener());
        assertArrayEquals(calibrator.getControlMean(),
                new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH], 
                0.0);
        double[] controlMean = new double[
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH];
        calibrator.getControlMean(controlMean);
        assertArrayEquals(controlMean, new double[
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH], 0.0);
        assertEquals(calibrator.getControlCovariance(),
                new Matrix(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH));
        Matrix cov = new Matrix(1,1);
        calibrator.getControlCovariance(cov);
        assertEquals(cov, new Matrix(
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH));
    }
    
    @Test
    public void testIsConverged() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertFalse(calibrator.isConverged());
        
        //set new value
        calibrator.mConverged = true;
        
        //check correctness
        assertTrue(calibrator.isConverged());                
    }
    
    @Test
    public void testIsFailed() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertFalse(calibrator.isFailed());
        
        //set new value
        calibrator.mFailed = true;
        
        //check correctness
        assertTrue(calibrator.isFailed());
    }
    
    @Test
    public void testIsFinished() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertFalse(calibrator.isFinished());
        
        //set new value
        calibrator.mFinished = true;
        
        //check correctness
        assertTrue(calibrator.isFinished());
    }
    
    @Test
    public void testGetSampleCount() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getSampleCount(), 0);
        
        //set new value
        calibrator.mSampleCount = 5;
        
        //check correctness
        assertEquals(calibrator.getSampleCount(), 5);
    }
    
    @Test
    public void testGetSetMinNumSamples() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getMinNumSamples(), 
                SlamCalibrator.DEFAULT_MIN_NUM_SAMPLES);
        
        //set new value
        calibrator.setMinNumSamples(50);
        
        //check correctness
        assertEquals(calibrator.getMinNumSamples(), 50);
    }
    
    @Test
    public void testGetSetMaxNumSamples() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getMaxNumSamples(),
                SlamCalibrator.DEFAULT_MAX_NUM_SAMPLES);
        
        //set new value
        calibrator.setMaxNumSamples(1000);
        
        //check correctness
        assertEquals(calibrator.getMaxNumSamples(), 1000);
    }
    
    @Test
    public void testGetSetConvergenceThreshold() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getConvergenceThreshold(),
                SlamCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, 0.0);
        
        //set new value
        calibrator.setConvergenceThreshold(1.0);
        
        //check correctness
        assertEquals(calibrator.getConvergenceThreshold(), 1.0, 0.0);
        
        //Force IllegalArgumentException
        try {
            calibrator.setConvergenceThreshold(-0.1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testReset() throws WrongSizeException {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        calibrator.reset();
        
        //check correctness
        assertFalse(calibrator.isConverged());
        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertEquals(calibrator.getSampleCount(), 0);
        assertEquals(calibrator.getMinNumSamples(), 
                SlamCalibrator.DEFAULT_MIN_NUM_SAMPLES);
        assertEquals(calibrator.getMaxNumSamples(),
                SlamCalibrator.DEFAULT_MAX_NUM_SAMPLES);
        assertEquals(calibrator.getConvergenceThreshold(),
                SlamCalibrator.DEFAULT_CONVERGENCE_THRESHOLD, 0.0);
        assertEquals(calibrator.isAccumulationEnabled(),
                SlamCalibrator.DEFAULT_ENABLE_SAMPLE_ACCUMULATION);
        assertEquals(calibrator.getAccelerometerTimestampNanos(), -1);
        assertEquals(calibrator.getGyroscopeTimestampNanos(), -1);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        assertArrayEquals(calibrator.getAccumulatedAccelerationSample(),
                new double[]{0.0,0.0,0.0}, 0.0);
        double[] accumAcceleration = new double[3];
        calibrator.getAccumulatedAccelerationSample(accumAcceleration);
        assertArrayEquals(accumAcceleration, new double[]{0.0,0.0,0.0}, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertArrayEquals(calibrator.getAccumulatedAngularSpeedSample(),
                new double[]{0.0,0.0,0.0}, 0.0);
        double[] accumAngularSpeed = new double[3];
        calibrator.getAccumulatedAngularSpeedSample(accumAngularSpeed);
        assertArrayEquals(accumAngularSpeed, new double[]{0.0,0.0,0.0}, 0.0);
        assertNull(calibrator.getListener());
        assertArrayEquals(calibrator.getControlMean(),
                new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH], 
                0.0);
        double[] controlMean = new double[
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH];
        calibrator.getControlMean(controlMean);
        assertArrayEquals(controlMean, new double[
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH], 0.0);
        assertEquals(calibrator.getControlCovariance(),
                new Matrix(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH));
        Matrix cov = new Matrix(1,1);
        calibrator.getControlCovariance(cov);
        assertEquals(cov, new Matrix(
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH));        
    }
    
    @Test
    public void testIsSetAccumulationEnabled() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertTrue(calibrator.isAccumulationEnabled());
        
        //set new value
        calibrator.setAccumulationEnabled(false);
        
        //check correctness
        assertFalse(calibrator.isAccumulationEnabled());
    }
    
    @Test
    public void testGetAccelerometerTimestampNanos() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getAccelerometerTimestampNanos(), -1);
        
        //set new value
        calibrator.mAccelerometerTimestampNanos = 1000;
        
        //check correctness
        assertEquals(calibrator.getAccelerometerTimestampNanos(), 1000);
    }
    
    @Test
    public void testGetGyroscopeTimestampNanos() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getGyroscopeTimestampNanos(), -1);
        
        //set new value
        calibrator.mGyroscopeTimestampNanos = 2000;
        
        //check correctness
        assertEquals(calibrator.getGyroscopeTimestampNanos(), 2000);
    }
    
    @Test
    public void testGetAccumulatedAccelerometerSamplesAndIsAccelerometerSampleReceived() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 0);
        assertFalse(calibrator.isAccelerometerSampleReceived());
        
        //set new value
        calibrator.mAccumulatedAccelerometerSamples = 50;
        
        //check correctness
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 50);
        assertTrue(calibrator.isAccelerometerSampleReceived());
    }
    
    @Test
    public void testGetAccumulatedGyroscopeSamples() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(calibrator.isGyroscopeSampleReceived());
        
        //set new value
        calibrator.mAccumulatedGyroscopeSamples = 500;
        
        //check correctness
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 500);
        assertTrue(calibrator.isGyroscopeSampleReceived());
    }
    
    @Test
    public void testIsFullSampleAvailable() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial values
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
        assertEquals(calibrator.getAccumulatedOrientationSamples(), 0);
        assertFalse(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isOrientationSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());
        
        //set accelerometer sample
        calibrator.mAccumulatedAccelerometerSamples = 1;
        
        //check correctness
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 1);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
        assertEquals(calibrator.getAccumulatedOrientationSamples(), 0);
        assertTrue(calibrator.isAccelerometerSampleReceived());
        assertFalse(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isOrientationSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());
        
        //set gyroscope sample
        calibrator.mAccumulatedGyroscopeSamples = 1;
        
        //check correctness
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 1);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 1);
        assertEquals(calibrator.getAccumulatedOrientationSamples(), 0);
        assertTrue(calibrator.isAccelerometerSampleReceived());
        assertTrue(calibrator.isGyroscopeSampleReceived());
        assertFalse(calibrator.isOrientationSampleReceived());
        assertFalse(calibrator.isFullSampleAvailable());        
        
        //set orientation sample
        calibrator.mAccumulatedOrientationSamples = 1;
        
        //check correctness
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 1);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 1);
        assertEquals(calibrator.getAccumulatedOrientationSamples(), 1);
        assertTrue(calibrator.isAccelerometerSampleReceived());
        assertTrue(calibrator.isGyroscopeSampleReceived());
        assertTrue(calibrator.isOrientationSampleReceived());
        assertTrue(calibrator.isFullSampleAvailable());                
    }
    
    @Test
    public void testGetAccumulatedAccelerationSampleX() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        
        //set new value
        calibrator.mAccumulatedAccelerationSampleX = 1.0;
        
        //check correctness
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(), 1.0, 0.0);        
    }
    
    @Test
    public void testGetAccumulatedAccelerationSampleY() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        
        //set new value
        calibrator.mAccumulatedAccelerationSampleY = 2.0;
        
        //check correctness
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(), 2.0, 0.0);
    }
    
    @Test
    public void testGetAccumulatedAccelerationSampleZ() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        
        //set new value
        calibrator.mAccumulatedAccelerationSampleZ = 3.0;
        
        //check correctness
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(), 3.0, 0.0);
    }
    
    @Test
    public void testGetAccumulatedAccelerationSample() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertArrayEquals(calibrator.getAccumulatedAccelerationSample(),
                new double[]{0.0,0.0,0.0}, 0.0);
        
        //set new value
        calibrator.mAccumulatedAccelerationSampleX = 1.0;
        calibrator.mAccumulatedAccelerationSampleY = 2.0;
        calibrator.mAccumulatedAccelerationSampleZ = 3.0;
        
        //check correctness
        assertArrayEquals(calibrator.getAccumulatedAccelerationSample(),
                new double[]{1.0,2.0,3.0}, 0.0);
        
        double[] sample = new double[3];
        calibrator.getAccumulatedAccelerationSample(sample);
        
        //check correctness
        assertArrayEquals(sample, new double[]{1.0,2.0,3.0}, 0.0);
    }
    
    @Test
    public void testGetAccumulatedAngularSpeedSampleX() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        
        //set new value
        calibrator.mAccumulatedAngularSpeedSampleX = 1.0;
        
        //check correctness
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(), 1.0, 0.0);
    }
    
    @Test
    public void testGetAccumulatedAngularSpeedSampleY() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        
        //set new value
        calibrator.mAccumulatedAngularSpeedSampleY = 2.0;
        
        //check correctness
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(), 2.0, 0.0);
    }
    
    @Test
    public void testGetAccumulatedAngularSpeedSampleZ() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        
        //set new value
        calibrator.mAccumulatedAngularSpeedSampleZ = 3.0;
        
        //check correctness
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(), 3.0, 0.0);
    }
    
    @Test
    public void testGetAccumulatedAngularSpeedSample() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertArrayEquals(calibrator.getAccumulatedAngularSpeedSample(),
                new double[]{0.0,0.0,0.0}, 0.0);
        
        //set new value
        calibrator.mAccumulatedAngularSpeedSampleX = 1.0;
        calibrator.mAccumulatedAngularSpeedSampleY = 2.0;
        calibrator.mAccumulatedAngularSpeedSampleZ = 3.0;
        
        //check correctness
        assertArrayEquals(calibrator.getAccumulatedAngularSpeedSample(),
                new double[]{1.0,2.0,3.0}, 0.0);
        
        double[] sample = new double[3];
        calibrator.getAccumulatedAngularSpeedSample(sample);
        
        //check correctness
        assertArrayEquals(sample, new double[]{1.0,2.0,3.0}, 0.0);
    }
    
    @Test
    public void testGetMostRecentTimestampNanos() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //check correctness
        assertEquals(calibrator.getMostRecentTimestampNanos(), -1);
        
        //set new value
        calibrator.mAccelerometerTimestampNanos = 1000;
        
        //check correctness
        assertEquals(calibrator.getMostRecentTimestampNanos(), 1000);
        
        //set new value
        calibrator.mGyroscopeTimestampNanos = 2000;
        
        //check correctness
        assertEquals(calibrator.getMostRecentTimestampNanos(), 2000);
    }
    
    @Test
    public void testGetSetListener() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        //initial value
        assertNull(calibrator.getListener());
        
        //set new value
        calibrator.setListener(this);
        
        //check correctness
        assertSame(calibrator.getListener(), this);
    }
    
    @Test
    public void testGetControlMean() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        assertArrayEquals(calibrator.getControlMean(), 
                new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH], 
                0.0);
        assertArrayEquals(calibrator.getControlMean(), 
                calibrator.mEstimator.getSampleAverage(), 0.0);
        
        double[] controlMean = new double[
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH];
        calibrator.getControlMean(controlMean);
        assertArrayEquals(calibrator.getControlMean(), controlMean, 0.0);
        
        //Force IllegalArgumentException
        double[] wrong = new double[1];
        try {
            calibrator.getControlMean(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetControlCovariance() throws WrongSizeException {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        assertEquals(calibrator.getControlCovariance(), 
                new Matrix(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH, 
                        AbsoluteOrientationSlamEstimator.CONTROL_LENGTH));
        
        Matrix cov = new Matrix(1,1);
        calibrator.getControlCovariance(cov);
        
        assertEquals(calibrator.getControlCovariance(), cov);
    }
    
    @Test
    public void testGetControlDistribution() 
            throws InvalidCovarianceMatrixException {
        AbsoluteOrientationSlamCalibrator calibrator =
                new AbsoluteOrientationSlamCalibrator();
        
        double[] controlMean = calibrator.getControlMean();
        Matrix cov = calibrator.getControlCovariance();
        
        MultivariateNormalDist dist = calibrator.getControlDistribution();
        
        //check correctness
        assertArrayEquals(controlMean, dist.getMean(), ABSOLUTE_ERROR);
        assertTrue(cov.equals(dist.getCovariance(), ABSOLUTE_ERROR));
        
        MultivariateNormalDist dist2 = new MultivariateNormalDist();
        calibrator.getControlDistribution(dist2);
        
        //check correctness
        assertArrayEquals(controlMean, dist2.getMean(), ABSOLUTE_ERROR);
        assertTrue(cov.equals(dist2.getCovariance(), ABSOLUTE_ERROR));
    }
    
    @Test
    public void testGetCalibrationData() {
        AbsoluteOrientationSlamCalibrator calibrator =
                new AbsoluteOrientationSlamCalibrator();
        
        double[] controlMean = calibrator.getControlMean();
        Matrix cov = calibrator.getControlCovariance();

        AbsoluteOrientationSlamCalibrationData data1 =
                calibrator.getCalibrationData();
        AbsoluteOrientationSlamCalibrationData data2 = 
                new AbsoluteOrientationSlamCalibrationData();
        calibrator.getCalibrationData(data2);
        
        //check correctness
        assertSame(data1.getControlMean(), controlMean);
        assertSame(data1.getControlCovariance(), cov);
        
        assertSame(data2.getControlMean(), controlMean);
        assertSame(data2.getControlCovariance(), cov);        
    }
    
    @Test
    public void testPropagateWithControlJacobian() throws WrongSizeException,
            InvalidCovarianceMatrixException {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        
        UniformRandomizer offsetRandomizer = new UniformRandomizer(
                new Random());
        GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, NOISE_DEVIATION);
        
        float accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        float accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        float accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        
        float angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        float angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        float angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET,
                MAX_OFFSET);
        
        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
        
        float accelerationNoiseX, accelerationNoiseY, accelerationNoiseZ;
        float angularNoiseX, angularNoiseY, angularNoiseZ;
        
        double accelerationX, accelerationY, accelerationZ;
        double angularX, angularY, angularZ;
        Quaternion orientation = new Quaternion();
        
        calibrator.reset();
        
        for (int i = 0; i < N_SAMPLES; i++) {
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
            
            calibrator.updateAccelerometerSample(timestamp, 
                    (float)accelerationX, (float)accelerationY,
                    (float)accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float)angularX,
                    (float)angularY, (float)angularZ);
            calibrator.updateOrientationSample(timestamp, orientation);
            
            if(calibrator.isFinished()) break;
            
            timestamp += DELTA_NANOS;
        }
        
        assertTrue(calibrator.isConverged());
        assertTrue(calibrator.isFinished());
        assertFalse(calibrator.isFailed());
        
        Matrix cov = calibrator.getControlCovariance();
        
        Matrix jacobian = Matrix.identity(calibrator.getEstimatorStateLength(),
                calibrator.getSampleLength());
        jacobian.multiplyByScalar(2.0);
        MultivariateNormalDist dist = calibrator.propagateWithControlJacobian(
                jacobian);
        MultivariateNormalDist dist2 = new MultivariateNormalDist();
        calibrator.propagateWithControlJacobian(jacobian, dist2);
        
        //check correctness
        Matrix propagatedCov = jacobian.multiplyAndReturnNew(cov).
                multiplyAndReturnNew(jacobian.transposeAndReturnNew());
        
        assertTrue(dist.getCovariance().equals(propagatedCov, ABSOLUTE_ERROR));
        assertTrue(dist2.getCovariance().equals(propagatedCov, ABSOLUTE_ERROR));
    }
    
    @Test
    public void testUpdateAccelerometerSampleWithAccumulationDisabled() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        calibrator.setAccumulationEnabled(false);
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        long timestamp = System.currentTimeMillis();
        float accelerationX = randomizer.nextFloat();
        float accelerationY = randomizer.nextFloat();
        float accelerationZ = randomizer.nextFloat();
        
        //check initial values
        assertEquals(calibrator.getAccelerometerTimestampNanos(), -1);
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 0);
        assertFalse(calibrator.isFullSampleAvailable());
        
        calibrator.updateAccelerometerSample(timestamp, accelerationX, 
                accelerationY, accelerationZ);
        
        //check correctness
        assertEquals(calibrator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(),
                accelerationX, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(),
                accelerationY, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(),
                accelerationZ, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 1);
        assertFalse(calibrator.isFullSampleAvailable());
        
        //test again but using an array
        float[] acceleration = new float[3];
        randomizer.fill(acceleration);
        timestamp += 1000;
        
        calibrator.updateAccelerometerSample(timestamp, acceleration);
        
        //check correctness
        assertEquals(calibrator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(),
                acceleration[0], 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(),
                acceleration[1], 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(),
                acceleration[2], 0.0);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 2);
        assertFalse(calibrator.isFullSampleAvailable());
        
        //Force IllegalArguemntException
        float[] wrong = new float[4];
        try {
            calibrator.updateAccelerometerSample(timestamp, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }    
    
    @Test
    public void testUpdateAccelerometerSampleWithAccumulationEnabled() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        calibrator.setAccumulationEnabled(true);
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(calibrator.getAccelerometerTimestampNanos(), -1);
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), 0);
        assertFalse(calibrator.isFullSampleAvailable());
        
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
            
            calibrator.updateAccelerometerSample(timestamp, accelerationX, 
                    accelerationY, accelerationZ);
        }
        
        //check correctness
        assertEquals(calibrator.getAccelerometerTimestampNanos(), timestamp);
        assertEquals(calibrator.getAccumulatedAccelerationSampleX(),
                avgAccelerationX, ABSOLUTE_ERROR);
        assertEquals(calibrator.getAccumulatedAccelerationSampleY(),
                avgAccelerationY, ABSOLUTE_ERROR);
        assertEquals(calibrator.getAccumulatedAccelerationSampleZ(),
                avgAccelerationZ, ABSOLUTE_ERROR);
        assertEquals(calibrator.getAccumulatedAccelerometerSamples(), TIMES);
        assertFalse(calibrator.isFullSampleAvailable());
    }
    
    @Test
    public void testUpdateGyroscopeSampleWithAccumulationDisabled() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        calibrator.setAccumulationEnabled(false);
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        long timestamp = System.currentTimeMillis();
        float angularSpeedX = randomizer.nextFloat();
        float angularSpeedY = randomizer.nextFloat();
        float angularSpeedZ = randomizer.nextFloat();
        
        //check initial values
        assertEquals(calibrator.getGyroscopeTimestampNanos(), -1);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(calibrator.isFullSampleAvailable());
        
        calibrator.updateGyroscopeSample(timestamp, angularSpeedX, 
                angularSpeedY, angularSpeedZ);
        
        //check correctness
        assertEquals(calibrator.getGyroscopeTimestampNanos(), timestamp);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(),
                angularSpeedX, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(),
                angularSpeedY, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(),
                angularSpeedZ, 0.0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 1);
        assertFalse(calibrator.isFullSampleAvailable());
        
        //test again but using an array
        float[] angularSpeed = new float[3];
        randomizer.fill(angularSpeed);
        timestamp += 100;
        
        calibrator.updateGyroscopeSample(timestamp, angularSpeed);
        
        //check correctness
        assertEquals(calibrator.getGyroscopeTimestampNanos(), timestamp);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(),
                angularSpeed[0], 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(),
                angularSpeed[1], 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(),
                angularSpeed[2], 0.0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 2);
        assertFalse(calibrator.isFullSampleAvailable());
        
        //Force IllegalArgumentException
        float[] wrong = new float[4];
        try {
            calibrator.updateGyroscopeSample(timestamp, wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testUpdateGyroscopeSampleWithAccumulationEnabled() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        calibrator.setAccumulationEnabled(true);
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(calibrator.getGyroscopeTimestampNanos(), -1);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), 0);
        assertFalse(calibrator.isFullSampleAvailable());
        
        //update with several samples
        long timestamp = System.currentTimeMillis();
        float angularSpeedX, angularSpeedY, angularSpeedZ;
        double avgAngularSpeedX = 0.0, avgAngularSpeedY = 0.0,
                avgAngularSpeedZ = 0.0;
        for(int i = 0; i < TIMES; i++) {
            timestamp += 1000;
            angularSpeedX = randomizer.nextFloat();
            angularSpeedY = randomizer.nextFloat();
            angularSpeedZ = randomizer.nextFloat();
            
            avgAngularSpeedX += angularSpeedX / TIMES;
            avgAngularSpeedY += angularSpeedY / TIMES;
            avgAngularSpeedZ += angularSpeedZ / TIMES;
            
            calibrator.updateGyroscopeSample(timestamp, angularSpeedX, 
                    angularSpeedY, angularSpeedZ);
        }
        
        //check correctness
        assertEquals(calibrator.getGyroscopeTimestampNanos(), timestamp);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleX(),
                avgAngularSpeedX, ABSOLUTE_ERROR);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleY(),
                avgAngularSpeedY, ABSOLUTE_ERROR);
        assertEquals(calibrator.getAccumulatedAngularSpeedSampleZ(),
                avgAngularSpeedZ, ABSOLUTE_ERROR);
        assertEquals(calibrator.getAccumulatedGyroscopeSamples(), TIMES);
        assertFalse(calibrator.isFullSampleAvailable());
    }
    
    @Test
    public void testUpdateOrientationSampleWithAccumulationDisabled() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        calibrator.setAccumulationEnabled(false);
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        long timestamp = System.currentTimeMillis();
        double orientationA = randomizer.nextDouble();
        double orientationB = randomizer.nextDouble();
        double orientationC = randomizer.nextDouble();
        double orientationD = randomizer.nextDouble();
        Quaternion orientation = new Quaternion(orientationA, orientationB, 
                orientationC, orientationD);
        
        //check initial values
        assertEquals(calibrator.getOrientationTimestampNanos(), -1);
        Quaternion accumulatedOrientation = 
                (Quaternion)calibrator.getAccumulatedOrientation();
        assertEquals(accumulatedOrientation.getA(), 1.0, 0.0);
        assertEquals(accumulatedOrientation.getB(), 0.0, 0.0);
        assertEquals(accumulatedOrientation.getC(), 0.0, 0.0);
        assertEquals(accumulatedOrientation.getD(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedOrientationSamples(), 0);
        assertFalse(calibrator.isFullSampleAvailable());
        
        calibrator.updateOrientationSample(timestamp, orientation);
        
        //check correctness
        assertEquals(calibrator.getOrientationTimestampNanos(), timestamp);
        accumulatedOrientation = 
                (Quaternion)calibrator.getAccumulatedOrientation();
        assertEquals(accumulatedOrientation.getA(), orientationA, 0.0);
        assertEquals(accumulatedOrientation.getB(), orientationB, 0.0);
        assertEquals(accumulatedOrientation.getC(), orientationC, 0.0);
        assertEquals(accumulatedOrientation.getD(), orientationD, 0.0);
    }
    
    @Test
    public void testUpdateOrientationSampleWithAccumulationEnabled() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        calibrator.setAccumulationEnabled(true);
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //check initial values
        assertEquals(calibrator.getOrientationTimestampNanos(), -1);
        Quaternion accumulatedOrientation = 
                (Quaternion)calibrator.getAccumulatedOrientation();
        assertEquals(accumulatedOrientation.getA(), 1.0, 0.0);
        assertEquals(accumulatedOrientation.getB(), 0.0, 0.0);
        assertEquals(accumulatedOrientation.getC(), 0.0, 0.0);
        assertEquals(accumulatedOrientation.getD(), 0.0, 0.0);
        assertEquals(calibrator.getAccumulatedOrientationSamples(), 0);
        assertFalse(calibrator.isFullSampleAvailable());
        
        //update with several samples
        long timestamp = System.currentTimeMillis();
        double orientationA, orientationB, orientationC, orientationD;
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
            
            calibrator.updateOrientationSample(timestamp, orientation);
        }
        
        //check correctness
        assertEquals(calibrator.getOrientationTimestampNanos(), timestamp);
        accumulatedOrientation = 
                (Quaternion)calibrator.getAccumulatedOrientation();
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
    public void testCalibrationWithOffset() throws SignalProcessingException {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        calibrator.setListener(this);
        
        //setup so that calibrator doesn't reach convergence
        calibrator.setMaxNumSamples(N_SAMPLES);
        calibrator.setConvergenceThreshold(0.0);
        
        UniformRandomizer offsetRandomizer = new UniformRandomizer(
                new Random());
        GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, NOISE_DEVIATION);
        
        float accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        float accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        float accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        
        float angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        float angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        float angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        
        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
        
        float accelerationNoiseX, accelerationNoiseY, accelerationNoiseZ;
        float angularNoiseX, angularNoiseY, angularNoiseZ;
        
        double accelerationX = 0.0, accelerationY = 0.0, accelerationZ = 0.0;
        double angularX = 0.0, angularY = 0.0, angularZ = 0.0;
        double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
        double deltaAngularX, deltaAngularY, deltaAngularZ;
        double lastAccelerationX = 0.0, lastAccelerationY = 0.0, 
                lastAccelerationZ = 0.0;
        double lastAngularX = 0.0, lastAngularY = 0.0, lastAngularZ = 0.0;
        Quaternion orientation = new Quaternion();
                
        calibrator.reset();
        reset();
        
        MeasurementNoiseCovarianceEstimator estimator = 
                new MeasurementNoiseCovarianceEstimator(13);
        double[] sample = new double[13];
        
        for (int i = 0; i < N_SAMPLES; i++) {
            accelerationNoiseX = noiseRandomizer.nextFloat();
            accelerationNoiseY = noiseRandomizer.nextFloat();
            accelerationNoiseZ = noiseRandomizer.nextFloat();
            
            angularNoiseX = noiseRandomizer.nextFloat();
            angularNoiseY = noiseRandomizer.nextFloat();
            angularNoiseZ = noiseRandomizer.nextFloat();
            
            accelerationX += accelerationOffsetX + accelerationNoiseX;
            accelerationY += accelerationOffsetY + accelerationNoiseY;
            accelerationZ += accelerationOffsetZ + accelerationNoiseZ;
            
            angularX += angularOffsetX + angularNoiseX;
            angularY += angularOffsetY + angularNoiseY;
            angularZ += angularOffsetZ + angularNoiseZ;
            
            calibrator.updateAccelerometerSample(timestamp, (float)accelerationX, 
                    (float)accelerationY, (float)accelerationZ);
            calibrator.updateGyroscopeSample(timestamp, (float)angularX, (float)angularY, 
                    (float)angularZ);
            calibrator.updateOrientationSample(timestamp, orientation);
            
            timestamp += DELTA_NANOS;
            
            if (i != 0) {
                deltaAccelerationX = accelerationX - lastAccelerationX;
                deltaAccelerationY = accelerationY - lastAccelerationY;
                deltaAccelerationZ = accelerationZ - lastAccelerationZ;
                
                deltaAngularX = angularX - lastAngularX;
                deltaAngularY = angularY - lastAngularY;
                deltaAngularZ = angularZ - lastAngularZ;
                
                sample[0] = orientation.getA();
                sample[1] = orientation.getB();
                sample[2] = orientation.getC();
                sample[3] = orientation.getD();
                sample[4] = sample[5] = sample[6] = 0.0;
                sample[7] = deltaAccelerationX;
                sample[8] = deltaAccelerationY;
                sample[9] = deltaAccelerationZ;
                sample[10] = deltaAngularX;
                sample[11] = deltaAngularY;
                sample[12] = deltaAngularZ;
                estimator.update(sample);                                
            }
            
            lastAccelerationX = accelerationX;
            lastAccelerationY = accelerationY;
            lastAccelerationZ = accelerationZ;
                
            lastAngularX = angularX;
            lastAngularY = angularY;
            lastAngularZ = angularZ;
        }
        
        double[] mean = calibrator.getControlMean();
        double[] mean2 = estimator.getSampleAverage();
        
        assertArrayEquals(mean, mean2, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[0], 1.0, ABSOLUTE_ERROR);
        assertEquals(mean[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[3], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[4], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[5], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[6], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[7], accelerationOffsetX, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[8], accelerationOffsetY, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[9], accelerationOffsetZ, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[10], angularOffsetX, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[11], angularOffsetY, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[12], angularOffsetZ, LARGE_ABSOLUTE_ERROR);
        
        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertFalse(calibrator.isConverged());
        assertEquals(fullSampleReceived, N_SAMPLES);
        assertEquals(fullSampleProcessed, N_SAMPLES);
        assertEquals(calibratorFinished, 0);
        
        //add one last sample
        calibrator.updateAccelerometerSample(timestamp, (float)accelerationX, 
                (float)accelerationY, (float)accelerationZ);
        calibrator.updateGyroscopeSample(timestamp, (float)angularX, (float)angularY, 
                (float)angularZ);
        calibrator.updateOrientationSample(timestamp, orientation);

        //check
        assertFalse(calibrator.isFailed());
        assertTrue(calibrator.isFinished());
        assertFalse(calibrator.isConverged()); 
        assertEquals(fullSampleReceived, N_SAMPLES + 1);
        assertEquals(fullSampleProcessed, N_SAMPLES + 1);
        assertEquals(calibratorFinished, 1);        
    }
    
    @Test
    public void testCalibrationWithoutOffset() throws SignalProcessingException {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        calibrator.setListener(this);
        
        //setup so that calibrator doesn't reach convergence
        calibrator.setMaxNumSamples(N_SAMPLES);
        calibrator.setConvergenceThreshold(0.0);
        
        UniformRandomizer offsetRandomizer = new UniformRandomizer(
                new Random());
        GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, NOISE_DEVIATION);
        
        float accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        float accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        float accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        
        float angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        float angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        float angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        
        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
        
        float accelerationNoiseX, accelerationNoiseY, accelerationNoiseZ;
        float angularNoiseX, angularNoiseY, angularNoiseZ;
        
        double accelerationX, accelerationY, accelerationZ;
        double angularX, angularY, angularZ;
        double deltaAccelerationX, deltaAccelerationY, deltaAccelerationZ;
        double deltaAngularX, deltaAngularY, deltaAngularZ;
        double lastAccelerationX = 0.0, lastAccelerationY = 0.0, 
                lastAccelerationZ = 0.0;
        double lastAngularX = 0.0, lastAngularY = 0.0, lastAngularZ = 0.0;
        Quaternion orientation = new Quaternion();
                
        calibrator.reset();
        reset();
        
        MeasurementNoiseCovarianceEstimator estimator = 
                new MeasurementNoiseCovarianceEstimator(13);
        double[] sample = new double[13];
        
        for (int i = 0; i < N_SAMPLES; i++) {
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
            calibrator.updateOrientationSample(timestamp, orientation);
            
            timestamp += DELTA_NANOS;
            
            if (i != 0) {
                deltaAccelerationX = accelerationX - lastAccelerationX;
                deltaAccelerationY = accelerationY - lastAccelerationY;
                deltaAccelerationZ = accelerationZ - lastAccelerationZ;
                
                deltaAngularX = angularX - lastAngularX;
                deltaAngularY = angularY - lastAngularY;
                deltaAngularZ = angularZ - lastAngularZ;
                
                sample[0] = orientation.getA();
                sample[1] = orientation.getB();
                sample[2] = orientation.getC();
                sample[3] = orientation.getD();
                sample[4] = sample[5] = sample[6] = 0.0;
                sample[7] = deltaAccelerationX;
                sample[8] = deltaAccelerationY;
                sample[9] = deltaAccelerationZ;
                sample[10] = deltaAngularX;
                sample[11] = deltaAngularY;
                sample[12] = deltaAngularZ;                
                estimator.update(sample);                                
            }
            
            lastAccelerationX = accelerationX;
            lastAccelerationY = accelerationY;
            lastAccelerationZ = accelerationZ;
                
            lastAngularX = angularX;
            lastAngularY = angularY;
            lastAngularZ = angularZ;
        }
        
        double[] mean = calibrator.getControlMean();
        double[] mean2 = estimator.getSampleAverage();
        
        Matrix cov = calibrator.getControlCovariance();
        Matrix cov2 = estimator.getMeasurementNoiseCov();
        
        assertArrayEquals(mean, mean2, LARGE_ABSOLUTE_ERROR);
        assertEquals(mean[0], 1.0, ABSOLUTE_ERROR);
        assertEquals(mean[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[3], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[4], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[5], 0.0, ABSOLUTE_ERROR);
        assertEquals(mean[6], 0.0, ABSOLUTE_ERROR);
        
        assertTrue(cov.equals(cov2, ABSOLUTE_ERROR));
        
        assertFalse(calibrator.isFailed());
        assertFalse(calibrator.isFinished());
        assertFalse(calibrator.isConverged());
        assertEquals(fullSampleReceived, N_SAMPLES);
        assertEquals(fullSampleProcessed, N_SAMPLES);
        assertEquals(calibratorFinished, 0);
        
        //add one last sample
        calibrator.updateAccelerometerSample(timestamp, 0.0f, 0.0f, 0.0f);
        calibrator.updateGyroscopeSample(timestamp, 0.0f, 0.0f, 0.0f);
        calibrator.updateOrientationSample(timestamp, orientation);

        //check
        assertFalse(calibrator.isFailed());
        assertTrue(calibrator.isFinished());
        assertFalse(calibrator.isConverged()); 
        assertEquals(fullSampleReceived, N_SAMPLES + 1);
        assertEquals(fullSampleProcessed, N_SAMPLES + 1);
        assertEquals(calibratorFinished, 1);        
    }
    
    @Test
    public void testCalibrationConvergence() {
        AbsoluteOrientationSlamCalibrator calibrator = 
                new AbsoluteOrientationSlamCalibrator();
        calibrator.setListener(this);
                
        UniformRandomizer offsetRandomizer = new UniformRandomizer(
                new Random());
        GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, NOISE_DEVIATION);
        
        float accelerationOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        float accelerationOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        float accelerationOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        
        float angularOffsetX = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        float angularOffsetY = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        float angularOffsetZ = offsetRandomizer.nextFloat(MIN_OFFSET, 
                MAX_OFFSET);
        
        long timestamp = System.currentTimeMillis() * MILLIS_TO_NANOS;
        
        float accelerationNoiseX, accelerationNoiseY, accelerationNoiseZ;
        float angularNoiseX, angularNoiseY, angularNoiseZ;
        
        double accelerationX, accelerationY, accelerationZ;
        double angularX, angularY, angularZ;
        Quaternion orientation = new Quaternion();
                
        calibrator.reset();
        reset();
        
        for (int i = 0; i < N_SAMPLES; i++) {
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
            calibrator.updateOrientationSample(timestamp, orientation);
            
            if (calibrator.isFinished()) {
                break;
            }
            
            timestamp += DELTA_NANOS;            
        }
        
        assertTrue(calibrator.isConverged());
        assertTrue(calibrator.isFinished());
        assertFalse(calibrator.isFailed());
        
        assertTrue(fullSampleReceived < N_SAMPLES);
        assertTrue(fullSampleProcessed < N_SAMPLES);
        assertEquals(fullSampleReceived, fullSampleProcessed);
        assertEquals(calibratorFinished, 1);
    }        

    @Override
    public void onFullSampleReceived(BaseSlamCalibrator calibrator) {
        fullSampleReceived++;
    }

    @Override
    public void onFullSampleProcessed(BaseSlamCalibrator calibrator) {
        fullSampleProcessed++;
    }

    @Override
    public void onCalibratorFinished(BaseSlamCalibrator calibrator, 
            boolean converged, boolean failed) {
        calibratorFinished++;
    }

    private void reset() {
        fullSampleReceived = fullSampleProcessed = calibratorFinished = 0;
    }
}
