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

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class VelocityPredictorTest {
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double ABSOLUTE_ERROR = 1e-7;
    
    private static final double JACOBIAN_ERROR = 1e-6;
    
    public VelocityPredictorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testPredict() throws WrongSizeException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ax = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ay = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double az = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double[] v = new double[]{ vx, vy, vz };
        double[] a = new double[]{ ax, ay, az };        
        
        //test with parameters and jacobians
        double[] result = new double[3];
        Matrix jacobianV = new Matrix(3,3);
        Matrix jacobianA = new Matrix(3,3);
        VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, result,
                jacobianV, jacobianA);
        
        //check correctness
        double[] result2 = new double[] {
            vx + ax*dt,
            vy + ay*dt,
            vz + az*dt
        };
        
        Matrix jacobianV2 = Matrix.identity(3, 3);
        Matrix jacobianA2 = Matrix.identity(3, 3).
                multiplyByScalarAndReturnNew(dt);
        
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException 
        try {
            VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, 
                    new double[1], jacobianV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, result, 
                    new Matrix(1,1), jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, result, 
                    jacobianV, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with parameters, without jacobians
        result = new double[3];
        VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, result);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        try {
            VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, 
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        
        //test with arrays, with jacobians
        result = new double[3];
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        VelocityPredictor.predict(v, a, dt, result, jacobianV, jacobianA);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            VelocityPredictor.predict(new double[1], a, dt, result, 
                    jacobianV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predict(v, new double[1], dt, result, 
                    jacobianV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predict(v, a, dt, new double[1], jacobianV, 
                    jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predict(v, a, dt, result, new Matrix(1,1), 
                    jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predict(v, a, dt, result, jacobianV, 
                    new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        
        //test with arrays, without jacobians
        result = new double[3];
        VelocityPredictor.predict(v, a, dt, result);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        try {
            VelocityPredictor.predict(new double[1], a, dt, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predict(v, new double[1], dt, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predict(v, a, dt, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with new instance, with parameters and jacobians
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        result = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, 
                jacobianV, jacobianA);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException 
        result = null;
        try {
            result = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, 
                    new Matrix(1,1), jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, 
                    jacobianV, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, with parameters, without jacobians
        result = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        
        //test with new instance, with arrays, with jacobians
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        result = VelocityPredictor.predict(v, a, dt, jacobianV, 
                jacobianA);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = VelocityPredictor.predict(new double[1], a, dt, 
                    jacobianV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predict(v, new double[1], dt, 
                    jacobianV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predict(v, a, dt, new Matrix(1,1), 
                    jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predict(v, a, dt, jacobianV, 
                    new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, with arrays, without jacobians
        result = VelocityPredictor.predict(v, a, dt);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        //Force IllegalArgumentException
        result = null;
        try {
            result = VelocityPredictor.predict(new double[1], a, dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predict(v, new double[1], dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //check correctness of jacobians
        
        //check velocity variation
        result = VelocityPredictor.predict(v, a, dt);
        
        double[] diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] v2 = ArrayUtils.sumAndReturnNew(v, diff);
        result2 = VelocityPredictor.predict(v2, a, dt);
        
        double[] diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        double[] diffResult2 = jacobianV.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check acceleration variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] a2 = ArrayUtils.sumAndReturnNew(a, diff);
        result2 = VelocityPredictor.predict(v, a2, dt);
        
        diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        diffResult2 = jacobianA.multiplyAndReturnNew(Matrix.newFromArray(diff)).
                toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testPredictWithVelocityAdjustment() 
            throws WrongSizeException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double dvx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double dvy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double dvz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        double ax = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ay = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double az = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        
        double[] v = new double[] { vx, vy, vz };
        double[] dv = new double[] { dvx, dvy, dvz };
        double[] a = new double[] { ax, ay, az };
        
        //test with parameters and jacobians
        double[] result = new double[3];
        Matrix jacobianV = new Matrix(3,3);
        Matrix jacobianDV = new Matrix(3,3);
        Matrix jacobianA = new Matrix(3,3);
        VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz, 
                dvx, dvy, dvz, ax, ay, az, dt, result, 
                jacobianV, jacobianDV, jacobianA);
        
        //check correctness
        double[] result2 = new double[] {
            vx + dvx + ax*dt,
            vy + dvy + ay*dt,
            vz + dvz + az*dt
        };
        
        Matrix jacobianV2 = Matrix.identity(3, 3);
        Matrix jacobianDV2 = Matrix.identity(3, 3);
        Matrix jacobianA2 = Matrix.identity(3, 3).
                multiplyByScalarAndReturnNew(dt);
        
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianDV.equals(jacobianDV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz, 
                    dvx, dvy, dvz, ax, ay, az, dt, new double[1], jacobianV, 
                    jacobianDV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz, 
                    dvx, dvy, dvz, ax, ay, az, dt, result, new Matrix(1,1), 
                    jacobianDV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz, 
                    dvx, dvy, dvz, ax, ay, az, dt, result, jacobianV, 
                    new Matrix(1,1), jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz, 
                    dvx, dvy, dvz, ax, ay, az, dt, result, jacobianV, 
                    jacobianDV, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with parameters, without jacobians
        result = new double[3];
        VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz, 
                dvx, dvy, dvz, ax, ay, az, dt, result);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        try {
            VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz, 
                    dvx, dvy, dvz, ax, ay, az, dt, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with arrays, with jacobians
        result = new double[3];
        jacobianV = new Matrix(3,3);
        jacobianDV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt, 
                result, jacobianV, jacobianDV, jacobianA);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianDV.equals(jacobianDV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));

        //Force IllegalArgumentException
        try {
            VelocityPredictor.predictWithVelocityAdjustment(new double[1],
                    dv, a, dt, result, jacobianV, jacobianDV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predictWithVelocityAdjustment(v, 
                    new double[1], a, dt, result, jacobianV, jacobianDV, 
                    jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predictWithVelocityAdjustment(v, dv, 
                    new double[1], dt, result, jacobianV, jacobianDV, 
                    jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt, 
                    new double[1], jacobianV, jacobianDV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt, 
                    result, new Matrix(1,1), jacobianDV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt, 
                    result, jacobianV, new Matrix(1,1), jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt, 
                    result, jacobianV, jacobianDV, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with arrays, without jacobians
        result = new double[3];
        VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt, 
                result);        
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        //Force IllegalArgumentException
        try {
            VelocityPredictor.predictWithVelocityAdjustment(new double[1],
                    dv, a, dt, result);        
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predictWithVelocityAdjustment(v, 
                    new double[1], a, dt, result);        
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predictWithVelocityAdjustment(v, dv, 
                    new double[1], dt, result);        
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt, 
                    new double[1]);        
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with new instance, with parameters and jacobians
        jacobianV = new Matrix(3,3);
        jacobianDV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        result = VelocityPredictor.predictWithVelocityAdjustment(
                vx, vy, vz, dvx, dvy, dvz, ax, ay, az, dt, jacobianV, 
                jacobianDV, jacobianA);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianDV.equals(jacobianDV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = VelocityPredictor.predictWithVelocityAdjustment(
                    vx, vy, vz, dvx, dvy, dvz, ax, ay, az, dt, new Matrix(1,1), 
                    jacobianDV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predictWithVelocityAdjustment(
                    vx, vy, vz, dvx, dvy, dvz, ax, ay, az, dt, jacobianV, 
                    new Matrix(1,1), jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predictWithVelocityAdjustment(
                    vx, vy, vz, dvx, dvy, dvz, ax, ay, az, dt, jacobianV, 
                    jacobianDV, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, with parameters, without jacobians
        result = VelocityPredictor.predictWithVelocityAdjustment(
                vx, vy, vz, dvx, dvy, dvz, ax, ay, az, dt);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);  
        
        
        //test with new instance, with arrays, with jacobians
        jacobianV = new Matrix(3,3);
        jacobianDV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        result = VelocityPredictor.predictWithVelocityAdjustment(
                v, dv, a, dt, jacobianV, jacobianDV, jacobianA);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianDV.equals(jacobianDV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR)); 
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = VelocityPredictor.predictWithVelocityAdjustment(
                    new double[1], dv, a, dt, jacobianV, jacobianDV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predictWithVelocityAdjustment(
                    v, new double[1], a, dt, jacobianV, jacobianDV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predictWithVelocityAdjustment(
                    v, dv, new double[1], dt, jacobianV, jacobianDV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predictWithVelocityAdjustment(
                    v, dv, a, dt, new Matrix(1,1), jacobianDV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predictWithVelocityAdjustment(
                    v, dv, a, dt, jacobianV, new Matrix(1,1), jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predictWithVelocityAdjustment(
                    v, dv, a, dt, jacobianV, jacobianDV, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, with arrays, without jacobians
        result = VelocityPredictor.predictWithVelocityAdjustment(
                v, dv, a, dt);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);     
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = VelocityPredictor.predictWithVelocityAdjustment(
                    new double[1], dv, a, dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predictWithVelocityAdjustment(
                    v, new double[1], a, dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = VelocityPredictor.predictWithVelocityAdjustment(
                    v, dv, new double[1], dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
    }
    
}
