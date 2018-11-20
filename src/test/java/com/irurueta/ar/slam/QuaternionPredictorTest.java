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
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.geometry.RotationUtils;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class QuaternionPredictorTest {
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;
    
    private static final double ABSOLUTE_ERROR = 1e-7;
    
    private static final double JACOBIAN_ERROR = 1e-6;
    
    public QuaternionPredictorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testPredict() throws WrongSizeException, 
            RotationException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Quaternion q = new Quaternion(roll, pitch, yaw);
        
        double wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double[] w = new double[]{ wx, wy, wz };
        
        double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //test with parameters and jacobians
        
        //test exact method
        Quaternion result = new Quaternion();
        Matrix jacobianQ = new Matrix(4,4);
        Matrix jacobianW = new Matrix(4,3);
        QuaternionPredictor.predict(q, wx, wy, wz, dt, true, result,
                jacobianQ, jacobianW);
        
        //check correctness
        double[] dtw = new double[]{ dt * wx, dt * wy, dt * wz };
        Quaternion tmp = new Quaternion();        
        Matrix jacobianW2 = new Matrix(4,3);
        Quaternion.rotationVectorToQuaternion(dtw, tmp, jacobianW2);
        jacobianW2.multiplyByScalar(dt);
        Quaternion result2 = new Quaternion();
        Matrix jacobianQ2 = new Matrix(4,4);
        Matrix jacobianQ2W = new Matrix(4,4);
        Quaternion.product(q, tmp, result2, jacobianQ2, jacobianQ2W);
        
        jacobianW2 = jacobianQ2W.multiplyAndReturnNew(jacobianW2);

        
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));
        
        //test tustin method
        result = new Quaternion();
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);
        QuaternionPredictor.predict(q, wx, wy, wz, dt, false, result, 
                jacobianQ, jacobianW);
                
        Matrix W = RotationUtils.w2omega(w);
        Matrix Q = Matrix.newFromArray(q.getValues());
        W.multiply(Q);
        W.multiplyByScalar(0.5 * dt);
        
        Quaternion result3 = new Quaternion(q.getA() + W.getElementAtIndex(0), 
                q.getB() + W.getElementAtIndex(1), 
                q.getC() + W.getElementAtIndex(2), 
                q.getD() + W.getElementAtIndex(3));
        
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        
    
        //check jacobians
        W = RotationUtils.w2omega(w);
        Matrix jacobianQ3 = Matrix.identity(4,4).addAndReturnNew(W).
                multiplyByScalarAndReturnNew(0.5 * dt);
        
        Matrix jacobianW3 = RotationUtils.quaternionToConjugatedPiMatrix(q).
                multiplyByScalarAndReturnNew(0.5 * dt);
        
        assertTrue(jacobianQ.equals(jacobianQ3, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW3, ABSOLUTE_ERROR));

        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, wx, wy, wz, dt, false, result, 
                new Matrix(1,1), jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predict(q, wx, wy, wz, dt, false, result, 
                jacobianQ, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with w array and jacobians
        
        //test exact method
        QuaternionPredictor.predict(q, w, dt, true, result, jacobianQ, 
                jacobianW);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));

        //test tustin method
        QuaternionPredictor.predict(q, w, dt, false, result, jacobianQ, 
                jacobianW);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        
    
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ3, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW3, ABSOLUTE_ERROR));

        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, new double[1], dt, true, 
                    result, jacobianQ, jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predict(q, w, dt, true, result, 
                    new Matrix(1,1), jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predict(q, w, dt, true, result, 
                    jacobianQ, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        
        
        //test with parameters, exact method and jacobians
        result = new Quaternion();
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);
        QuaternionPredictor.predict(q, wx, wy, wz, dt, result, jacobianQ, 
                jacobianW);

        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
    
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));

        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, wx, wy, wz, dt, result, 
                    new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predict(q, wx, wy, wz, dt, result, 
                    jacobianQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }

        
        //test with array, tustin method and jacobians
        result = new Quaternion();
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);
        QuaternionPredictor.predict(q, w, dt, result, jacobianQ, 
                jacobianW);

        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
    
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));

        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, new double[1], dt, result, 
                    jacobianQ, jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predict(q, w, dt, result, new Matrix(1,1), 
                    jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predict(q, w, dt, result, jacobianQ, 
                    new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        
        
        //test with parameters, without jacobians
        
        //test exact method
        result = new Quaternion();
        QuaternionPredictor.predict(q, wx, wy, wz, dt, true, result);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
        
        //test tustin method
        result = new Quaternion();
        QuaternionPredictor.predict(q, wx, wy, wz, dt, false, result);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        
        
        
        //test with array, without jacobians
        
        //test exact method
        result = new Quaternion();
        QuaternionPredictor.predict(q, w, dt, true, result);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));    
        
        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, new double[1], dt, true, 
                    result);
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }

        
        //test tustin method
        result = new Quaternion();
        QuaternionPredictor.predict(q, w, dt, false, result);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        

        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, new double[1], dt, false, 
                    result);
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        
        
        //test with parameters, exact method, without jacobians
        result = new Quaternion();
        QuaternionPredictor.predict(q, wx, wy, wz, dt, result);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
        
        
        //test with array, exact method, without jacobians
        result = new Quaternion();
        QuaternionPredictor.predict(q, w, dt, result);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
        
        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, new double[1], dt, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }

        
        //test with new instance, parameters and jacobians
        
        //test exact method        
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, 
                jacobianQ, jacobianW);
                
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        

        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, 
                    new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, 
                    jacobianQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        assertNull(result);

        
        //test tustin method        
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, wx, wy, wz, dt, false, 
                jacobianQ, jacobianW);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        
        
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ3, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW3, ABSOLUTE_ERROR));                
        
        result = null;
        try {
            result = QuaternionPredictor.predict(q, wx, wy, wz, dt, false, 
                    new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, wx, wy, wz, dt, false, 
                    jacobianQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, with array and jacobians
        
        //test exact method        
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, w, dt, true, jacobianQ, 
                jacobianW);
                
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        

        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, new double[1], dt, 
                    true, jacobianQ, jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, w, dt, true, 
                    new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, w, dt, true, 
                    jacobianQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        assertNull(result);
        
        //test tustin method        
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, w, dt, false, jacobianQ, 
                jacobianW);
                
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        

        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ3, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW3, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, new double[1], dt, 
                    false, jacobianQ, jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, w, dt, false, 
                    new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, w, dt, false, 
                    jacobianQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, parameters, exact method and jacobians
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, wx, wy, wz, dt, jacobianQ, 
                jacobianW);
                
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        

        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, wx, wy, wz, dt, 
                    new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, wx, wy, wz, dt, 
                    jacobianQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        assertNull(result);

        
        //test with new instance, array, exact method and jacobians
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, w, dt, jacobianQ, 
                jacobianW);
                
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        

        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, new double[1], dt, 
                    jacobianQ, jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, w, dt, 
                    new Matrix(1,1), jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, w, dt, jacobianQ, 
                    new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        assertNull(result);

        
        //test with new instance, with parameters and no jacobians
        
        //test exact method
        result = QuaternionPredictor.predict(q, wx, wy, wz, dt, true);
                
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
        
        //test tustin method
        result = QuaternionPredictor.predict(q, wx, wy, wz, dt, false);
                
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        
        
        
        //test with new instance, array and no jacobians

        //test exact method
        result = QuaternionPredictor.predict(q, w, dt, true);
                
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, new double[1], dt, 
                    true);
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        assertNull(result);
        
        //test tustin method
        result = QuaternionPredictor.predict(q, w, dt, false);
                
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, new double[1], dt, 
                    false);
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        assertNull(result);   
        
        
        //test with new instance, with parameters, with exact method, without 
        //jacobians
        result = QuaternionPredictor.predict(q, wx, wy, wz, dt);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));                
        
        
        //test with new instance, array, exact method and no jacobians
        result = QuaternionPredictor.predict(q, w, dt);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));                
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, new double[1], dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch(IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //check correctness of jacobians (for exact method)
        
        //check quaternion variation
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, w, dt, true, jacobianQ, 
                jacobianW);
        
        double[] diff = new double[4];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        Quaternion q2 = new Quaternion(q.getA() + diff[0], q.getB() + diff[1],
                q.getC() + diff[2], q.getD() + diff[3]);
        result2 = QuaternionPredictor.predict(q2, w, dt, true);
        
        double[] diffResult = new double[]{ 
            result2.getA() - result.getA(), 
            result2.getB() - result.getB(),
            result2.getC() - result.getC(),
            result2.getD() - result.getD()
        };
        double[] diffResult2 = jacobianQ.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check angular speed variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] w2 = ArrayUtils.sumAndReturnNew(w, diff);
        result2 = QuaternionPredictor.predict(q, w2, dt, true);
        
        diffResult = new double[]{ 
            result2.getA() - result.getA(), 
            result2.getB() - result.getB(),
            result2.getC() - result.getC(),
            result2.getD() - result.getD()
        };
        diffResult2 = jacobianW.multiplyAndReturnNew(Matrix.newFromArray(diff)).
                toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testPredictDefaultDt() throws WrongSizeException, 
            RotationException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Quaternion q = new Quaternion(roll, pitch, yaw);
        
        double wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double[] w = new double[]{ wx, wy, wz };
        
        double dt = 1.0;
        
        //test with parameters and jacobians
        
        //test exact method
        Quaternion result = new Quaternion();
        Matrix jacobianQ = new Matrix(4,4);
        Matrix jacobianW = new Matrix(4,3);
        QuaternionPredictor.predict(q, wx, wy, wz, true, result, 
                jacobianQ, jacobianW);
        
        //check correctness
        double[] dtw = new double[]{ dt * wx, dt * wy, dt * wz };
        Quaternion tmp = new Quaternion();        
        Matrix jacobianW2 = new Matrix(4,3);
        Quaternion.rotationVectorToQuaternion(dtw, tmp, jacobianW2);
        jacobianW2.multiplyByScalar(dt);
        Quaternion result2 = new Quaternion();
        Matrix jacobianQ2 = new Matrix(4,4);
        Matrix jacobianQ2W = new Matrix(4,4);
        Quaternion.product(q, tmp, result2, jacobianQ2, jacobianQ2W);
        
        jacobianW2 = jacobianQ2W.multiplyAndReturnNew(jacobianW2);
        
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));
        
        //test tustin method
        result = new Quaternion();
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);
        QuaternionPredictor.predict(q, wx, wy, wz, false, result, 
                jacobianQ, jacobianW);
                
        Matrix W = RotationUtils.w2omega(w);
        Matrix Q = Matrix.newFromArray(q.getValues());
        W.multiply(Q);
        W.multiplyByScalar(0.5 * dt);
        
        Quaternion result3 = new Quaternion(q.getA() + W.getElementAtIndex(0), 
                q.getB() + W.getElementAtIndex(1), 
                q.getC() + W.getElementAtIndex(2), 
                q.getD() + W.getElementAtIndex(3));
        
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        
    
        //check jacobians
        W = RotationUtils.w2omega(w);
        Matrix jacobianQ3 = Matrix.identity(4,4).addAndReturnNew(W).
                multiplyByScalarAndReturnNew(0.5 * dt);
        
        Matrix jacobianW3 = RotationUtils.quaternionToConjugatedPiMatrix(q).
                multiplyByScalarAndReturnNew(0.5 * dt);
        
        assertTrue(jacobianQ.equals(jacobianQ3, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW3, ABSOLUTE_ERROR));

        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, wx, wy, wz, false, result, 
                new Matrix(1,1), jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predict(q, wx, wy, wz, false, result, 
                jacobianQ, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with w array and jacobians
        
        //test exact method
        QuaternionPredictor.predict(q, w, true, result, jacobianQ, 
                jacobianW);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));

        //test tustin method
        QuaternionPredictor.predict(q, w, false, result, jacobianQ, 
                jacobianW);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        
    
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ3, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW3, ABSOLUTE_ERROR));

        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, new double[1], true, 
                    result, jacobianQ, jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predict(q, w, true, result, 
                    new Matrix(1,1), jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predict(q, w, true, result, 
                    jacobianQ, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with parameters, exact method and jacobians
        result = new Quaternion();
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);
        QuaternionPredictor.predict(q, wx, wy, wz, result, jacobianQ, 
                jacobianW);

        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
    
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));

        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, wx, wy, wz, result, 
                    new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predict(q, wx, wy, wz, result, 
                    jacobianQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        
        //test with array, exact method and jacobians
        result = new Quaternion();
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);
        QuaternionPredictor.predict(q, w, result, jacobianQ, 
                jacobianW);

        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
    
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));

        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, new double[1], result, 
                    jacobianQ, jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predict(q, w, result, new Matrix(1,1), 
                    jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predict(q, w, result, jacobianQ, 
                    new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with parameters, without jacobians
        
        //test exact method
        result = new Quaternion();
        QuaternionPredictor.predict(q, wx, wy, wz, true, result);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
        
        //test tustin method
        result = new Quaternion();
        QuaternionPredictor.predict(q, wx, wy, wz, false, result);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        
        
        
        //test with array, without jacobians
        
        //test exact method
        result = new Quaternion();
        QuaternionPredictor.predict(q, w, true, result);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));    
        
        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, new double[1], true, 
                    result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        
        //test tustin method
        result = new Quaternion();
        QuaternionPredictor.predict(q, w, false, result);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        

        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, new double[1], false, 
                    result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with parameters, exact method, without jacobians
        result = new Quaternion();
        QuaternionPredictor.predict(q, wx, wy, wz, result);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
        
        
        //test with array, exact method, without jacobians
        result = new Quaternion();
        QuaternionPredictor.predict(q, w, result);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
        
        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predict(q, new double[1], result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        
        //test with new instance, parameters and jacobians
        
        //test exact method        
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, wx, wy, wz, true, 
                jacobianQ, jacobianW);
                
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        

        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, wx, wy, wz, true, 
                    new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, wx, wy, wz, true, 
                    jacobianQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);

        
        //test tustin method        
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, wx, wy, wz, false, 
                jacobianQ, jacobianW);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        
        
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ3, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW3, ABSOLUTE_ERROR));                
        
        result = null;
        try {
            result = QuaternionPredictor.predict(q, wx, wy, wz, false, 
                    new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, wx, wy, wz, false, 
                    jacobianQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, with array and jacobians
        
        //test exact method        
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, w, true, jacobianQ, 
                jacobianW);
                
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        

        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, new double[1], 
                    true, jacobianQ, jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, w, true, 
                    new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, w, true, 
                    jacobianQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        //test tustin method        
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, w, false, jacobianQ, 
                jacobianW);
                
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        

        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ3, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW3, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, new double[1], 
                    false, jacobianQ, jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, w, false, 
                    new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, w, false, 
                    jacobianQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, parameters, exact method and jacobians
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, wx, wy, wz, jacobianQ, 
                jacobianW);
                
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        

        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, wx, wy, wz, 
                    new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, wx, wy, wz, 
                    jacobianQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);

        
        //test with new instance, array, exact method and jacobians
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, w, jacobianQ, 
                jacobianW);
                
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        

        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, new double[1], 
                    jacobianQ, jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, w, 
                    new Matrix(1,1), jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predict(q, w, jacobianQ, 
                    new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);

        
        //test with new instance, with parameters and no jacobians
        
        //test exact method
        result = QuaternionPredictor.predict(q, wx, wy, wz, true);
                
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
        
        //test tustin method
        result = QuaternionPredictor.predict(q, wx, wy, wz, false);
                
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        
        
        
        //test with new instance, array and no jacobians

        //test exact method
        result = QuaternionPredictor.predict(q, w, true);
                
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));        
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, new double[1], true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        //test tustin method
        result = QuaternionPredictor.predict(q, w, false);
                
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));        
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, new double[1], false);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);   
        
        
        //test with new instance, with parameters, with exact method, without 
        //jacobians
        result = QuaternionPredictor.predict(q, wx, wy, wz);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));                
        
        
        //test with new instance, array, exact method and no jacobians
        result = QuaternionPredictor.predict(q, w);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));                
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predict(q, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result); 
        
        //check correctness of jacobians (for exact method)
        
        //check quaternion variation
        jacobianQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);        
        result = QuaternionPredictor.predict(q, w, true, jacobianQ, 
                jacobianW);
        
        double[] diff = new double[4];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        Quaternion q2 = new Quaternion(q.getA() + diff[0], q.getB() + diff[1],
                q.getC() + diff[2], q.getD() + diff[3]);
        result2 = QuaternionPredictor.predict(q2, w, true);
        
        double[] diffResult = new double[] {
            result2.getA() - result.getA(), 
            result2.getB() - result.getB(),
            result2.getC() - result.getC(),
            result2.getD() - result.getD()
        };
        double[] diffResult2 = jacobianQ.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check angular speed variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] w2 = ArrayUtils.sumAndReturnNew(w, diff);
        result2 = QuaternionPredictor.predict(q, w2, true);
        
        diffResult = new double[] {
            result2.getA() - result.getA(), 
            result2.getB() - result.getB(),
            result2.getC() - result.getC(),
            result2.getD() - result.getD()
        };
        diffResult2 = jacobianW.multiplyAndReturnNew(Matrix.newFromArray(diff)).
                toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);        
    }    
    
    @Test
    public void testPredictWithRotationAdjustment() 
            throws WrongSizeException, RotationException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double roll1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double pitch1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double yaw1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        double roll2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double pitch2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        double yaw2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        Quaternion q = new Quaternion(roll1, pitch1, yaw1);
        Quaternion dq = new Quaternion(roll2, pitch2, yaw2);
        
        double wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double[] w = new double[]{ wx, wy, wz };
        
        double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //test with parameters and jacobians
        
        //test exact method
        Quaternion result = new Quaternion();
        Matrix jacobianQ = new Matrix(4,4);
        Matrix jacobianDQ = new Matrix(4,4);
        Matrix jacobianW = new Matrix(4,3);
        QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx, wy, 
                wz, dt, result, jacobianQ, jacobianDQ, jacobianW);
        
        //check correctness
        double[] dtw = new double[]{ dt * wx, dt * wy, dt * wz };
        Quaternion tmp = new Quaternion();
        Matrix jacobianW2 = new Matrix(4,3);
        Quaternion.rotationVectorToQuaternion(dtw, tmp, jacobianW2);        
        Matrix jacobianQb = new Matrix(4,4);
        Matrix jacobianDQ2 = new Matrix(4,4);
        Quaternion tmp2 = new Quaternion();
        Quaternion.product(dq, tmp, tmp2, jacobianDQ2, jacobianQb);
        
        Matrix jacobianQ2 = new Matrix(4,4);
        Matrix jacobianQ3 = new Matrix(4,4);
        Quaternion result2 = new Quaternion();
        Quaternion.product(q, tmp2, result2, jacobianQ2, jacobianQ3);
                
        jacobianDQ2 = jacobianQ3.multiplyAndReturnNew(jacobianDQ2);                    
        jacobianW2 = jacobianQ3.multiplyAndReturnNew(jacobianQb).
                multiplyAndReturnNew(jacobianW2);
        jacobianW2.multiplyByScalar(dt);
        
        
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianDQ.equals(jacobianDQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx, 
                    wy, wz, dt, result, new Matrix(1,1), jacobianDQ, jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx, 
                    wy, wz, dt, result, jacobianQ, new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx, 
                    wy, wz, dt, result, jacobianQ, jacobianDQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with w array and jacobians
        result = new Quaternion();
        jacobianQ = new Matrix(4,4);
        jacobianDQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);
        QuaternionPredictor.predictWithRotationAdjustment(q, dq, w, dt, 
                result, jacobianQ, jacobianDQ, jacobianW);
        
        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predictWithRotationAdjustment(q, dq, 
                    new double[1], dt, result, jacobianQ, jacobianDQ, 
                    jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predictWithRotationAdjustment(q, dq, w, 
                    dt, result, new Matrix(1,1), jacobianDQ, jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predictWithRotationAdjustment(q, dq, w, 
                    dt, result, jacobianQ, new Matrix(1,1), jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            QuaternionPredictor.predictWithRotationAdjustment(q, dq, w, 
                    dt, result, jacobianQ, jacobianDQ, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianDQ.equals(jacobianDQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));

        
        //test without jacobians
        result = new Quaternion();
        QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx, wy, 
                wz, dt, result);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        
        //test with w array without jacobians
        result = new Quaternion();
        QuaternionPredictor.predictWithRotationAdjustment(q, dq, w, dt, 
                result);        
        
        //Force IllegalArgumentException
        try {
            QuaternionPredictor.predictWithRotationAdjustment(q, dq, 
                    new double[1], dt, result);        
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with new instance
        result = QuaternionPredictor.predictWithRotationAdjustment(q, dq,
                wx, wy, wz, dt, jacobianQ, jacobianDQ, jacobianW);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianDQ.equals(jacobianDQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predictWithRotationAdjustment(q, 
                    dq, wx, wy, wz, dt, new Matrix(1,1), jacobianDQ, jacobianW);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predictWithRotationAdjustment(q,
                    dq, wx, wy, wz, dt, jacobianQ, new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predictWithRotationAdjustment(q,
                    dq, wx, wy, wz, dt, jacobianQ, jacobianDQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance and array
        result = QuaternionPredictor.predictWithRotationAdjustment(q, dq, 
                w, dt, jacobianQ, jacobianDQ, jacobianW);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianQ.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianDQ.equals(jacobianDQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW.equals(jacobianW2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predictWithRotationAdjustment(q,
                    dq, new double[1], dt, jacobianQ, jacobianDQ, jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predictWithRotationAdjustment(q,
                    dq, w, dt, new Matrix(1,1), jacobianDQ, jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predictWithRotationAdjustment(q,
                    dq, w, dt, jacobianQ, new Matrix(1,1), jacobianW);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = QuaternionPredictor.predictWithRotationAdjustment(q,
                    dq, w, dt, jacobianQ, jacobianDQ, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance without jacobians
        result = QuaternionPredictor.predictWithRotationAdjustment(q, dq, 
                wx, wy, wz, dt);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        
        //test with new instance, with array and without jacobians
        result = QuaternionPredictor.predictWithRotationAdjustment(q, dq, 
                w, dt);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
                
        //Force IllegalArgumentException
        result = null;
        try {
            result = QuaternionPredictor.predictWithRotationAdjustment(q, 
                    dq, new double[1], dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //check correctness of jacobians        
        jacobianQ = new Matrix(4,4);
        jacobianDQ = new Matrix(4,4);
        jacobianW = new Matrix(4,3);
        result = QuaternionPredictor.predictWithRotationAdjustment(q, dq, 
                w, dt, jacobianQ, jacobianDQ, jacobianW);
        
        double[] diff = new double[4];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        Quaternion q2 = new Quaternion(q.getA() + diff[0], q.getB() + diff[1],
                q.getC() + diff[2], q.getD() + diff[3]);
        result2 = QuaternionPredictor.predictWithRotationAdjustment(q2, 
                dq, w, dt);
        
        double[] diffResult = new double[]{
            result2.getA() - result.getA(),
            result2.getB() - result.getB(),
            result2.getC() - result.getC(),
            result2.getD() - result.getD()
        };
        double[] diffResult2 = jacobianQ.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check rotation variation
        diff = new double[4];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        Quaternion dq2 = new Quaternion(dq.getA() + diff[0], 
                dq.getB() + diff[1], dq.getC() + diff[2], dq.getD() + diff[3]);
        result2 = QuaternionPredictor.predictWithRotationAdjustment(q, 
                dq2, w, dt);
        
        diffResult = new double[]{
            result2.getA() - result.getA(),
            result2.getB() - result.getB(),
            result2.getC() - result.getC(),
            result2.getD() - result.getD()            
        };
        diffResult2 = jacobianDQ.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check angular speed variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] w2 = ArrayUtils.sumAndReturnNew(w, diff);
        result2 = QuaternionPredictor.predictWithRotationAdjustment(q, dq,
                w2, dt);
        
        diffResult = new double[]{ 
            result2.getA() - result.getA(), 
            result2.getB() - result.getB(),
            result2.getC() - result.getC(),
            result2.getD() - result.getD()
        };
        diffResult2 = jacobianW.multiplyAndReturnNew(Matrix.newFromArray(diff)).
                toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);        
    }
    
}
