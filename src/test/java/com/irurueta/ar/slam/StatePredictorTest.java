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
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class StatePredictorTest {
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;
    
    private static final double ABSOLUTE_ERROR = 1e-7;
    
    private static final double JACOBIAN_ERROR = 1e-6;
    
    
    public StatePredictorTest() { }
    
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
        
        double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ax = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ay = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double az = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                
        double[] state = new double[]{
            x, y, z,
            q.getA(), q.getB(), q.getC(), q.getD(),
            vx, vy, vz,
            ax, ay, az,
            wx, wy, wz
        };
        
        double[] u = new double[9];
        randomizer.fill(u, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Matrix jacobianX = new Matrix(16, 16);
        Matrix jacobianU = new Matrix(16, 9);
        double[] result = new double[16];
        StatePredictor.predict(state, u, dt,
                result, jacobianX, jacobianU);
        
        //check correctness
        Matrix Vv = new Matrix(3,3);
        Matrix Va = new Matrix(3,3);
        double[] v = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt,
                Vv, Va);
        
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x,y,z);
        Matrix Rr = new Matrix(3,3);
        Matrix Rv = new Matrix(3,3);
        Matrix Ra = new Matrix(3,3);
        r = PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt, Rr,
                Rv, Ra);
        
        Matrix Qq = new Matrix(4,4);
        Matrix Qw = new Matrix(4,3);
        q = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, Qq, Qw);
        
        double[] result2 = new double[] {
            r.getInhomX(), r.getInhomY(), r.getInhomZ(),
            q.getA(), q.getB(), q.getC(), q.getD(),
            v[0] + u[0], v[1] + u[1], v[2] + u[2],
            ax + u[3], ay + u[4], az + u[5],
            wx + u[6], wy + u[7], wz + u[8]
        };
        
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        Matrix jacobianX2 = Matrix.identity(16, 16);
        jacobianX2.setSubmatrix(0, 0, 2, 2, Rr);
        jacobianX2.setSubmatrix(3, 3, 6, 6, Qq);
        jacobianX2.setSubmatrix(0, 7, 2, 9, Rv);
        jacobianX2.setSubmatrix(7, 7, 9, 9, Vv);
        jacobianX2.setSubmatrix(0, 10, 2, 12, Ra);
        jacobianX2.setSubmatrix(7, 10, 9, 12, Va);
        jacobianX2.setSubmatrix(3, 13, 6, 15, Qw);
        
        Matrix jacobianU2 = new Matrix(16,9);
        jacobianU2.setSubmatrix(7, 0, 15, 8, Matrix.identity(9, 9));
        
        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            StatePredictor.predict(new double[1],
                    u, dt, result, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predict(state, 
                    new double[1], dt, result, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predict(state, u, dt, 
                    new double[1], jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predict(state, u, dt, 
                    result, new Matrix(1,1), jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predict(state, u, dt, 
                    result, jacobianX, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test without jacobians
        result = new double[16];
        StatePredictor.predict(state, u, dt, 
                result);   
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        //Force IllegalArgumentException
        try {
            StatePredictor.predict(new double[1],
                    u, dt, result);               
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predict(state, 
                    new double[1], dt, result);               
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predict(state, u, dt, 
                    new double[1]);               
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        
        //test with new instance, with jacobians
        jacobianX = new Matrix(16, 16);
        jacobianU = new Matrix(16, 9);
        result = StatePredictor.predict(state, u,
                dt, jacobianX, jacobianU);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = StatePredictor.predict(
                    new double[1], u, dt, jacobianX, jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predict(
                    state, new double[1], dt, jacobianX, jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predict(
                    state, u, dt, new Matrix(1,1), jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predict(
                    state, u, dt, jacobianX, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);

        
        //test with new instance without jacobians
        result = StatePredictor.predict(state, u,
                dt);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);        
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = StatePredictor.predict(
                    new double[1], u, dt);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predict(
                    state, new double[1], dt);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //check correctness of jacobians
        jacobianX = new Matrix(16, 16);
        jacobianU = new Matrix(16, 9);
        result = StatePredictor.predict(state, u,
                dt, jacobianX, jacobianU);

        //check state variation
        double[] diff = new double[16];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] state2 = ArrayUtils.sumAndReturnNew(state, diff);
        result2 = StatePredictor.predict(state2, 
                u, dt);
        
        double[] diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        double[] diffResult2 = jacobianX.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check control variation
        diff = new double[9];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] u2 = ArrayUtils.sumAndReturnNew(u, diff);
        result2 = StatePredictor.predict(state, 
                u2, dt);

        diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        diffResult2 = jacobianU.multiplyAndReturnNew(Matrix.newFromArray(diff)).
                toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);        
    }
    
    @Test
    public void testPredictWithPositionAdjustment() throws WrongSizeException {
        
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
        
        double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ax = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ay = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double az = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                
        double[] state = new double[]{
            x, y, z,
            q.getA(), q.getB(), q.getC(), q.getD(),
            vx, vy, vz,
            ax, ay, az,
            wx, wy, wz
        };
        
        double[] u = new double[12];
        randomizer.fill(u, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double drx = u[0];
        double dry = u[1];
        double drz = u[2];
        
        Matrix jacobianX = new Matrix(16, 16);
        Matrix jacobianU = new Matrix(16, 12);
        double[] result = new double[16];
        StatePredictor.predictWithPositionAdjustment(state, u, dt, 
                result, jacobianX, jacobianU);
        
        //check correctness
        Matrix Vv = new Matrix(3,3);
        Matrix Va = new Matrix(3,3);
        double[] v = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, 
                Vv, Va);
        
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x,y,z);
        Matrix Rr = new Matrix(3,3);
        Matrix Rv = new Matrix(3,3);
        Matrix Ra = new Matrix(3,3);
        r = PositionPredictor.predictWithPositionAdjustment(r, 
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, Rr, null,
                Rv, Ra);
        
        Matrix Qq = new Matrix(4,4);
        Matrix Qw = new Matrix(4,3);
        q = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, Qq, Qw);
        
        double[] result2 = new double[] {
            r.getInhomX(), r.getInhomY(), r.getInhomZ(),
            q.getA(), q.getB(), q.getC(), q.getD(),
            v[0] + u[3], v[1] + u[4], v[2] + u[5],
            ax + u[6], ay + u[7], az + u[8],
            wx + u[9], wy + u[10], wz + u[11]
        };
        
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        Matrix jacobianX2 = Matrix.identity(16, 16);
        jacobianX2.setSubmatrix(0, 0, 2, 2, Rr);
        jacobianX2.setSubmatrix(3, 3, 6, 6, Qq);
        jacobianX2.setSubmatrix(0, 7, 2, 9, Rv);
        jacobianX2.setSubmatrix(7, 7, 9, 9, Vv);
        jacobianX2.setSubmatrix(0, 10, 2, 12, Ra);
        jacobianX2.setSubmatrix(7, 10, 9, 12, Va);
        jacobianX2.setSubmatrix(3, 13, 6, 15, Qw);
        
        Matrix jacobianU2 = new Matrix(16,12);
        jacobianU2.setSubmatrix(0, 0, 2, 2, Matrix.identity(3, 3));
        jacobianU2.setSubmatrix(7, 3, 15, 11, Matrix.identity(9, 9));
        
        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            StatePredictor.predictWithPositionAdjustment(
                    new double[1], u, dt, result, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithPositionAdjustment(
                    state, new double[1], dt, result, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithPositionAdjustment(
                    state, u, dt, new double[1], jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithPositionAdjustment(
                    state, u, dt, result, new Matrix(1,1), jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithPositionAdjustment(
                    state, u, dt, result, jacobianX, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test without jacobians
        result = new double[16];
        StatePredictor.predictWithPositionAdjustment(state, u, dt, 
                result);
        
        //check correctness     
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        try {
            StatePredictor.predictWithPositionAdjustment(
                    new double[1], u, dt, result);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithPositionAdjustment(
                    state, new double[1], dt, result);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithPositionAdjustment(
                    state, u, dt, new double[1]);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with new instance and jacobians
        jacobianX = new Matrix(16, 16);
        jacobianU = new Matrix(16, 12);
        result = StatePredictor.predictWithPositionAdjustment(
                        state, u, dt, jacobianX, jacobianU);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = StatePredictor.predictWithPositionAdjustment(
                        new double[1], u, dt, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predictWithPositionAdjustment(
                        state, new double[1], dt, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predictWithPositionAdjustment(
                        state, u, dt, new Matrix(1,1), jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predictWithPositionAdjustment(
                        state, u, dt, jacobianX, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, without jacobians
        result = StatePredictor.predictWithPositionAdjustment(
                        state, u, dt);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = StatePredictor.predictWithPositionAdjustment(
                new double[1], u, dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predictWithPositionAdjustment(
                state, new double[1], dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);

        
        //check correctness of jacobians
        jacobianX = new Matrix(16,16);
        jacobianU = new Matrix(16,12);
        result = StatePredictor.predictWithPositionAdjustment(
                state, u, dt, jacobianX, jacobianU);

        //check state variation
        double[] diff = new double[16];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] state2 = ArrayUtils.sumAndReturnNew(state, diff);
        result2 = StatePredictor.predictWithPositionAdjustment(
                        state2, u, dt);
        
        double[] diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        double[] diffResult2 = jacobianX.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check control variation
        diff = new double[12];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] u2 = ArrayUtils.sumAndReturnNew(u, diff);
        result2 = StatePredictor.predictWithPositionAdjustment(
                        state, u2, dt);
        
        diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        diffResult2 = jacobianU.multiplyAndReturnNew(Matrix.newFromArray(diff)).
                toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);        
    }
    
    @Test
    public void testConstantAccelerationModelPredictStateWithRotationAdjustment() 
            throws WrongSizeException {
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
        
        double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ax = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ay = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double az = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
       
        double[] state = new double[] {
            x, y, z,
            q.getA(), q.getB(), q.getC(), q.getD(),
            vx, vy, vz,
            ax, ay, az,
            wx, wy, wz
        };
        
        double[] u = new double[13];
        randomizer.fill(u, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        u[0] = dq.getA();
        u[1] = dq.getB();
        u[2] = dq.getC();
        u[3] = dq.getD();
        
        Matrix jacobianX = new Matrix(16,16);
        Matrix jacobianU = new Matrix(16,13);
        double[] result = new double[16];
        StatePredictor.predictWithRotationAdjustment(
                state, u, dt, result, jacobianX, jacobianU);
        
        //check correctness
        Matrix Vv = new Matrix(3,3);
        Matrix Va = new Matrix(3,3);
        double[] v = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, 
                Vv, Va);
        
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x,y,z);
        Matrix Rr = new Matrix(3,3);
        Matrix Rv = new Matrix(3,3);
        Matrix Ra = new Matrix(3,3);
        r = PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt, Rr, 
                Rv, Ra);
        
        Matrix Qq = new Matrix(4,4);
        Matrix Qw = new Matrix(4,3);
        Matrix Qdq = new Matrix(4,4);
        q = QuaternionPredictor.predictWithRotationAdjustment(q, dq, 
                wx, wy, wz, dt, Qq, Qdq, Qw);
        
        double[] result2 = new double[] {
            r.getInhomX(), r.getInhomY(), r.getInhomZ(),
            q.getA(), q.getB(), q.getC(), q.getD(),
            v[0] + u[4], v[1] + u[5], v[2] + u[6],
            ax + u[7], ay + u[8], az + u[9],
            wx + u[10], wy + u[11], wz + u[12]
        };
        
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        Matrix jacobianX2 = Matrix.identity(16, 16);
        jacobianX2.setSubmatrix(0, 0, 2, 2, Rr);
        jacobianX2.setSubmatrix(3, 3, 6, 6, Qq);
        jacobianX2.setSubmatrix(0, 7, 2, 9, Rv);
        jacobianX2.setSubmatrix(7, 7, 9, 9, Vv);
        jacobianX2.setSubmatrix(0, 10, 2, 12, Ra);
        jacobianX2.setSubmatrix(7, 10, 9, 12, Va);
        jacobianX2.setSubmatrix(3, 13, 6, 15, Qw);
        
        Matrix jacobianU2 = new Matrix(16,13);
        jacobianU2.setSubmatrix(3, 0, 6, 3, Qdq);
        jacobianU2.setSubmatrix(7, 4, 15, 12, Matrix.identity(9, 9));
        
        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            StatePredictor.predictWithRotationAdjustment(
                    new double[1], u, dt, result, jacobianX, jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithRotationAdjustment(
                    state, new double[1], dt, result, jacobianX, jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithRotationAdjustment(
                    state, u, dt, new double[1], jacobianX, jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithRotationAdjustment(
                    state, u, dt, result, new Matrix(1,1), jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithRotationAdjustment(
                    state, u, dt, result, jacobianX, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test without jacobians
        result = new double[16];
        StatePredictor.predictWithRotationAdjustment(
                state, u, dt, result);   
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        try {
            StatePredictor.predictWithRotationAdjustment(
                            new double[1], u, dt, result);               
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithRotationAdjustment(
                            state, new double[1], dt, result);               
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithRotationAdjustment(
                            state, u, dt, new double[1]);               
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with new instance, with jacobians
        jacobianX = new Matrix(16,16);
        jacobianU = new Matrix(16,13);
        result = StatePredictor.predictWithRotationAdjustment(
                state, u, dt, jacobianX, jacobianU);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));

        //Force IllegalArgumentException
        result = null;
        try {
            result = StatePredictor.predictWithRotationAdjustment(
                    new double[1], u, dt, jacobianX, jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predictWithRotationAdjustment(
                    state, new double[1], dt, jacobianX, jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predictWithRotationAdjustment(
                    state, u, dt, new Matrix(1,1), jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predictWithRotationAdjustment(
                    state, u, dt, jacobianX, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, without jacobians
        result = StatePredictor.predictWithRotationAdjustment(
                state, u, dt);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);  
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = StatePredictor.predictWithRotationAdjustment(
                    new double[1], u, dt);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predictWithRotationAdjustment(
                    state, new double[1], dt);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //check correctness of jacobians
        jacobianX = new Matrix(16,16);
        jacobianU = new Matrix(16,13);
        result = StatePredictor.predictWithRotationAdjustment(
                state, u, dt, jacobianX, jacobianU);

        //check state variation
        double[] diff = new double[16];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] state2 = ArrayUtils.sumAndReturnNew(state, diff);
        result2 = StatePredictor.predictWithRotationAdjustment(
                        state2, u, dt);    
        
        double[] diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        double[] diffResult2 = jacobianX.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check control variation
        diff = new double[13];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] u2 = ArrayUtils.sumAndReturnNew(u, diff);
        result2 = StatePredictor.predictWithRotationAdjustment(
                        state, u2, dt);
        
        diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        diffResult2 = jacobianU.multiplyAndReturnNew(Matrix.newFromArray(diff)).
                toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);                        
    }
    
    @Test
    public void testConstantAccelerationModelPredictStateWithPositionAndRotationAdjustment() 
            throws WrongSizeException {
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
        
        double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ax = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ay = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double az = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
       
        double[] state = new double[] {
            x, y, z,
            q.getA(), q.getB(), q.getC(), q.getD(),
            vx, vy, vz,
            ax, ay, az,
            wx, wy, wz
        };
        
        double[] u = new double[16];
        randomizer.fill(u, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double drx = u[0];
        double dry = u[1];
        double drz = u[2];
        u[3] = dq.getA();
        u[4] = dq.getB();
        u[5] = dq.getC();
        u[6] = dq.getD();
        
        Matrix jacobianX = new Matrix(16,16);
        Matrix jacobianU = new Matrix(16,16);
        double[] result = new double[16];
        StatePredictor.predictWithPositionAndRotationAdjustment(
                state, u, dt, result, jacobianX, jacobianU);
        
        //check correctness
        Matrix Vv = new Matrix(3,3);
        Matrix Va = new Matrix(3,3);
        double[] v = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, 
                Vv, Va);
        
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x,y,z);
        Matrix Rr = new Matrix(3,3);
        Matrix Rv = new Matrix(3,3);
        Matrix Ra = new Matrix(3,3);
        r = PositionPredictor.predictWithPositionAdjustment(r, 
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, Rr, null,
                Rv, Ra);
        
        Matrix Qq = new Matrix(4,4);
        Matrix Qw = new Matrix(4,3);
        Matrix Qdq = new Matrix(4,4);
        q = QuaternionPredictor.predictWithRotationAdjustment(q, dq, 
                wx, wy, wz, dt, Qq, Qdq, Qw);
        
        double[] result2 = new double[] {
            r.getInhomX(), r.getInhomY(), r.getInhomZ(),
            q.getA(), q.getB(), q.getC(), q.getD(),
            v[0] + u[7], v[1] + u[8], v[2] + u[9],
            ax + u[10], ay + u[11], az + u[12],
            wx + u[13], wy + u[14], wz + u[15]
        };
        
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        Matrix jacobianX2 = Matrix.identity(16, 16);
        jacobianX2.setSubmatrix(0, 0, 2, 2, Rr);
        jacobianX2.setSubmatrix(3, 3, 6, 6, Qq);
        jacobianX2.setSubmatrix(0, 7, 2, 9, Rv);
        jacobianX2.setSubmatrix(7, 7, 9, 9, Vv);
        jacobianX2.setSubmatrix(0, 10, 2, 12, Ra);
        jacobianX2.setSubmatrix(7, 10, 9, 12, Va);
        jacobianX2.setSubmatrix(3, 13, 6, 15, Qw);
        
        Matrix jacobianU2 = Matrix.identity(16,16);
        jacobianU2.setSubmatrix(3, 3, 6, 6, Qdq);
        
        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            StatePredictor.predictWithPositionAndRotationAdjustment(
                    new double[1], u, dt, result, jacobianX, jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithPositionAndRotationAdjustment(
                    state, new double[1], dt, result, jacobianX, jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithPositionAndRotationAdjustment(
                    state, u, dt, new double[1], jacobianX, jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithPositionAndRotationAdjustment(
                    state, u, dt, result, new Matrix(1,1), jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithPositionAndRotationAdjustment(
                    state, u, dt, result, jacobianX, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test without jacobians
        result = new double[16];
        StatePredictor.predictWithPositionAndRotationAdjustment(
                state, u, dt, result);   
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        try {
            StatePredictor.predictWithPositionAndRotationAdjustment(
                            new double[1], u, dt, result);               
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithPositionAndRotationAdjustment(
                            state, new double[1], dt, result);               
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            StatePredictor.predictWithPositionAndRotationAdjustment(
                            state, u, dt, new double[1]);               
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with new instance, with jacobians
        jacobianX = new Matrix(16,16);
        jacobianU = new Matrix(16,16);
        result = StatePredictor.predictWithPositionAndRotationAdjustment(
                state, u, dt, jacobianX, jacobianU);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        
        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));

        //Force IllegalArgumentException
        result = null;
        try {
            result = StatePredictor.predictWithPositionAndRotationAdjustment(
                    new double[1], u, dt, jacobianX, jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predictWithPositionAndRotationAdjustment(
                    state, new double[1], dt, jacobianX, jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predictWithPositionAndRotationAdjustment(
                    state, u, dt, new Matrix(1,1), jacobianU);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predictWithPositionAndRotationAdjustment(
                    state, u, dt, jacobianX, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, without jacobians
        result = StatePredictor.predictWithPositionAndRotationAdjustment(
                state, u, dt);
        
        //check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);  
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = StatePredictor.predictWithPositionAndRotationAdjustment(
                    new double[1], u, dt);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = StatePredictor.predictWithPositionAndRotationAdjustment(
                    state, new double[1], dt);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //check correctness of jacobians
        jacobianX = new Matrix(16,16);
        jacobianU = new Matrix(16,16);
        result = StatePredictor.predictWithPositionAndRotationAdjustment(
                state, u, dt, jacobianX, jacobianU);

        //check state variation
        double[] diff = new double[16];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] state2 = ArrayUtils.sumAndReturnNew(state, diff);
        result2 = StatePredictor.predictWithPositionAndRotationAdjustment(
                        state2, u, dt);    
        
        double[] diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        double[] diffResult2 = jacobianX.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check control variation
        diff = new double[16];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] u2 = ArrayUtils.sumAndReturnNew(u, diff);
        result2 = StatePredictor.predictWithPositionAndRotationAdjustment(
                        state, u2, dt);
        
        diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        diffResult2 = jacobianU.multiplyAndReturnNew(Matrix.newFromArray(diff)).
                toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);                                
    }
}
