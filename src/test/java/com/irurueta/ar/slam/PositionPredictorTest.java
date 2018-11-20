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
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class PositionPredictorTest {
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double ABSOLUTE_ERROR = 1e-7;
    
    private static final double JACOBIAN_ERROR = 1e-6;
    
    
    public PositionPredictorTest() { }
    
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
        
        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ax, ay, az;
        do {
            ax = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            ay = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            az = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        } while (ax == 0.0 && ay == 0.0 && az == 0.0);
        double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x, y, z);
        double[] v = new double[]{ vx, vy, vz };
        double[] a = new double[]{ ax, ay, az };
        
        //test with all parameters and jacobians
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        Matrix jacobianR = new Matrix(3,3);
        Matrix jacobianV = new Matrix(3,3);
        Matrix jacobianA = new Matrix(3,3);
        PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt, result,
                jacobianR, jacobianV, jacobianA);
        
        //check correctness
        InhomogeneousPoint3D result2 = new InhomogeneousPoint3D(
                x + vx*dt + 0.5*ax*dt*dt, 
                y + vy*dt + 0.5*ay*dt*dt, 
                z + vz*dt + 0.5*az*dt*dt);
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        Matrix jacobianR2 = Matrix.identity(3, 3);
        Matrix jacobianV2 = jacobianR2.multiplyByScalarAndReturnNew(dt);
        Matrix jacobianA2 = jacobianR2.multiplyByScalarAndReturnNew(0.5*dt*dt);
        
        assertTrue(jacobianR.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt, 
                    result, new Matrix(1,1), jacobianV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt, 
                    result, jacobianR, new Matrix(1,1), jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt, 
                    result, jacobianR, jacobianV, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with all parameters without jacobians
        result = new InhomogeneousPoint3D();
        PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt, result);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        
        //test with arrays and jacobians
        result = new InhomogeneousPoint3D();
        jacobianR = new Matrix(3,3);
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        PositionPredictor.predict(r, v, a, dt, result, jacobianR, 
                jacobianV, jacobianA);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianR.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            PositionPredictor.predict(r, new double[1], a, dt, result, 
                    jacobianR, jacobianV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predict(r, v, new double[1], dt, result, 
                    jacobianR, jacobianV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predict(r, v, a, dt, result, 
                    new Matrix(1,1), jacobianV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predict(r, v, a, dt, result, jacobianR, 
                    new Matrix(1,1), jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predict(r, v, a, dt, result, jacobianR, 
                    jacobianV, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        
        //test with arrays and no jacobians
        result = new InhomogeneousPoint3D();
        PositionPredictor.predict(r, v, a, dt, result);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        
        //test with no acceleration parameters and jacobians
        result = new InhomogeneousPoint3D();
        jacobianR = new Matrix(3,3);
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        PositionPredictor.predict(r, vx, vy, vz, dt, result, jacobianR, 
                jacobianV, jacobianA);
        
        //check correctness
        InhomogeneousPoint3D result3 = new InhomogeneousPoint3D(x + vx*dt, 
                y + vy*dt, z + vz*dt);
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));
        
        //check jacobians
        Matrix jacobianR3 = Matrix.identity(3, 3);
        Matrix jacobianV3 = jacobianR2.multiplyByScalarAndReturnNew(dt);
        Matrix jacobianA3 = new Matrix(3,3);
        
        assertTrue(jacobianR.equals(jacobianR3, ABSOLUTE_ERROR));
        assertTrue(jacobianV.equals(jacobianV3, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA3, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            PositionPredictor.predict(r, vx, vy, vz, dt, result, 
                    new Matrix(1,1), jacobianV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predict(r, vx, vy, vz, dt, result, 
                    jacobianR, new Matrix(1,1), jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predict(r, vx, vy, vz, dt, result, 
                    jacobianR, jacobianV, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with no acceleration parameters and no jacobians
        result = new InhomogeneousPoint3D();
        PositionPredictor.predict(r, vx, vy, vz, dt, result);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));
        
        
        //test with no acceleration, v array, and jacobians
        result = new InhomogeneousPoint3D();
        jacobianR = new Matrix(3,3);
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        PositionPredictor.predict(r, v, dt, result, jacobianR, 
                jacobianV, jacobianA);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianR.equals(jacobianR3, ABSOLUTE_ERROR));
        assertTrue(jacobianV.equals(jacobianV3, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA3, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            PositionPredictor.predict(r, new double[1], dt, result, 
                    jacobianR, jacobianV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predict(r, v, dt, result, new Matrix(1,1), 
                    jacobianV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predict(r, v, dt, result, jacobianR, 
                    new Matrix(1,1), jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predict(r, v, dt, result, jacobianR, 
                    jacobianV, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        
        //test with no acceleration, v array, and no jacobians
        result = new InhomogeneousPoint3D();
        PositionPredictor.predict(r, v, dt, result);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            PositionPredictor.predict(r, new double[1], dt, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with new instance, with all parameters and jacobians
        jacobianR = new Matrix(3,3);
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        result = PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt, 
                jacobianR, jacobianV, jacobianA);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianR.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));

        //Force IllegalArgumentException
        result = null;
        try {
            result = PositionPredictor.predict(r, vx, vy, vz, 
                    ax, ay, az, dt, new Matrix(1,1), jacobianV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predict(r, vx, vy, vz, 
                    ax, ay, az, dt, jacobianR, new Matrix(1,1), jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predict(r, vx, vy, vz, 
                    ax, ay, az, dt, jacobianR, jacobianV, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        //test with new instance, with all parameters, without jacobians
        result = PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        
        //test with new instance, with arrays and jacobians
        jacobianR = new Matrix(3,3);
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        result = PositionPredictor.predict(r, v, a, dt, jacobianR, 
                jacobianV, jacobianA);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianR.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = PositionPredictor.predict(r, new double[1], a, dt, 
                    jacobianR, jacobianV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predict(r, v, new double[1], dt, 
                    jacobianR, jacobianV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predict(r, v, a, dt, 
                    new Matrix(1,1), jacobianV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predict(r, v, a, dt, jacobianR, 
                    new Matrix(1,1), jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predict(r, v, a, dt, jacobianR, 
                    jacobianV, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, with arrays and no jacobians
        result = PositionPredictor.predict(r, v, a, dt);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        
        //test with new instance, with no acceleration parameters and jacobians
        jacobianR = new Matrix(3,3);
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        result = PositionPredictor.predict(r, vx, vy, vz, dt, jacobianR, 
                jacobianV, jacobianA);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianR.equals(jacobianR3, ABSOLUTE_ERROR));
        assertTrue(jacobianV.equals(jacobianV3, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA3, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = PositionPredictor.predict(r, vx, vy, vz, dt,  
                    new Matrix(1,1), jacobianV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predict(r, vx, vy, vz, dt, 
                    jacobianR, new Matrix(1,1), jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predict(r, vx, vy, vz, dt, 
                    jacobianR, jacobianV, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, with no acceleration parameters and no 
        //jacobians
        result = PositionPredictor.predict(r, vx, vy, vz, dt);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));
        
        
        //test with new instance, with no acceleration, v array, and jacobians
        jacobianR = new Matrix(3,3);
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        result = PositionPredictor.predict(r, v, dt, jacobianR, jacobianV,
                jacobianA);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianR.equals(jacobianR3, ABSOLUTE_ERROR));
        assertTrue(jacobianV.equals(jacobianV3, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA3, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = PositionPredictor.predict(r, new double[1], dt, 
                    jacobianR, jacobianV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predict(r, v, dt, new Matrix(1,1), 
                    jacobianV, jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predict(r, v, dt, jacobianR, 
                    new Matrix(1,1), jacobianA);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predict(r, v, dt, jacobianR, 
                    jacobianV, new Matrix(1,1));            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, with no acceleration, v array, and no 
        //jacobians
        result = PositionPredictor.predict(r, v, dt);
        
        //check correctness
        assertTrue(result.equals(result3, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = PositionPredictor.predict(r, new double[1], dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);

        //check correctness of jacobians
        
        jacobianR = new Matrix(3,3);
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        result = PositionPredictor.predict(r, v, a, dt, jacobianR, 
                jacobianV, jacobianA);
        
        //check position variation
        double[] diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        InhomogeneousPoint3D r2 = new InhomogeneousPoint3D(
                r.getInhomX() + diff[0], r.getInhomY() + diff[1],
                r.getInhomZ() + diff[2]);
        result2 = PositionPredictor.predict(r2, v, a, dt);
        
        double[] diffResult = new double[]{
            result2.getInhomX() - result.getInhomX(),
            result2.getInhomY() - result.getInhomY(),
            result2.getInhomZ() - result.getInhomZ()
        };
        double[] diffResult2 = jacobianR.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check velocity variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] v2 = ArrayUtils.sumAndReturnNew(v, diff);
        result2 = PositionPredictor.predict(r, v2, a, dt);
        
        diffResult = new double[]{
            result2.getInhomX() - result.getInhomX(),
            result2.getInhomY() - result.getInhomY(),
            result2.getInhomZ() - result.getInhomZ()
        };
        diffResult2 = jacobianV.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check acceleration variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] a2 = ArrayUtils.sumAndReturnNew(a, diff);
        result2 = PositionPredictor.predict(r, v, a2, dt);
        
        diffResult = new double[]{
            result2.getInhomX() - result.getInhomX(),
            result2.getInhomY() - result.getInhomY(),
            result2.getInhomZ() - result.getInhomZ()
        };
        diffResult2 = jacobianA.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);        
    } 
    
    @Test
    public void testPredictWithPositionAdjustment() 
            throws WrongSizeException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double rx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ry = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double rz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double drx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double dry = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double drz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double ax = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double ay = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double az = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(rx, ry, rz);
        double[] dr = new double[]{ drx, dry, drz };
        double[] v = new double[]{ vx, vy, vz };
        double[] a = new double[]{ ax, ay, az };
        
        //test with all parameters and jacobians
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        Matrix jacobianR = new Matrix(3,3);
        Matrix jacobianDR = new Matrix(3,3);
        Matrix jacobianV = new Matrix(3,3);
        Matrix jacobianA = new Matrix(3,3);
        PositionPredictor.predictWithPositionAdjustment(r, drx, dry, drz, 
                vx, vy, vz, ax, ay, az, dt, result, jacobianR, jacobianDR, 
                jacobianV, jacobianA);
        
        //check correctness
        InhomogeneousPoint3D result2 = new InhomogeneousPoint3D(
                rx + drx + vx*dt + 0.5*ax*dt*dt,
                ry + dry + vy*dt + 0.5*ay*dt*dt,
                rz + drz + vz*dt + 0.5*az*dt*dt);
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        Matrix jacobianR2 = Matrix.identity(3, 3);
        Matrix jacobianDR2 = Matrix.identity(3, 3);
        Matrix jacobianV2 = jacobianR2.multiplyByScalarAndReturnNew(dt);
        Matrix jacobianA2 = jacobianR2.multiplyByScalarAndReturnNew(0.5*dt*dt);
        
        assertTrue(jacobianR.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianDR.equals(jacobianDR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));
        
        //force IllegalArgumentException
        try {
            PositionPredictor.predictWithPositionAdjustment(r, 
                    drx, dry, drz, vx, vy, vz, ax, ay, az, dt, result, 
                    new Matrix(1,1), jacobianDR, jacobianV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predictWithPositionAdjustment(r, 
                    drx, dry, drz, vx, vy, vz, ax, ay, az, dt, result, 
                    jacobianR, new Matrix(1,1), jacobianV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predictWithPositionAdjustment(r, 
                    drx, dry, drz, vx, vy, vz, ax, ay, az, dt, result, 
                    jacobianR, jacobianDR, new Matrix(1,1), jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predictWithPositionAdjustment(r, 
                    drx, dry, drz, vx, vy, vz, ax, ay, az, dt, result, 
                    jacobianR, jacobianDR, jacobianV, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with all parameters without jacobians
        result = new InhomogeneousPoint3D();
        PositionPredictor.predictWithPositionAdjustment(r, drx, dry, drz, 
                vx, vy, vz, ax, ay, az, dt, result);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        
        //test with arrays and jacobians
        result = new InhomogeneousPoint3D();
        jacobianR = new Matrix(3,3);
        jacobianDR = new Matrix(3,3);
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        PositionPredictor.predictWithPositionAdjustment(r, dr, v, a, dt, 
                result, jacobianR, jacobianDR, jacobianV, jacobianA);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianR.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianDR.equals(jacobianDR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));

        //Force IllegalArgumentException
        try {
            PositionPredictor.predictWithPositionAdjustment(r, 
                    new double[1], v, a, dt, result, jacobianR, jacobianDR, 
                    jacobianV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predictWithPositionAdjustment(r, dr, 
                    new double[1], a, dt, result, jacobianR, jacobianDR, 
                    jacobianV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predictWithPositionAdjustment(r, dr, v, 
                    new double[1], dt, result, jacobianR, jacobianDR, jacobianV, 
                    jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predictWithPositionAdjustment(r, dr, v, a, 
                    dt, result, new Matrix(1,1), jacobianDR, jacobianV, 
                    jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predictWithPositionAdjustment(r, dr, v, a, 
                    dt, result, jacobianR, new Matrix(1,1), jacobianV, 
                    jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predictWithPositionAdjustment(r, dr, v, a, 
                    dt, result, jacobianR, jacobianDR, new Matrix(1,1), 
                    jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predictWithPositionAdjustment(r, dr, v, a, 
                    dt, result, jacobianR, jacobianDR, jacobianV, 
                    new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        //test with arrays and no jacobians
        result = new InhomogeneousPoint3D();
        PositionPredictor.predictWithPositionAdjustment(r, dr, v, a, dt, 
                result);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try {
            PositionPredictor.predictWithPositionAdjustment(r, 
                    new double[1], v, a, dt, result);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predictWithPositionAdjustment(r, dr, 
                    new double[1], a, dt, result);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            PositionPredictor.predictWithPositionAdjustment(r, dr, v, 
                    new double[1], dt, result);            
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }

        
        //test with new instance, with all parameters and jacobians
        jacobianR = new Matrix(3,3);
        jacobianDR = new Matrix(3,3);
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        result = PositionPredictor.predictWithPositionAdjustment(r, 
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, 
                jacobianR, jacobianDR, jacobianV, jacobianA);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianR.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianDR.equals(jacobianDR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, 
                new Matrix(1,1), jacobianDR, jacobianV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, 
                jacobianR, new Matrix(1,1), jacobianV, jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, 
                jacobianR, jacobianDR, new Matrix(1,1), jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, 
                jacobianR, jacobianDR, jacobianV, new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, with all parameters and no jacobians
        result = PositionPredictor.predictWithPositionAdjustment(r, 
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        
        //test with new instance, with arrays and jacobians
        jacobianR = new Matrix(3,3);
        jacobianDR = new Matrix(3,3);
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        result = PositionPredictor.predictWithPositionAdjustment(r, dr, v, 
                a, dt, jacobianR, jacobianDR, jacobianV, jacobianA);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //check jacobians
        assertTrue(jacobianR.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianDR.equals(jacobianDR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA.equals(jacobianA2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                    new double[1], v, a, dt, jacobianR, jacobianDR, jacobianV, 
                    jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                    dr, new double[1], a, dt, jacobianR, jacobianDR, jacobianV, 
                    jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                    dr, v, new double[1], dt, jacobianR, jacobianDR, jacobianV, 
                    jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                    dr, v, a, dt, new Matrix(1,1), jacobianDR, jacobianV, 
                    jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                    dr, v, a, dt, jacobianR, new Matrix(1,1), jacobianV, 
                    jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                    dr, v, a, dt, jacobianR, jacobianDR, new Matrix(1,1), 
                    jacobianA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                    dr, v, a, dt, jacobianR, jacobianDR, jacobianV, 
                    new Matrix(1,1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //test with new instance, with arrays and no jacobians
        result = PositionPredictor.predictWithPositionAdjustment(r, dr, v, 
                a, dt);
        
        //check correctness
        assertTrue(result.equals(result2, ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        result = null;
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                    new double[1], v, a, dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                    dr, new double[1], a, dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            result = PositionPredictor.predictWithPositionAdjustment(r, 
                    dr, v, new double[1], dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(result);
        
        
        //check correctness of jacobians
        jacobianR = new Matrix(3,3);
        jacobianDR = new Matrix(3,3);
        jacobianV = new Matrix(3,3);
        jacobianA = new Matrix(3,3);
        result = PositionPredictor.predictWithPositionAdjustment(r, dr, v, 
                a, dt, jacobianR, jacobianDR, jacobianV, jacobianA);

        //check position variation
        double[] diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        InhomogeneousPoint3D r2 = new InhomogeneousPoint3D(
                r.getInhomX() + diff[0], r.getInhomY() + diff[1], 
                r.getInhomZ() + diff[2]);
        result2 = PositionPredictor.predictWithPositionAdjustment(r2, dr, 
                v, a, dt);
        
        double[] diffResult = new double[]{
            result2.getInhomX() - result.getInhomX(),
            result2.getInhomY() - result.getInhomY(),
            result2.getInhomZ() - result.getInhomZ()
        };
        double[] diffResult2 = jacobianR.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check position adjustment variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] dr2 = ArrayUtils.sumAndReturnNew(dr, diff);
        result2 = PositionPredictor.predictWithPositionAdjustment(r, dr2, 
                v, a, dt);
        
        diffResult = new double[]{
            result2.getInhomX() - result.getInhomX(),
            result2.getInhomY() - result.getInhomY(),
            result2.getInhomZ() - result.getInhomZ()            
        };
        diffResult2 = jacobianDR.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check velocity variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] v2 = ArrayUtils.sumAndReturnNew(v, diff);
        result2 = PositionPredictor.predictWithPositionAdjustment(r, dr, 
                v2, a, dt);
        
        diffResult = new double[]{
            result2.getInhomX() - result.getInhomX(),
            result2.getInhomY() - result.getInhomY(),
            result2.getInhomZ() - result.getInhomZ()
        };
        diffResult2 = jacobianV.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
        
        //check acceleration variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        double[] a2 = ArrayUtils.sumAndReturnNew(a, diff);
        result2 = PositionPredictor.predictWithPositionAdjustment(r, dr, 
                v, a2, dt);
        
        diffResult = new double[]{
            result2.getInhomX() - result.getInhomX(),
            result2.getInhomY() - result.getInhomY(),
            result2.getInhomZ() - result.getInhomZ()
        };
        diffResult2 = jacobianA.multiplyAndReturnNew(
                Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }
    
}
