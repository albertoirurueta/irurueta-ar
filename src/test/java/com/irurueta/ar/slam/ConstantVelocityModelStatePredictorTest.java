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
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class ConstantVelocityModelStatePredictorTest {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-7;

    private static final double JACOBIAN_ERROR = 1e-6;

    @Test
    public void testPredict() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        Quaternion q = new Quaternion(roll, pitch, yaw);

        final double wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double[] state = new double[]{
                x, y, z, q.getA(), q.getB(), q.getC(), q.getD(), vx, vy, vz, wx, wy, wz
        };

        final double[] u = new double[6];
        randomizer.fill(u, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        Matrix jacobianX = new Matrix(13, 13);
        Matrix jacobianU = new Matrix(13, 6);
        double[] result = new double[13];
        ConstantVelocityModelStatePredictor.predict(state, u, dt, result, jacobianX, jacobianU);

        // check correctness
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x, y, z);
        final Matrix Rr = new Matrix(3, 3);
        final Matrix Rv = new Matrix(3, 3);
        r = PositionPredictor.predict(r, vx, vy, vz, 0.0, 0.0, 0.0, dt, Rr, Rv, null);

        final Matrix Qq = new Matrix(4, 4);
        final Matrix Qw = new Matrix(4, 3);
        q = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, Qq, Qw);

        double[] result2 = new double[]{
                r.getInhomX(), r.getInhomY(), r.getInhomZ(), q.getA(), q.getB(), q.getC(), q.getD(),
                vx + u[0], vy + u[1], vz + u[2], wx + u[3], wy + u[4], wz + u[5]
        };

        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        final Matrix jacobianX2 = Matrix.identity(13, 13);
        jacobianX2.setSubmatrix(0, 0, 2, 2, Rr);
        jacobianX2.setSubmatrix(3, 3, 6, 6, Qq);
        jacobianX2.setSubmatrix(0, 7, 2, 9, Rv);
        jacobianX2.setSubmatrix(3, 10, 6, 12, Qw);

        final Matrix jacobianU2 = new Matrix(13, 6);
        jacobianU2.setSubmatrix(7, 0, 12, 5,
                Matrix.identity(6, 6));

        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        try {
            ConstantVelocityModelStatePredictor.predict(new double[1], u, dt, result, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predict(state, new double[1], dt, result, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predict(state, u, dt, new double[1], jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predict(state, u, dt,
                    result, new Matrix(1, 1), jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predict(state, u, dt,
                    result, jacobianX, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }


        // test without jacobians
        result = new double[13];
        ConstantVelocityModelStatePredictor.predict(state, u, dt, result);

        // check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            ConstantVelocityModelStatePredictor.predict(new double[1], u, dt, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predict(state, new double[1], dt, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predict(state, u, dt, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // test with new instance, with jacobians
        jacobianX = new Matrix(13, 13);
        jacobianU = new Matrix(13, 6);
        result = ConstantVelocityModelStatePredictor.predict(state, u, dt, jacobianX, jacobianU);

        // check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        result = null;
        try {
            result = ConstantVelocityModelStatePredictor.predict(new double[1], u, dt, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.predict(state,
                    new double[1], dt, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.predict(state, u,
                    dt, new Matrix(1, 1), jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.predict(state, u,
                    dt, jacobianX, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(result);

        // test with new instance without jacobians
        result = ConstantVelocityModelStatePredictor.predict(state, u, dt);

        // check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        result = null;
        try {
            result = ConstantVelocityModelStatePredictor.predict(new double[1], u, dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.predict(state, new double[1], dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(result);

        // check correctness of jacobians
        jacobianX = new Matrix(13, 13);
        jacobianU = new Matrix(13, 6);
        result = ConstantVelocityModelStatePredictor.predict(state, u, dt, jacobianX, jacobianU);

        // check state variation
        double[] diff = new double[13];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final double[] state2 = ArrayUtils.sumAndReturnNew(state, diff);
        result2 = ConstantVelocityModelStatePredictor.predict(state2, u, dt);

        double[] diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        double[] diffResult2 = jacobianX.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check control variation
        diff = new double[6];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final double[] u2 = ArrayUtils.sumAndReturnNew(u, diff);
        result2 = ConstantVelocityModelStatePredictor.predict(state, u2, dt);

        diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        diffResult2 = jacobianU.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    public void testPredictWithPositionAdjustment()
            throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        Quaternion q = new Quaternion(roll, pitch, yaw);

        final double wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double[] state = new double[]{
                x, y, z, q.getA(), q.getB(), q.getC(), q.getD(), vx, vy, vz, wx, wy, wz
        };

        final double[] u = new double[9];
        randomizer.fill(u, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double drx = u[0];
        final double dry = u[1];
        final double drz = u[2];

        Matrix jacobianX = new Matrix(13, 13);
        Matrix jacobianU = new Matrix(13, 9);
        double[] result = new double[13];
        ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(
                state, u, dt, result, jacobianX, jacobianU);

        // check correctness
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x, y, z);
        final Matrix Rr = new Matrix(3, 3);
        final Matrix Rdr = new Matrix(3, 3);
        final Matrix Rv = new Matrix(3, 3);
        r = PositionPredictor.predictWithPositionAdjustment(r,
                drx, dry, drz, vx, vy, vz, 0.0, 0.0, 0.0, dt,
                Rr, Rdr, Rv, null);

        final Matrix Qq = new Matrix(4, 4);
        final Matrix Qw = new Matrix(4, 3);
        q = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, Qq, Qw);

        double[] result2 = new double[]{
                r.getInhomX(), r.getInhomY(), r.getInhomZ(),
                q.getA(), q.getB(), q.getC(), q.getD(),
                vx + u[3], vy + u[4], vz + u[5],
                wx + u[6], wy + u[7], wz + u[8]
        };

        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        final Matrix jacobianX2 = Matrix.identity(13, 13);
        jacobianX2.setSubmatrix(0, 0, 2, 2, Rr);
        jacobianX2.setSubmatrix(3, 3, 6, 6, Qq);
        jacobianX2.setSubmatrix(0, 7, 2, 9, Rv);
        jacobianX2.setSubmatrix(3, 10, 6, 12, Qw);

        final Matrix jacobianU2 = new Matrix(13, 9);
        jacobianU2.setSubmatrix(0, 0, 2, 2,
                Matrix.identity(3, 3));
        jacobianU2.setSubmatrix(7, 3, 12, 8,
                Matrix.identity(6, 6));

        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(
                    new double[1], u, dt, result, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(
                    state, new double[1], dt, result, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(
                    state, u, dt, new double[1], jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(
                    state, u, dt, result, new Matrix(1, 1), jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(
                    state, u, dt, result, jacobianX, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // test without jacobians
        result = new double[13];
        ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt, result);

        // check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(new double[1], u, dt, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, new double[1], dt, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // test with new instance, with jacobians
        jacobianX = new Matrix(13, 13);
        jacobianU = new Matrix(13, 9);
        result = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(
                state, u, dt, jacobianX, jacobianU);

        // check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        result = null;
        try {
            result = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(
                    new double[1], u, dt, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(
                    state, new double[1], dt, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(
                    state, u, dt, new Matrix(1, 1), jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(
                    state, u, dt, jacobianX, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(result);

        // test with new instance without jacobians
        result = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt);

        // check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        result = null;
        try {
            result = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(new double[1], u, dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.
                    predictWithPositionAdjustment(state, new double[1], dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(result);

        // check correctness of jacobians
        jacobianX = new Matrix(13, 13);
        jacobianU = new Matrix(13, 9);
        result = ConstantVelocityModelStatePredictor.
                predictWithPositionAdjustment(state, u, dt, jacobianX, jacobianU);

        // check state variation
        double[] diff = new double[13];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final double[] state2 = ArrayUtils.sumAndReturnNew(state, diff);
        result2 = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state2, u, dt);

        double[] diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        double[] diffResult2 = jacobianX.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check control variation
        diff = new double[9];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final double[] u2 = ArrayUtils.sumAndReturnNew(u, diff);
        result2 = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u2, dt);

        diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        diffResult2 = jacobianU.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    public void testPredictWithRotationAdjustment() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double roll2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        Quaternion q = new Quaternion(roll1, pitch1, yaw1);
        final Quaternion dq = new Quaternion(roll2, pitch2, yaw2);

        final double wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double[] state = new double[]{
                x, y, z, q.getA(), q.getB(), q.getC(), q.getD(), vx, vy, vz, wx, wy, wz
        };

        final double[] u = new double[10];
        randomizer.fill(u, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        u[0] = dq.getA();
        u[1] = dq.getB();
        u[2] = dq.getC();
        u[3] = dq.getD();

        Matrix jacobianX = new Matrix(13, 13);
        Matrix jacobianU = new Matrix(13, 10);
        double[] result = new double[13];
        ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(
                state, u, dt, result, jacobianX, jacobianU);

        // check correctness
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x, y, z);
        final Matrix Rr = new Matrix(3, 3);
        final Matrix Rv = new Matrix(3, 3);
        r = PositionPredictor.predict(r, vx, vy, vz, dt, Rr, Rv, null);

        final Matrix Qq = new Matrix(4, 4);
        final Matrix Qw = new Matrix(4, 3);
        final Matrix Qdq = new Matrix(4, 4);
        q = QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx, wy, wz, dt, Qq, Qdq, Qw);

        double[] result2 = new double[]{
                r.getInhomX(), r.getInhomY(), r.getInhomZ(), q.getA(), q.getB(), q.getC(), q.getD(),
                vx + u[4], vy + u[5], vz + u[6], wx + u[7], wy + u[8], wz + u[9]
        };

        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        final Matrix jacobianX2 = Matrix.identity(13, 13);
        jacobianX2.setSubmatrix(0, 0, 2, 2, Rr);
        jacobianX2.setSubmatrix(3, 3, 6, 6, Qq);
        jacobianX2.setSubmatrix(0, 7, 2, 9, Rv);
        jacobianX2.setSubmatrix(3, 10, 6, 12, Qw);

        final Matrix jacobianU2 = new Matrix(13, 10);
        jacobianU2.setSubmatrix(3, 0, 6, 3, Qdq);
        jacobianU2.setSubmatrix(7, 4, 12, 9,
                Matrix.identity(6, 6));

        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        try {
            ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(
                    new double[1], u, dt, result, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(
                    state, new double[1], dt, result, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(
                    state, u, dt, new double[1], jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(
                    state, u, dt, result, new Matrix(1, 1), jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(
                    state, u, dt, result, jacobianX, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // test without jacobians
        result = new double[13];
        ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt, result);

        // check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(new double[1], u, dt, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, new double[1], dt, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // test with new instance, with jacobians
        jacobianX = new Matrix(13, 13);
        jacobianU = new Matrix(13, 10);
        result = ConstantVelocityModelStatePredictor.
                predictWithRotationAdjustment(state, u, dt, jacobianX, jacobianU);

        // check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        result = null;
        try {
            result = ConstantVelocityModelStatePredictor.
                    predictWithRotationAdjustment(new double[1], u, dt, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.
                    predictWithRotationAdjustment(state, new double[1], dt, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.
                    predictWithRotationAdjustment(state, u, dt, new Matrix(1, 1), jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.
                    predictWithRotationAdjustment(state, u, dt, jacobianX, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(result);

        // test with new instance, without jacobians
        result = ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt);

        // check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        result = null;
        try {
            result = ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(new double[1], u, dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.
                    predictWithRotationAdjustment(state, new double[1], dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(result);

        // check correctness of jacobians
        jacobianX = new Matrix(13, 13);
        jacobianU = new Matrix(13, 10);
        result = ConstantVelocityModelStatePredictor.
                predictWithRotationAdjustment(state, u, dt, jacobianX, jacobianU);

        // check state variation
        double[] diff = new double[13];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final double[] state2 = ArrayUtils.sumAndReturnNew(state, diff);
        result2 = ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state2, u, dt);

        double[] diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        double[] diffResult2 = jacobianX.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check control variation
        diff = new double[10];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final double[] u2 = ArrayUtils.sumAndReturnNew(u, diff);
        result2 = ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u2, dt);

        diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        diffResult2 = jacobianU.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    public void testPredictWithPositionAndRotationAdjustment()
            throws WrongSizeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw1 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double roll2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double pitch2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double yaw2 = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        Quaternion q = new Quaternion(roll1, pitch1, yaw1);
        final Quaternion dq = new Quaternion(roll2, pitch2, yaw2);

        final double wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double[] state = new double[]{
                x, y, z, q.getA(), q.getB(), q.getC(), q.getD(), vx, vy, vz, wx, wy, wz
        };

        final double[] u = new double[13];
        randomizer.fill(u, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double drx = u[0];
        final double dry = u[1];
        final double drz = u[2];
        u[3] = dq.getA();
        u[4] = dq.getB();
        u[5] = dq.getC();
        u[6] = dq.getD();

        Matrix jacobianX = new Matrix(13, 13);
        Matrix jacobianU = new Matrix(13, 13);
        double[] result = new double[13];
        ConstantVelocityModelStatePredictor.
                predictWithPositionAndRotationAdjustment(state, u, dt, result, jacobianX, jacobianU);

        // check correctness
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x, y, z);
        final Matrix Rr = new Matrix(3, 3);
        final Matrix Rv = new Matrix(3, 3);
        r = PositionPredictor.predictWithPositionAdjustment(r,
                drx, dry, drz, vx, vy, vz, 0.0, 0.0, 0.0, dt, Rr, null, Rv,
                null);

        final Matrix Qq = new Matrix(4, 4);
        final Matrix Qw = new Matrix(4, 3);
        final Matrix Qdq = new Matrix(4, 4);
        q = QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx, wy, wz, dt, Qq, Qdq, Qw);

        double[] result2 = new double[]{
                r.getInhomX(), r.getInhomY(), r.getInhomZ(), q.getA(), q.getB(), q.getC(), q.getD(),
                vx + u[7], vy + u[8], vz + u[9], wx + u[10], wy + u[11], wz + u[12]
        };

        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        final Matrix jacobianX2 = Matrix.identity(13, 13);
        jacobianX2.setSubmatrix(0, 0, 2, 2, Rr);
        jacobianX2.setSubmatrix(3, 3, 6, 6, Qq);
        jacobianX2.setSubmatrix(0, 7, 2, 9, Rv);
        jacobianX2.setSubmatrix(3, 10, 6, 12, Qw);

        final Matrix jacobianU2 = new Matrix(13, 13);
        jacobianU2.setSubmatrix(0, 0, 2, 2,
                Matrix.identity(3, 3));
        jacobianU2.setSubmatrix(3, 3, 6, 6, Qdq);
        jacobianU2.setSubmatrix(7, 7, 12, 12,
                Matrix.identity(6, 6));

        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    new double[1], u, dt, result, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    state, new double[1], dt, result, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    state, u, dt, new double[1], jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    state, u, dt, result, new Matrix(1, 1), jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    state, u, dt, result, jacobianX, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // test without jacobians
        result = new double[13];
        ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u, dt, result);

        // check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    new double[1], u, dt, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    state, new double[1], dt, result);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    state, u, dt, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // test with new instance, with jacobians
        jacobianX = new Matrix(13, 13);
        jacobianU = new Matrix(13, 13);
        result = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                state, u, dt, jacobianX, jacobianU);

        // check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        assertTrue(jacobianX.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        result = null;
        try {
            result = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    new double[1], u, dt, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    state, new double[1], dt, jacobianX, jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    state, u, dt, new Matrix(1, 1), jacobianU);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    state, u, dt, jacobianX, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(result);

        // test with new instance, without jacobians
        result = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u, dt);

        // check correctness
        assertArrayEquals(result, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        result = null;
        try {
            result = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    new double[1], u, dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            result = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                    state, new double[1], dt);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(result);

        // check correctness of jacobians
        jacobianX = new Matrix(13, 13);
        jacobianU = new Matrix(13, 13);
        result = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(
                state, u, dt, jacobianX, jacobianU);

        // check state variation
        double[] diff = new double[13];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final double[] state2 = ArrayUtils.sumAndReturnNew(state, diff);
        result2 = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state2, u, dt);

        double[] diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        double[] diffResult2 = jacobianX.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check control variation
        diff = new double[13];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final double[] u2 = ArrayUtils.sumAndReturnNew(u, diff);
        result2 = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u2, dt);

        diffResult = ArrayUtils.subtractAndReturnNew(result2, result);
        diffResult2 = jacobianU.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }
}
