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
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class ConstantVelocityModelStatePredictorTest {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-7;

    private static final double JACOBIAN_ERROR = 1e-6;

    @Test
    void testPredict() throws WrongSizeException {
        final var randomizer = new UniformRandomizer(new Random());

        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        var q = new Quaternion(roll, pitch, yaw);

        final var wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var state = new double[]{
                x, y, z, q.getA(), q.getB(), q.getC(), q.getD(), vx, vy, vz, wx, wy, wz
        };

        final var u = new double[6];
        randomizer.fill(u, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var jacobianX1 = new Matrix(13, 13);
        final var jacobianU1 = new Matrix(13, 6);
        final var result1 = new double[13];
        ConstantVelocityModelStatePredictor.predict(state, u, dt, result1, jacobianX1, jacobianU1);

        // check correctness
        var r = new InhomogeneousPoint3D(x, y, z);
        final var rr = new Matrix(3, 3);
        final var rv = new Matrix(3, 3);
        r = PositionPredictor.predict(r, vx, vy, vz, 0.0, 0.0, 0.0, dt, rr, rv, null);

        final var qq = new Matrix(4, 4);
        final var qw = new Matrix(4, 3);
        q = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, qq, qw);

        final var result2 = new double[]{
                r.getInhomX(), r.getInhomY(), r.getInhomZ(), q.getA(), q.getB(), q.getC(), q.getD(),
                vx + u[0], vy + u[1], vz + u[2], wx + u[3], wy + u[4], wz + u[5]
        };

        assertArrayEquals(result1, result2, ABSOLUTE_ERROR);

        final var jacobianX2 = Matrix.identity(13, 13);
        jacobianX2.setSubmatrix(0, 0, 2, 2, rr);
        jacobianX2.setSubmatrix(3, 3, 6, 6, qq);
        jacobianX2.setSubmatrix(0, 7, 2, 9, rv);
        jacobianX2.setSubmatrix(3, 10, 6, 12, qw);

        final var jacobianU2 = new Matrix(13, 6);
        jacobianU2.setSubmatrix(7, 0, 12, 5,
                Matrix.identity(6, 6));

        assertTrue(jacobianX1.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU1.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predict(new double[1], u, dt, result1, jacobianX1,
                        jacobianU1));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predict(state, new double[1], dt, result1, jacobianX1,
                        jacobianU1));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predict(state, u, dt, new double[1], jacobianX1, jacobianU1));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> ConstantVelocityModelStatePredictor.predict(state, u, dt,
                result1, m, jacobianU1));
        assertThrows(IllegalArgumentException.class, () -> ConstantVelocityModelStatePredictor.predict(state, u, dt,
                result1, jacobianX1, m));

        // test without jacobians
        final var result3 = new double[13];
        ConstantVelocityModelStatePredictor.predict(state, u, dt, result3);

        // check correctness
        assertArrayEquals(result3, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ConstantVelocityModelStatePredictor.predict(new double[1],
                u, dt, result3));
        assertThrows(IllegalArgumentException.class, () -> ConstantVelocityModelStatePredictor.predict(state,
                new double[1], dt, result3));
        assertThrows(IllegalArgumentException.class, () -> ConstantVelocityModelStatePredictor.predict(state, u, dt,
                new double[1]));

        // test with new instance, with jacobians
        final var jacobianX3 = new Matrix(13, 13);
        final var jacobianU3 = new Matrix(13, 6);
        final var result4 = ConstantVelocityModelStatePredictor.predict(state, u, dt, jacobianX3, jacobianU3);

        // check correctness
        assertArrayEquals(result4, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianX3.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU3.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predict(new double[1], u, dt, jacobianX3, jacobianU3));
        assertThrows(IllegalArgumentException.class, () -> ConstantVelocityModelStatePredictor.predict(state,
                new double[1], dt, jacobianX3, jacobianU3));
        assertThrows(IllegalArgumentException.class, () -> ConstantVelocityModelStatePredictor.predict(state, u,
                dt, m, jacobianU3));
        assertThrows(IllegalArgumentException.class, () -> ConstantVelocityModelStatePredictor.predict(state, u,
                dt, jacobianX3, m));

        // test with new instance without jacobians
        final var result5 = ConstantVelocityModelStatePredictor.predict(state, u, dt);

        // check correctness
        assertArrayEquals(result5, result3, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> ConstantVelocityModelStatePredictor.predict(new double[1], u,
                dt));
        assertThrows(IllegalArgumentException.class, () -> ConstantVelocityModelStatePredictor.predict(state,
                new double[1], dt));

        // check correctness of jacobians
        final var jacobianX4 = new Matrix(13, 13);
        final var jacobianU4 = new Matrix(13, 6);
        final var result6 = ConstantVelocityModelStatePredictor.predict(state, u, dt, jacobianX4, jacobianU4);

        // check state variation
        var diff = new double[13];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final double[] state2 = ArrayUtils.sumAndReturnNew(state, diff);
        final var result7 = ConstantVelocityModelStatePredictor.predict(state2, u, dt);

        double[] diffResult = ArrayUtils.subtractAndReturnNew(result7, result6);
        double[] diffResult2 = jacobianX3.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check control variation
        diff = new double[6];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final double[] u2 = ArrayUtils.sumAndReturnNew(u, diff);
        final var result8 = ConstantVelocityModelStatePredictor.predict(state, u2, dt);

        diffResult = ArrayUtils.subtractAndReturnNew(result8, result6);
        diffResult2 = jacobianU4.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    void testPredictWithPositionAdjustment() throws WrongSizeException {
        final var randomizer = new UniformRandomizer(new Random());

        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        var q = new Quaternion(roll, pitch, yaw);

        final var wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var state = new double[]{
                x, y, z, q.getA(), q.getB(), q.getC(), q.getD(), vx, vy, vz, wx, wy, wz
        };

        final var u = new double[9];
        randomizer.fill(u, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var drx = u[0];
        final var dry = u[1];
        final var drz = u[2];

        var jacobianX1 = new Matrix(13, 13);
        var jacobianU1 = new Matrix(13, 9);
        final var result1 = new double[13];
        ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt, result1, jacobianX1,
                jacobianU1);

        // check correctness
        var r = new InhomogeneousPoint3D(x, y, z);
        final var rr = new Matrix(3, 3);
        final var rdr = new Matrix(3, 3);
        final var rv = new Matrix(3, 3);
        r = PositionPredictor.predictWithPositionAdjustment(r,
                drx, dry, drz, vx, vy, vz, 0.0, 0.0, 0.0, dt, rr, rdr, rv, null);

        final var qq = new Matrix(4, 4);
        final var qw = new Matrix(4, 3);
        q = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, qq, qw);

        var result2 = new double[]{
                r.getInhomX(), r.getInhomY(), r.getInhomZ(),
                q.getA(), q.getB(), q.getC(), q.getD(),
                vx + u[3], vy + u[4], vz + u[5],
                wx + u[6], wy + u[7], wz + u[8]
        };

        assertArrayEquals(result1, result2, ABSOLUTE_ERROR);

        final var jacobianX2 = Matrix.identity(13, 13);
        jacobianX2.setSubmatrix(0, 0, 2, 2, rr);
        jacobianX2.setSubmatrix(3, 3, 6, 6, qq);
        jacobianX2.setSubmatrix(0, 7, 2, 9, rv);
        jacobianX2.setSubmatrix(3, 10, 6, 12, qw);

        final var jacobianU2 = new Matrix(13, 9);
        jacobianU2.setSubmatrix(0, 0, 2, 2,
                Matrix.identity(3, 3));
        jacobianU2.setSubmatrix(7, 3, 12, 8,
                Matrix.identity(6, 6));

        assertTrue(jacobianX1.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU1.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(new double[1], u, dt, result1,
                        jacobianX1, jacobianU1));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, new double[1], dt,
                        result1, jacobianX1, jacobianU1));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt, new double[1],
                        jacobianX1, jacobianU1));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt, result1, m,
                        jacobianU1));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt, result1,
                        jacobianX1, m));

        // test without jacobians
        final var result3 = new double[13];
        ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt, result3);

        // check correctness
        assertArrayEquals(result3, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(new double[1], u, dt, result3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, new double[1], dt,
                        result3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt, new double[1]));

        // test with new instance, with jacobians
        final var jacobianX3 = new Matrix(13, 13);
        final var jacobianU3 = new Matrix(13, 9);
        final var result4 = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt, jacobianX3,
                jacobianU3);

        // check correctness
        assertArrayEquals(result4, result2, ABSOLUTE_ERROR);

        assertTrue(jacobianX3.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU3.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(new double[1], u, dt,
                        jacobianX3, jacobianU3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, new double[1], dt,
                        jacobianX3, jacobianU3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt, m, jacobianU3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt, jacobianX3, m));

        // test with new instance without jacobians
        final var result5 = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt);

        // check correctness
        assertArrayEquals(result5, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(new double[1], u, dt));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, new double[1], dt));

        // check correctness of jacobians
        final var jacobianX4 = new Matrix(13, 13);
        final var jacobianU4 = new Matrix(13, 9);
        final var result6 = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u, dt, jacobianX4,
                jacobianU4);

        // check state variation
        var diff = new double[13];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var state2 = ArrayUtils.sumAndReturnNew(state, diff);
        result2 = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state2, u, dt);

        var diffResult = ArrayUtils.subtractAndReturnNew(result2, result6);
        var diffResult2 = jacobianX4.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check control variation
        diff = new double[9];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var u2 = ArrayUtils.sumAndReturnNew(u, diff);
        result2 = ConstantVelocityModelStatePredictor.predictWithPositionAdjustment(state, u2, dt);

        diffResult = ArrayUtils.subtractAndReturnNew(result2, result6);
        diffResult2 = jacobianU4.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    void testPredictWithRotationAdjustment() throws WrongSizeException {
        final var randomizer = new UniformRandomizer(new Random());

        final var roll1 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var roll2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        var q = new Quaternion(roll1, pitch1, yaw1);
        final var dq = new Quaternion(roll2, pitch2, yaw2);

        final var wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var state = new double[]{
                x, y, z, q.getA(), q.getB(), q.getC(), q.getD(), vx, vy, vz, wx, wy, wz
        };

        final var u = new double[10];
        randomizer.fill(u, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        u[0] = dq.getA();
        u[1] = dq.getB();
        u[2] = dq.getC();
        u[3] = dq.getD();

        var jacobianX1 = new Matrix(13, 13);
        var jacobianU1 = new Matrix(13, 10);
        var result1 = new double[13];
        ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt, result1, jacobianX1,
                jacobianU1);

        // check correctness
        var r = new InhomogeneousPoint3D(x, y, z);
        final var rr = new Matrix(3, 3);
        final var rv = new Matrix(3, 3);
        r = PositionPredictor.predict(r, vx, vy, vz, dt, rr, rv, null);

        final var qq = new Matrix(4, 4);
        final var qw = new Matrix(4, 3);
        final var qdq = new Matrix(4, 4);
        q = QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx, wy, wz, dt, qq, qdq, qw);

        var result2 = new double[]{
                r.getInhomX(), r.getInhomY(), r.getInhomZ(), q.getA(), q.getB(), q.getC(), q.getD(),
                vx + u[4], vy + u[5], vz + u[6], wx + u[7], wy + u[8], wz + u[9]
        };

        assertArrayEquals(result1, result2, ABSOLUTE_ERROR);

        final var jacobianX2 = Matrix.identity(13, 13);
        jacobianX2.setSubmatrix(0, 0, 2, 2, rr);
        jacobianX2.setSubmatrix(3, 3, 6, 6, qq);
        jacobianX2.setSubmatrix(0, 7, 2, 9, rv);
        jacobianX2.setSubmatrix(3, 10, 6, 12, qw);

        final var jacobianU2 = new Matrix(13, 10);
        jacobianU2.setSubmatrix(3, 0, 6, 3, qdq);
        jacobianU2.setSubmatrix(7, 4, 12, 9,
                Matrix.identity(6, 6));

        assertTrue(jacobianX1.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU1.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(new double[1], u, dt, result1,
                        jacobianX1, jacobianU1));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, new double[1], dt,
                        result1, jacobianX1, jacobianU1));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt, new double[1],
                        jacobianX1, jacobianU1));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt, result1, m,
                        jacobianU1));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt, result1,
                        jacobianX1, m));

        // test without jacobians
        final var result3 = new double[13];
        ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt, result3);

        // check correctness
        assertArrayEquals(result3, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(new double[1], u, dt, result3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, new double[1], dt,
                        result3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt, new double[1]));

        // test with new instance, with jacobians
        final var jacobianX3 = new Matrix(13, 13);
        final var jacobianU3 = new Matrix(13, 10);
        final var result4 = ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt, jacobianX3,
                jacobianU3);

        // check correctness
        assertArrayEquals(result4, result2, ABSOLUTE_ERROR);

        assertTrue(jacobianX3.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU3.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(new double[1], u, dt,
                        jacobianX3, jacobianU3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, new double[1], dt,
                        jacobianX3, jacobianU3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt, m, jacobianU3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt, jacobianX3, m));

        // test with new instance, without jacobians
        final var result5 = ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt);

        // check correctness
        assertArrayEquals(result5, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(new double[1], u, dt));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, new double[1], dt));

        // check correctness of jacobians
        final var jacobianX4 = new Matrix(13, 13);
        final var jacobianU4 = new Matrix(13, 10);
        final var result6 = ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u, dt, jacobianX4,
                jacobianU4);

        // check state variation
        var diff = new double[13];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var state2 = ArrayUtils.sumAndReturnNew(state, diff);
        result2 = ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state2, u, dt);

        var diffResult = ArrayUtils.subtractAndReturnNew(result2, result6);
        var diffResult2 = jacobianX4.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check control variation
        diff = new double[10];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var u2 = ArrayUtils.sumAndReturnNew(u, diff);
        result2 = ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(state, u2, dt);

        diffResult = ArrayUtils.subtractAndReturnNew(result2, result6);
        diffResult2 = jacobianU4.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    void testPredictWithPositionAndRotationAdjustment() throws WrongSizeException {

        final var randomizer = new UniformRandomizer(new Random());

        final var roll1 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var roll2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        var q = new Quaternion(roll1, pitch1, yaw1);
        final var dq = new Quaternion(roll2, pitch2, yaw2);

        final var wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var state = new double[]{
                x, y, z, q.getA(), q.getB(), q.getC(), q.getD(), vx, vy, vz, wx, wy, wz
        };

        final var u = new double[13];
        randomizer.fill(u, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var drx = u[0];
        final var dry = u[1];
        final var drz = u[2];
        u[3] = dq.getA();
        u[4] = dq.getB();
        u[5] = dq.getC();
        u[6] = dq.getD();

        final var jacobianX1 = new Matrix(13, 13);
        final var jacobianU1 = new Matrix(13, 13);
        final var result1 = new double[13];
        ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u, dt, result1, jacobianX1,
                jacobianU1);

        // check correctness
        var r = new InhomogeneousPoint3D(x, y, z);
        final var rr = new Matrix(3, 3);
        final var rv = new Matrix(3, 3);
        r = PositionPredictor.predictWithPositionAdjustment(r,
                drx, dry, drz, vx, vy, vz, 0.0, 0.0, 0.0, dt, rr, null, rv, null);

        final var qq = new Matrix(4, 4);
        final var qw = new Matrix(4, 3);
        final var qdq = new Matrix(4, 4);
        q = QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx, wy, wz, dt, qq, qdq, qw);

        var result2 = new double[]{
                r.getInhomX(), r.getInhomY(), r.getInhomZ(), q.getA(), q.getB(), q.getC(), q.getD(),
                vx + u[7], vy + u[8], vz + u[9], wx + u[10], wy + u[11], wz + u[12]
        };

        assertArrayEquals(result1, result2, ABSOLUTE_ERROR);

        final var jacobianX2 = Matrix.identity(13, 13);
        jacobianX2.setSubmatrix(0, 0, 2, 2, rr);
        jacobianX2.setSubmatrix(3, 3, 6, 6, qq);
        jacobianX2.setSubmatrix(0, 7, 2, 9, rv);
        jacobianX2.setSubmatrix(3, 10, 6, 12, qw);

        final var jacobianU2 = new Matrix(13, 13);
        jacobianU2.setSubmatrix(0, 0, 2, 2,
                Matrix.identity(3, 3));
        jacobianU2.setSubmatrix(3, 3, 6, 6, qdq);
        jacobianU2.setSubmatrix(7, 7, 12, 12,
                Matrix.identity(6, 6));

        assertTrue(jacobianX1.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU1.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(new double[1], u,
                        dt, result1, jacobianX1, jacobianU1));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, new double[1],
                        dt, result1, jacobianX1, jacobianU1));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u, dt,
                        new double[1], jacobianX1, jacobianU1));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u, dt,
                        result1, m, jacobianU1));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u, dt,
                        result1, jacobianX1, m));

        // test without jacobians
        final var result3 = new double[13];
        ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u, dt, result3);

        // check correctness
        assertArrayEquals(result3, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(new double[1], u, dt,
                        result3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, new double[1],
                        dt, result3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u, dt,
                        new double[1]));

        // test with new instance, with jacobians
        final var jacobianX3 = new Matrix(13, 13);
        final var jacobianU3 = new Matrix(13, 13);
        final var result4 = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u, dt,
                jacobianX3, jacobianU3);

        // check correctness
        assertArrayEquals(result4, result2, ABSOLUTE_ERROR);

        assertTrue(jacobianX3.equals(jacobianX2, ABSOLUTE_ERROR));
        assertTrue(jacobianU3.equals(jacobianU2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(new double[1], u, dt,
                        jacobianX3, jacobianU3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, new double[1],
                        dt, jacobianX3, jacobianU3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u, dt,
                        m, jacobianU3));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u, dt,
                        jacobianX3, m));

        // test with new instance, without jacobians
        final var result5 = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u, dt);

        // check correctness
        assertArrayEquals(result5, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(new double[1], u,
                        dt));
        assertThrows(IllegalArgumentException.class,
                () -> ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, new double[1],
                        dt));

        // check correctness of jacobians
        final var jacobianX4 = new Matrix(13, 13);
        final var jacobianU4 = new Matrix(13, 13);
        final var result6 = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u, dt,
                jacobianX4, jacobianU4);

        // check state variation
        var diff = new double[13];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var state2 = ArrayUtils.sumAndReturnNew(state, diff);
        result2 = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state2, u, dt);

        var diffResult = ArrayUtils.subtractAndReturnNew(result2, result6);
        var diffResult2 = jacobianX4.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check control variation
        diff = new double[13];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var u2 = ArrayUtils.sumAndReturnNew(u, diff);
        result2 = ConstantVelocityModelStatePredictor.predictWithPositionAndRotationAdjustment(state, u2, dt);

        diffResult = ArrayUtils.subtractAndReturnNew(result2, result6);
        diffResult2 = jacobianU4.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }
}
