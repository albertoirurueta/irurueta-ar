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
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class VelocityPredictorTest {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-7;

    private static final double JACOBIAN_ERROR = 1e-6;

    @Test
    void testPredict() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var ax = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var ay = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var az = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var v = new double[]{vx, vy, vz};
        final var a = new double[]{ax, ay, az};

        // test with parameters and jacobians
        final var result1 = new double[3];
        final var jacobianV1 = new Matrix(3, 3);
        final var jacobianA1 = new Matrix(3, 3);
        VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, result1, jacobianV1, jacobianA1);

        // check correctness
        var result2 = new double[]{
                vx + ax * dt, vy + ay * dt, vz + az * dt
        };

        final var jacobianV2 = Matrix.identity(3, 3);
        final var jacobianA2 = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(dt);

        assertArrayEquals(result1, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV1.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA1.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt,
                new double[1], jacobianV1, jacobianA1));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt,
                result1, m, jacobianA1));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt,
                result1, jacobianV1, m));

        // test with parameters, without jacobians
        final var result3 = new double[3];
        VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, result3);

        // check correctness
        assertArrayEquals(result3, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt,
                new double[1]));

        // test with arrays, with jacobians
        final var result4 = new double[3];
        final var jacobianV3 = new Matrix(3, 3);
        final var jacobianA3 = new Matrix(3, 3);
        VelocityPredictor.predict(v, a, dt, result4, jacobianV3, jacobianA3);

        // check correctness
        assertArrayEquals(result4, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV3.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA3.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(new double[1], a, dt, result4,
                jacobianV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(v, new double[1], dt, result4,
                jacobianV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(v, a, dt, new double[1],
                jacobianV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(v, a, dt, result4,
                m, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(v, a, dt, result4, jacobianV3,
                m));

        // test with arrays, without jacobians
        final var result5 = new double[3];
        VelocityPredictor.predict(v, a, dt, result5);

        // check correctness
        assertArrayEquals(result5, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(new double[1], a, dt, result5));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(v, new double[1], dt, result5));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(v, a, dt, new double[1]));

        // test with new instance, with parameters and jacobians
        final var jacobianV4 = new Matrix(3, 3);
        final var jacobianA4 = new Matrix(3, 3);
        final var result6 = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, jacobianV4, jacobianA4);

        // check correctness
        assertArrayEquals(result6, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV4.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA4.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt,
                m, jacobianA4));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt,
                jacobianV4, m));

        // test with new instance, with parameters, without jacobians
        final var result7 = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt);

        // check correctness
        assertArrayEquals(result7, result2, ABSOLUTE_ERROR);

        // test with new instance, with arrays, with jacobians
        final var jacobianV5 = new Matrix(3, 3);
        final var jacobianA5 = new Matrix(3, 3);
        final var result8 = VelocityPredictor.predict(v, a, dt, jacobianV5, jacobianA5);

        // check correctness
        assertArrayEquals(result8, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV5.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA5.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(new double[1], a, dt, jacobianV5,
                jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(v, new double[1], dt, jacobianV5,
                jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(v, a, dt, m, jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(v, a, dt, jacobianV5, m));

        // test with new instance, with arrays, without jacobians
        final var result9 = VelocityPredictor.predict(v, a, dt);

        // check correctness
        assertArrayEquals(result9, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(new double[1], a, dt));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predict(v, new double[1], dt));

        // check correctness of jacobians

        // check velocity variation
        final var result10 = VelocityPredictor.predict(v, a, dt);

        var diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var v2 = ArrayUtils.sumAndReturnNew(v, diff);
        result2 = VelocityPredictor.predict(v2, a, dt);

        var diffResult = ArrayUtils.subtractAndReturnNew(result2, result10);
        var diffResult2 = jacobianV5.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check acceleration variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var a2 = ArrayUtils.sumAndReturnNew(a, diff);
        result2 = VelocityPredictor.predict(v, a2, dt);

        diffResult = ArrayUtils.subtractAndReturnNew(result2, result10);
        diffResult2 = jacobianA5.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    void testPredictWithVelocityAdjustment() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var dvx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var dvy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var dvz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var ax = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var ay = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var az = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var v = new double[]{vx, vy, vz};
        final var dv = new double[]{dvx, dvy, dvz};
        final var a = new double[]{ax, ay, az};

        // test with parameters and jacobians
        final var result1 = new double[3];
        final var jacobianV1 = new Matrix(3, 3);
        final var jacobianDV1 = new Matrix(3, 3);
        final var jacobianA1 = new Matrix(3, 3);
        VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz, dvx, dvy, dvz, ax, ay, az, dt, result1,
                jacobianV1, jacobianDV1, jacobianA1);

        // check correctness
        final var result2 = new double[]{
                vx + dvx + ax * dt, vy + dvy + ay * dt, vz + dvz + az * dt
        };

        final var jacobianV2 = Matrix.identity(3, 3);
        final var jacobianDV2 = Matrix.identity(3, 3);
        final var jacobianA2 = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(dt);

        assertArrayEquals(result1, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV1.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianDV1.equals(jacobianDV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA1.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz,
                dvx, dvy, dvz, ax, ay, az, dt, new double[1], jacobianV1, jacobianDV1, jacobianA1));
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz,
                dvx, dvy, dvz, ax, ay, az, dt, result1, m, jacobianDV1, jacobianA1));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz,
                dvx, dvy, dvz, ax, ay, az, dt, result1, jacobianV1, m, jacobianA1));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz,
                dvx, dvy, dvz, ax, ay, az, dt, result1, jacobianV1, jacobianDV1, m));

        // test with parameters, without jacobians
        final var result3 = new double[3];
        VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz, dvx, dvy, dvz, ax, ay, az, dt, result3);

        // check correctness
        assertArrayEquals(result3, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz,
                dvx, dvy, dvz, ax, ay, az, dt, new double[1]));

        // test with arrays, with jacobians
        final var result4 = new double[3];
        final var jacobianV3 = new Matrix(3, 3);
        final var jacobianDV3 = new Matrix(3, 3);
        final var jacobianA3 = new Matrix(3, 3);
        VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt, result4, jacobianV3, jacobianDV3, jacobianA3);

        // check correctness
        assertArrayEquals(result4, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV3.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianDV3.equals(jacobianDV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA3.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(
                new double[1], dv, a, dt, result4, jacobianV3, jacobianDV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(v,
                new double[1], a, dt, result4, jacobianV3, jacobianDV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(v, dv,
                new double[1], dt, result4, jacobianV3, jacobianDV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt,
                new double[1], jacobianV3, jacobianDV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt,
                result4, m, jacobianDV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt,
                result4, jacobianV3, m, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt,
                result4, jacobianV3, jacobianDV3, m));

        // test with arrays, without jacobians
        final var result5 = new double[3];
        VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt, result5);

        // check correctness
        assertArrayEquals(result5, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(
                new double[1], dv, a, dt, result5));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(v,
                new double[1], a, dt, result5));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(v, dv,
                new double[1], dt, result5));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt,
                new double[1]));

        // test with new instance, with parameters and jacobians
        final var jacobianV4 = new Matrix(3, 3);
        final var jacobianDV4 = new Matrix(3, 3);
        final var jacobianA4 = new Matrix(3, 3);
        final var result6 = VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz, dvx, dvy, dvz, ax, ay, az, dt,
                jacobianV4, jacobianDV4, jacobianA4);

        // check correctness
        assertArrayEquals(result6, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV4.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianDV4.equals(jacobianDV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA4.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(
                vx, vy, vz, dvx, dvy, dvz, ax, ay, az, dt, m, jacobianDV4, jacobianA4));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(
                vx, vy, vz, dvx, dvy, dvz, ax, ay, az, dt, jacobianV4, m, jacobianA4));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(
                vx, vy, vz, dvx, dvy, dvz, ax, ay, az, dt, jacobianV4, jacobianDV4, m));

        // test with new instance, with parameters, without jacobians
        final var result7 = VelocityPredictor.predictWithVelocityAdjustment(vx, vy, vz, dvx, dvy, dvz, ax, ay, az, dt);

        // check correctness
        assertArrayEquals(result7, result2, ABSOLUTE_ERROR);

        // test with new instance, with arrays, with jacobians
        final var jacobianV5 = new Matrix(3, 3);
        final var jacobianDV5 = new Matrix(3, 3);
        final var jacobianA5 = new Matrix(3, 3);
        final var result8 = VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt, jacobianV5, jacobianDV5,
                jacobianA5);

        // check correctness
        assertArrayEquals(result8, result2, ABSOLUTE_ERROR);
        assertTrue(jacobianV5.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianDV5.equals(jacobianDV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA5.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(
                new double[1], dv, a, dt, jacobianV5, jacobianDV5, jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(
                v, new double[1], a, dt, jacobianV5, jacobianDV5, jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(
                v, dv, new double[1], dt, jacobianV5, jacobianDV5, jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(
                v, dv, a, dt, m, jacobianDV5, jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(
                v, dv, a, dt, jacobianV5, m, jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(
                v, dv, a, dt, jacobianV5, jacobianDV5, m));

        // test with new instance, with arrays, without jacobians
        final var result9 = VelocityPredictor.predictWithVelocityAdjustment(v, dv, a, dt);

        // check correctness
        assertArrayEquals(result9, result2, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(
                new double[1], dv, a, dt));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(v,
                new double[1], a, dt));
        assertThrows(IllegalArgumentException.class, () -> VelocityPredictor.predictWithVelocityAdjustment(v, dv,
                new double[1], dt));
    }
}
