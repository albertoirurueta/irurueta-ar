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
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class PositionPredictorTest {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-7;

    private static final double JACOBIAN_ERROR = 1e-6;

    @Test
    void testPredict() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double axTemp;
        double ayTemp;
        double azTemp;
        do {
            axTemp = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            ayTemp = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            azTemp = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        } while (axTemp == 0.0 && ayTemp == 0.0 && azTemp == 0.0);
        final var ax = axTemp;
        final var ay = ayTemp;
        final var az = azTemp;
        final var dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var r = new InhomogeneousPoint3D(x, y, z);
        final var v = new double[]{vx, vy, vz};
        final var a = new double[]{ax, ay, az};

        // test with all parameters and jacobians
        final var result1 = new InhomogeneousPoint3D();
        final var jacobianR1 = new Matrix(3, 3);
        final var jacobianV1 = new Matrix(3, 3);
        final var jacobianA1 = new Matrix(3, 3);
        PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt, result1, jacobianR1, jacobianV1, jacobianA1);

        // check correctness
        var result2 = new InhomogeneousPoint3D(
                x + vx * dt + 0.5 * ax * dt * dt,
                y + vy * dt + 0.5 * ay * dt * dt,
                z + vz * dt + 0.5 * az * dt * dt);
        assertTrue(result1.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        final var jacobianR2 = Matrix.identity(3, 3);
        final var jacobianV2 = jacobianR2.multiplyByScalarAndReturnNew(dt);
        final var jacobianA2 = jacobianR2.multiplyByScalarAndReturnNew(0.5 * dt * dt);

        assertTrue(jacobianR1.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV1.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA1.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt,
                result1, m, jacobianV1, jacobianA1));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt,
                result1, jacobianR1, m, jacobianA1));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt,
                result1, jacobianR1, jacobianV1, m));

        // test with all parameters without jacobians
        final var result3 = new InhomogeneousPoint3D();
        PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt, result3);

        // check correctness
        assertTrue(result3.equals(result2, ABSOLUTE_ERROR));

        // test with arrays and jacobians
        final var result4 = new InhomogeneousPoint3D();
        final var jacobianR3 = new Matrix(3, 3);
        final var jacobianV3 = new Matrix(3, 3);
        final var jacobianA3 = new Matrix(3, 3);
        PositionPredictor.predict(r, v, a, dt, result4, jacobianR3, jacobianV3, jacobianA3);

        // check correctness
        assertTrue(result4.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianR3.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV3.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA3.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, new double[1], a, dt, result4,
                jacobianR3, jacobianV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, v, new double[1], dt, result4,
                jacobianR3, jacobianV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, v, a, dt, result4, m,
                jacobianV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, v, a, dt, result4, jacobianR3,
                m, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, v, a, dt, result4, jacobianR3,
                jacobianV3, m));

        // test with arrays and no jacobians
        final var result5 = new InhomogeneousPoint3D();
        PositionPredictor.predict(r, v, a, dt, result5);

        // check correctness
        assertTrue(result5.equals(result2, ABSOLUTE_ERROR));

        // test with no acceleration parameters and jacobians
        final var result6 = new InhomogeneousPoint3D();
        final var jacobianR4 = new Matrix(3, 3);
        final var jacobianV4 = new Matrix(3, 3);
        final var jacobianA4 = new Matrix(3, 3);
        PositionPredictor.predict(r, vx, vy, vz, dt, result6, jacobianR4, jacobianV4, jacobianA4);

        // check correctness
        final var result7 = new InhomogeneousPoint3D(x + vx * dt, y + vy * dt, z + vz * dt);
        assertTrue(result6.equals(result7, ABSOLUTE_ERROR));

        // check jacobians
        final var jacobianR5 = Matrix.identity(3, 3);
        final var jacobianV5 = jacobianR2.multiplyByScalarAndReturnNew(dt);
        final var jacobianA5 = new Matrix(3, 3);

        assertTrue(jacobianR4.equals(jacobianR5, ABSOLUTE_ERROR));
        assertTrue(jacobianV4.equals(jacobianV5, ABSOLUTE_ERROR));
        assertTrue(jacobianA4.equals(jacobianA5, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, vx, vy, vz, dt, result6,
                m, jacobianV4, jacobianA4));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, vx, vy, vz, dt, result6,
                jacobianR4, m, jacobianA4));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, vx, vy, vz, dt, result6,
                jacobianR4, jacobianV4, m));

        // test with no acceleration parameters and no jacobians
        final var result8 = new InhomogeneousPoint3D();
        PositionPredictor.predict(r, vx, vy, vz, dt, result8);

        // check correctness
        assertTrue(result8.equals(result7, ABSOLUTE_ERROR));

        // test with no acceleration, v array, and jacobians
        final var result9 = new InhomogeneousPoint3D();
        final var jacobianR6 = new Matrix(3, 3);
        final var jacobianV6 = new Matrix(3, 3);
        final var jacobianA6 = new Matrix(3, 3);
        PositionPredictor.predict(r, v, dt, result9, jacobianR6, jacobianV6, jacobianA6);

        // check correctness
        assertTrue(result9.equals(result7, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianR6.equals(jacobianR4, ABSOLUTE_ERROR));
        assertTrue(jacobianV6.equals(jacobianV4, ABSOLUTE_ERROR));
        assertTrue(jacobianA6.equals(jacobianA4, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, new double[1], dt, result9,
                jacobianR6, jacobianV6, jacobianA6));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, v, dt, result9, m, jacobianV6,
                jacobianA6));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, v, dt, result9, jacobianR6,
                m, jacobianA6));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, v, dt, result9, jacobianR6,
                jacobianV6, m));

        // test with no acceleration, v array, and no jacobians
        final var result10 = new InhomogeneousPoint3D();
        PositionPredictor.predict(r, v, dt, result10);

        // check correctness
        assertTrue(result10.equals(result7, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, new double[1], dt, result10));

        // test with new instance, with all parameters and jacobians
        final var jacobianR7 = new Matrix(3, 3);
        final var jacobianV7 = new Matrix(3, 3);
        final var jacobianA7 = new Matrix(3, 3);
        final var result11 = PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt, jacobianR7, jacobianV7,
                jacobianA7);

        // check correctness
        assertTrue(result11.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianR7.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV7.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA7.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, vx, vy, vz,
                ax, ay, az, dt, m, jacobianV7, jacobianA7));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, vx, vy, vz,
                ax, ay, az, dt, jacobianR7, m, jacobianA7));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, vx, vy, vz,
                ax, ay, az, dt, jacobianR7, jacobianV7, m));

        // test with new instance, with all parameters, without jacobians
        final var result12 = PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt);

        // check correctness
        assertTrue(result12.equals(result2, ABSOLUTE_ERROR));

        // test with new instance, with arrays and jacobians
        final var jacobianR8 = new Matrix(3, 3);
        final var jacobianV8 = new Matrix(3, 3);
        final var jacobianA8 = new Matrix(3, 3);
        final var result13 = PositionPredictor.predict(r, v, a, dt, jacobianR8, jacobianV8, jacobianA8);

        // check correctness
        assertTrue(result13.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianR8.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV8.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA8.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> PositionPredictor.predict(r, new double[1], a, dt, jacobianR8, jacobianV8, jacobianA8));
        assertThrows(IllegalArgumentException.class,
                () -> PositionPredictor.predict(r, v, new double[1], dt, jacobianR8, jacobianV8, jacobianA8));
        assertThrows(IllegalArgumentException.class,
                () -> PositionPredictor.predict(r, v, a, dt, m, jacobianV8, jacobianA8));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, v, a, dt, jacobianR8,
                m, jacobianA8));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, v, a, dt, jacobianR8,
                jacobianV8, m));

        // test with new instance, with arrays and no jacobians
        final var result14 = PositionPredictor.predict(r, v, a, dt);

        // check correctness
        assertTrue(result14.equals(result2, ABSOLUTE_ERROR));

        // test with new instance, with no acceleration parameters and jacobians
        final var jacobianR9 = new Matrix(3, 3);
        final var jacobianV9 = new Matrix(3, 3);
        final var jacobianA9 = new Matrix(3, 3);
        final var result15 = PositionPredictor.predict(r, vx, vy, vz, dt, jacobianR9, jacobianV9, jacobianA9);

        // check correctness
        assertTrue(result15.equals(result7, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianR9.equals(jacobianR4, ABSOLUTE_ERROR));
        assertTrue(jacobianV9.equals(jacobianV4, ABSOLUTE_ERROR));
        assertTrue(jacobianA9.equals(jacobianA4, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, vx, vy, vz, dt, m, jacobianV9,
                jacobianA9));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, vx, vy, vz, dt, jacobianR9,
                m, jacobianA9));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, vx, vy, vz, dt, jacobianR9,
                jacobianV9, m));

        // test with new instance, with no acceleration parameters and no
        // jacobians
        final var result16 = PositionPredictor.predict(r, vx, vy, vz, dt);

        // check correctness
        assertTrue(result16.equals(result7, ABSOLUTE_ERROR));

        // test with new instance, with no acceleration, v array, and jacobians
        final var jacobianR10 = new Matrix(3, 3);
        final var jacobianV10 = new Matrix(3, 3);
        final var jacobianA10 = new Matrix(3, 3);
        final var result17 = PositionPredictor.predict(r, v, dt, jacobianR10, jacobianV10, jacobianA10);

        // check correctness
        assertTrue(result17.equals(result7, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianR10.equals(jacobianR4, ABSOLUTE_ERROR));
        assertTrue(jacobianV10.equals(jacobianV4, ABSOLUTE_ERROR));
        assertTrue(jacobianA10.equals(jacobianA4, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, new double[1], dt, jacobianR10,
                jacobianV10, jacobianA10));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, v, dt, m, jacobianV10,
                jacobianA10));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, v, dt, jacobianR10, m,
                jacobianA10));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, v, dt, jacobianR10, jacobianV10,
                m));

        // test with new instance, with no acceleration, v array, and no
        // jacobians
        final var result18 = PositionPredictor.predict(r, v, dt);

        // check correctness
        assertTrue(result18.equals(result7, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predict(r, new double[1], dt));

        // check correctness of jacobians
        final var jacobianR11 = new Matrix(3, 3);
        final var jacobianV11 = new Matrix(3, 3);
        final var jacobianA11 = new Matrix(3, 3);
        final var result19 = PositionPredictor.predict(r, v, a, dt, jacobianR11, jacobianV11, jacobianA11);

        // check position variation
        var diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var r2 = new InhomogeneousPoint3D(
                r.getInhomX() + diff[0], r.getInhomY() + diff[1], r.getInhomZ() + diff[2]);
        result2 = PositionPredictor.predict(r2, v, a, dt);

        var diffResult = new double[]{
                result2.getInhomX() - result19.getInhomX(),
                result2.getInhomY() - result19.getInhomY(),
                result2.getInhomZ() - result19.getInhomZ()
        };
        var diffResult2 = jacobianR11.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check velocity variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var v2 = ArrayUtils.sumAndReturnNew(v, diff);
        result2 = PositionPredictor.predict(r, v2, a, dt);

        diffResult = new double[]{
                result2.getInhomX() - result19.getInhomX(),
                result2.getInhomY() - result19.getInhomY(),
                result2.getInhomZ() - result19.getInhomZ()
        };
        diffResult2 = jacobianV11.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check acceleration variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var a2 = ArrayUtils.sumAndReturnNew(a, diff);
        result2 = PositionPredictor.predict(r, v, a2, dt);

        diffResult = new double[]{
                result2.getInhomX() - result19.getInhomX(),
                result2.getInhomY() - result19.getInhomY(),
                result2.getInhomZ() - result19.getInhomZ()
        };
        diffResult2 = jacobianA11.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    void testPredictWithPositionAdjustment() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var rx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var ry = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var rz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var drx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var dry = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var drz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var vx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var vz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var ax = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var ay = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var az = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var r = new InhomogeneousPoint3D(rx, ry, rz);
        final var dr = new double[]{drx, dry, drz};
        final var v = new double[]{vx, vy, vz};
        final var a = new double[]{ax, ay, az};

        // test with all parameters and jacobians
        final var result1 = new InhomogeneousPoint3D();
        final var jacobianR1 = new Matrix(3, 3);
        final var jacobianDR1 = new Matrix(3, 3);
        final var jacobianV1 = new Matrix(3, 3);
        final var jacobianA1 = new Matrix(3, 3);
        PositionPredictor.predictWithPositionAdjustment(r, drx, dry, drz, vx, vy, vz, ax, ay, az, dt, result1,
                jacobianR1, jacobianDR1, jacobianV1, jacobianA1);

        // check correctness
        var result2 = new InhomogeneousPoint3D(
                rx + drx + vx * dt + 0.5 * ax * dt * dt,
                ry + dry + vy * dt + 0.5 * ay * dt * dt,
                rz + drz + vz * dt + 0.5 * az * dt * dt);
        assertTrue(result1.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        final var jacobianR2 = Matrix.identity(3, 3);
        final var jacobianDR2 = Matrix.identity(3, 3);
        final var jacobianV2 = jacobianR2.multiplyByScalarAndReturnNew(dt);
        final var jacobianA2 = jacobianR2.multiplyByScalarAndReturnNew(0.5 * dt * dt);

        assertTrue(jacobianR1.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianDR1.equals(jacobianDR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV1.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA1.equals(jacobianA2, ABSOLUTE_ERROR));

        // force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, result1, m, jacobianDR1, jacobianV1, jacobianA1));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, result1, jacobianR1, m, jacobianV1, jacobianA1));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, result1, jacobianR1, jacobianDR1, m, jacobianA1));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, result1, jacobianR1, jacobianDR1, jacobianV1, m));

        // test with all parameters without jacobians
        final var result3 = new InhomogeneousPoint3D();
        PositionPredictor.predictWithPositionAdjustment(r, drx, dry, drz, vx, vy, vz, ax, ay, az, dt, result3);

        // check correctness
        assertTrue(result3.equals(result2, ABSOLUTE_ERROR));

        // test with arrays and jacobians
        final var result4 = new InhomogeneousPoint3D();
        final var jacobianR3 = new Matrix(3, 3);
        final var jacobianDR3 = new Matrix(3, 3);
        final var jacobianV3 = new Matrix(3, 3);
        final var jacobianA3 = new Matrix(3, 3);
        PositionPredictor.predictWithPositionAdjustment(r, dr, v, a, dt, result4, jacobianR3, jacobianDR3, jacobianV3,
                jacobianA3);

        // check correctness
        assertTrue(result4.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianR3.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianDR3.equals(jacobianDR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV3.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA3.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                new double[1], v, a, dt, result4, jacobianR3, jacobianDR3, jacobianV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r, dr,
                new double[1], a, dt, result4, jacobianR3, jacobianDR3, jacobianV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r, dr, v,
                new double[1], dt, result4, jacobianR3, jacobianDR3, jacobianV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r, dr, v, a,
                dt, result4, m, jacobianDR3, jacobianV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r, dr, v, a,
                dt, result4, jacobianR3, m, jacobianV3, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r, dr, v, a,
                dt, result4, jacobianR3, jacobianDR3, m, jacobianA3));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r, dr, v, a,
                dt, result4, jacobianR3, jacobianDR3, jacobianV3, m));

        // test with arrays and no jacobians
        final var result5 = new InhomogeneousPoint3D();
        PositionPredictor.predictWithPositionAdjustment(r, dr, v, a, dt, result5);

        // check correctness
        assertTrue(result5.equals(result2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                new double[1], v, a, dt, result5));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r, dr,
                new double[1], a, dt, result5));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r, dr, v,
                new double[1], dt, result5));

        // test with new instance, with all parameters and jacobians
        final var jacobianR4 = new Matrix(3, 3);
        final var jacobianDR4 = new Matrix(3, 3);
        final var jacobianV4 = new Matrix(3, 3);
        final var jacobianA4 = new Matrix(3, 3);
        final var result6 = PositionPredictor.predictWithPositionAdjustment(r, drx, dry, drz, vx, vy, vz, ax, ay, az,
                dt, jacobianR4, jacobianDR4, jacobianV4, jacobianA4);

        // check correctness
        assertTrue(result6.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianR4.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianDR4.equals(jacobianDR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV4.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA4.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, m, jacobianDR4, jacobianV4, jacobianA4));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, jacobianR4, m, jacobianV4, jacobianA4));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, jacobianR4, jacobianDR4, m, jacobianA4));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                drx, dry, drz, vx, vy, vz, ax, ay, az, dt, jacobianR4, jacobianDR4, jacobianV4, m));

        // test with new instance, with all parameters and no jacobians
        final var result7 = PositionPredictor.predictWithPositionAdjustment(r, drx, dry, drz, vx, vy, vz, ax, ay, az,
                dt);

        // check correctness
        assertTrue(result7.equals(result2, ABSOLUTE_ERROR));


        // test with new instance, with arrays and jacobians
        final var jacobianR5 = new Matrix(3, 3);
        final var jacobianDR5 = new Matrix(3, 3);
        final var jacobianV5 = new Matrix(3, 3);
        final var jacobianA5 = new Matrix(3, 3);
        final var result8 = PositionPredictor.predictWithPositionAdjustment(r, dr, v, a, dt, jacobianR5, jacobianDR5,
                jacobianV5, jacobianA5);

        // check correctness
        assertTrue(result8.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianR5.equals(jacobianR2, ABSOLUTE_ERROR));
        assertTrue(jacobianDR5.equals(jacobianDR2, ABSOLUTE_ERROR));
        assertTrue(jacobianV5.equals(jacobianV2, ABSOLUTE_ERROR));
        assertTrue(jacobianA5.equals(jacobianA2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                new double[1], v, a, dt, jacobianR5, jacobianDR5, jacobianV5, jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                dr, new double[1], a, dt, jacobianR5, jacobianDR5, jacobianV5, jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                dr, v, new double[1], dt, jacobianR5, jacobianDR5, jacobianV5, jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                dr, v, a, dt, m, jacobianDR5, jacobianV5, jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                dr, v, a, dt, jacobianR5, m, jacobianV5, jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                dr, v, a, dt, jacobianR5, jacobianDR5, m, jacobianA5));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                dr, v, a, dt, jacobianR5, jacobianDR5, jacobianV5, m));

        // test with new instance, with arrays and no jacobians
        final var result9 = PositionPredictor.predictWithPositionAdjustment(r, dr, v, a, dt);

        // check correctness
        assertTrue(result9.equals(result2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r,
                new double[1], v, a, dt));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r, dr,
                new double[1], a, dt));
        assertThrows(IllegalArgumentException.class, () -> PositionPredictor.predictWithPositionAdjustment(r, dr, v,
                new double[1], dt));

        // check correctness of jacobians
        final var jacobianR6 = new Matrix(3, 3);
        final var jacobianDR6 = new Matrix(3, 3);
        final var jacobianV6 = new Matrix(3, 3);
        final var jacobianA6 = new Matrix(3, 3);
        final var result10 = PositionPredictor.predictWithPositionAdjustment(r, dr, v, a, dt, jacobianR6, jacobianDR6,
                jacobianV6, jacobianA6);

        // check position variation
        var diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var r2 = new InhomogeneousPoint3D(
                r.getInhomX() + diff[0], r.getInhomY() + diff[1], r.getInhomZ() + diff[2]);
        result2 = PositionPredictor.predictWithPositionAdjustment(r2, dr, v, a, dt);

        var diffResult = new double[]{
                result2.getInhomX() - result10.getInhomX(),
                result2.getInhomY() - result10.getInhomY(),
                result2.getInhomZ() - result10.getInhomZ()
        };
        var diffResult2 = jacobianR6.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check position adjustment variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var dr2 = ArrayUtils.sumAndReturnNew(dr, diff);
        result2 = PositionPredictor.predictWithPositionAdjustment(r, dr2, v, a, dt);

        diffResult = new double[]{
                result2.getInhomX() - result10.getInhomX(),
                result2.getInhomY() - result10.getInhomY(),
                result2.getInhomZ() - result10.getInhomZ()
        };
        diffResult2 = jacobianDR6.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check velocity variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var v2 = ArrayUtils.sumAndReturnNew(v, diff);
        result2 = PositionPredictor.predictWithPositionAdjustment(r, dr, v2, a, dt);

        diffResult = new double[]{
                result2.getInhomX() - result10.getInhomX(),
                result2.getInhomY() - result10.getInhomY(),
                result2.getInhomZ() - result10.getInhomZ()
        };
        diffResult2 = jacobianV6.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check acceleration variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var a2 = ArrayUtils.sumAndReturnNew(a, diff);
        result2 = PositionPredictor.predictWithPositionAdjustment(r, dr, v, a2, dt);

        diffResult = new double[]{
                result2.getInhomX() - result10.getInhomX(),
                result2.getInhomY() - result10.getInhomY(),
                result2.getInhomZ() - result10.getInhomZ()
        };
        diffResult2 = jacobianA6.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }
}
