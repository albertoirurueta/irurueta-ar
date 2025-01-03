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
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class QuaternionPredictorTest {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-7;

    private static final double JACOBIAN_ERROR = 1e-6;

    @Test
    void testPredict() throws WrongSizeException, RotationException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var q = new Quaternion(roll, pitch, yaw);

        final var wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var w = new double[]{wx, wy, wz};

        final var dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // test with parameters and jacobians

        // test exact method
        final var result1 = new Quaternion();
        final var jacobianQ1 = new Matrix(4, 4);
        final var jacobianW1 = new Matrix(4, 3);
        QuaternionPredictor.predict(q, wx, wy, wz, dt, true, result1, jacobianQ1, jacobianW1);

        // check correctness
        final var dtw = new double[]{dt * wx, dt * wy, dt * wz};
        final var tmp = new Quaternion();
        var jacobianW2 = new Matrix(4, 3);
        Quaternion.rotationVectorToQuaternion(dtw, tmp, jacobianW2);
        jacobianW2.multiplyByScalar(dt);
        final var result2 = new Quaternion();
        final var jacobianQ2 = new Matrix(4, 4);
        final var jacobianQ2W = new Matrix(4, 4);
        Quaternion.product(q, tmp, result2, jacobianQ2, jacobianQ2W);

        jacobianW2 = jacobianQ2W.multiplyAndReturnNew(jacobianW2);

        assertTrue(result1.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ1.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW1.equals(jacobianW2, ABSOLUTE_ERROR));

        // test Tustin method
        final var result3 = new Quaternion();
        final var jacobianQ3 = new Matrix(4, 4);
        final var jacobianW3 = new Matrix(4, 3);
        QuaternionPredictor.predict(q, wx, wy, wz, dt, false, result3, jacobianQ3, jacobianW3);

        Matrix wMatrix = RotationUtils.w2omega(w);
        Matrix qMatrix = Matrix.newFromArray(q.getValues());
        wMatrix.multiply(qMatrix);
        wMatrix.multiplyByScalar(0.5 * dt);

        final var result4 = new Quaternion(q.getA() + wMatrix.getElementAtIndex(0),
                q.getB() + wMatrix.getElementAtIndex(1),
                q.getC() + wMatrix.getElementAtIndex(2),
                q.getD() + wMatrix.getElementAtIndex(3));

        assertTrue(result3.equals(result4, ABSOLUTE_ERROR));

        // check jacobians
        wMatrix = RotationUtils.w2omega(w);
        final var jacobianQ4 = Matrix.identity(4, 4).addAndReturnNew(wMatrix)
                .multiplyByScalarAndReturnNew(0.5 * dt);

        final var jacobianW4 = RotationUtils.quaternionToConjugatedPiMatrix(q).multiplyByScalarAndReturnNew(0.5 * dt);

        assertTrue(jacobianQ3.equals(jacobianQ4, ABSOLUTE_ERROR));
        assertTrue(jacobianW3.equals(jacobianW4, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, dt,
                false, result3, m, jacobianW3));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, dt,
                false, result3, jacobianQ3, m));

        // test with w array and jacobians

        // test exact method
        QuaternionPredictor.predict(q, w, dt, true, result3, jacobianQ3, jacobianW3);

        // check correctness
        assertTrue(result3.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ3.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW3.equals(jacobianW2, ABSOLUTE_ERROR));

        // test Tustin method
        QuaternionPredictor.predict(q, w, dt, false, result3, jacobianQ3, jacobianW3);

        // check correctness
        assertTrue(result3.equals(result3, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ3.equals(jacobianQ4, ABSOLUTE_ERROR));
        assertTrue(jacobianW3.equals(jacobianW4, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> QuaternionPredictor.predict(q, new double[1], dt, true, result3, jacobianQ3,
                        jacobianW3));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, dt, true,
                result3, m, jacobianW3));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, dt, true,
                result3, jacobianQ3, m));

        // test with parameters, exact method and jacobians
        final var result5 = new Quaternion();
        final var jacobianQ5 = new Matrix(4, 4);
        final var jacobianW5 = new Matrix(4, 3);
        QuaternionPredictor.predict(q, wx, wy, wz, dt, result5, jacobianQ5, jacobianW5);

        // check correctness
        assertTrue(result5.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ5.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW5.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, dt, result5,
                m, jacobianW5));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, dt, result5,
                jacobianQ5, m));

        // test with array, Tustin method and jacobians
        final var result6 = new Quaternion();
        final var jacobianQ6 = new Matrix(4, 4);
        final var jacobianW6 = new Matrix(4, 3);
        QuaternionPredictor.predict(q, w, dt, result6, jacobianQ6, jacobianW6);

        // check correctness
        assertTrue(result6.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ6.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW6.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1], dt, result6,
                jacobianQ6, jacobianW6));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, dt, result6, m,
                jacobianW6));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, dt, result6, jacobianQ6,
                m));

        // test with parameters, without jacobians

        // test exact method
        final var result7 = new Quaternion();
        QuaternionPredictor.predict(q, wx, wy, wz, dt, true, result7);

        // check correctness
        assertTrue(result7.equals(result2, ABSOLUTE_ERROR));

        // test Tustin method
        final var result8 = new Quaternion();
        QuaternionPredictor.predict(q, wx, wy, wz, dt, false, result8);

        // check correctness
        assertTrue(result8.equals(result3, ABSOLUTE_ERROR));

        // test with array, without jacobians

        // test exact method
        final var result9 = new Quaternion();
        QuaternionPredictor.predict(q, w, dt, true, result9);

        // check correctness
        assertTrue(result9.equals(result2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1], dt,
                true, result9));

        // test Tustin method
        final var result10 = new Quaternion();
        QuaternionPredictor.predict(q, w, dt, false, result10);

        // check correctness
        assertTrue(result10.equals(result3, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1], dt,
                false, result10));

        // test with parameters, exact method, without jacobians
        final var result11 = new Quaternion();
        QuaternionPredictor.predict(q, wx, wy, wz, dt, result11);

        // check correctness
        assertTrue(result11.equals(result2, ABSOLUTE_ERROR));

        // test with array, exact method, without jacobians
        final var result12 = new Quaternion();
        QuaternionPredictor.predict(q, w, dt, result12);

        // check correctness
        assertTrue(result12.equals(result2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1], dt, result12));

        // test with new instance, parameters and jacobians

        // test exact method
        final var jacobianQ7 = new Matrix(4, 4);
        final var jacobianW7 = new Matrix(4, 3);
        final var result13 = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, jacobianQ7, jacobianW7);

        // check correctness
        assertTrue(result13.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ7.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW7.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, dt,
                true, m, jacobianW7));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, dt,
                true, jacobianQ7, m));

        // test Tustin method
        final var jacobianQ8 = new Matrix(4, 4);
        final var jacobianW8 = new Matrix(4, 3);
        final var result14 = QuaternionPredictor.predict(q, wx, wy, wz, dt, false, jacobianQ8, jacobianW8);

        // check correctness
        assertTrue(result14.equals(result3, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ8.equals(jacobianQ3, ABSOLUTE_ERROR));
        assertTrue(jacobianW8.equals(jacobianW3, ABSOLUTE_ERROR));

        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, dt,
                false, m, jacobianW8));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, dt,
                false, jacobianQ8, m));

        // test with new instance, with array and jacobians

        // test exact method
        final var jacobianQ9 = new Matrix(4, 4);
        final var jacobianW9 = new Matrix(4, 3);
        final var result15 = QuaternionPredictor.predict(q, w, dt, true, jacobianQ9, jacobianW9);

        // check correctness
        assertTrue(result15.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ9.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW9.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1], dt,
                true, jacobianQ9, jacobianW9));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, dt, true,
                m, jacobianW9));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, dt, true,
                jacobianQ9, m));

        // test Tustin method
        final var jacobianQ10 = new Matrix(4, 4);
        final var jacobianW10 = new Matrix(4, 3);
        final var result16 = QuaternionPredictor.predict(q, w, dt, false, jacobianQ10, jacobianW10);

        // check correctness
        assertTrue(result16.equals(result3, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ10.equals(jacobianQ3, ABSOLUTE_ERROR));
        assertTrue(jacobianW10.equals(jacobianW3, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1], dt,
                false, jacobianQ10, jacobianW10));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, dt, false,
                m, jacobianW10));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, dt, false,
                jacobianQ10, m));

        // test with new instance, parameters, exact method and jacobians
        final var jacobianQ11 = new Matrix(4, 4);
        final var jacobianW11 = new Matrix(4, 3);
        final var result17 = QuaternionPredictor.predict(q, wx, wy, wz, dt, jacobianQ11, jacobianW11);

        // check correctness
        assertTrue(result17.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ11.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW11.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, dt, m,
                jacobianW11));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, dt, jacobianQ11,
                m));

        // test with new instance, array, exact method and jacobians
        final var jacobianQ12 = new Matrix(4, 4);
        final var jacobianW12 = new Matrix(4, 3);
        final var result18 = QuaternionPredictor.predict(q, w, dt, jacobianQ12, jacobianW12);

        // check correctness
        assertTrue(result18.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ12.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW12.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1], dt,
                jacobianQ12, jacobianW12));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, dt, m, jacobianW12));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, dt, jacobianQ12, m));

        // test with new instance, with parameters and no jacobians

        // test exact method
        final var result19 = QuaternionPredictor.predict(q, wx, wy, wz, dt, true);

        // check correctness
        assertTrue(result19.equals(result2, ABSOLUTE_ERROR));

        // test Tustin method
        final var result20 = QuaternionPredictor.predict(q, wx, wy, wz, dt, false);

        // check correctness
        assertTrue(result20.equals(result3, ABSOLUTE_ERROR));

        // test with new instance, array and no jacobians

        // test exact method
        final var result21 = QuaternionPredictor.predict(q, w, dt, true);

        // check correctness
        assertTrue(result21.equals(result2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1], dt,
                true));

        // test Tustin method
        final var result22 = QuaternionPredictor.predict(q, w, dt, false);

        // check correctness
        assertTrue(result22.equals(result3, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1], dt,
                false));

        // test with new instance, with parameters, with exact method, without
        // jacobians
        final var result23 = QuaternionPredictor.predict(q, wx, wy, wz, dt);

        // check correctness
        assertTrue(result23.equals(result2, ABSOLUTE_ERROR));

        // test with new instance, array, exact method and no jacobians
        final var result24 = QuaternionPredictor.predict(q, w, dt);

        // check correctness
        assertTrue(result24.equals(result2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1], dt));

        // check correctness of jacobians (for exact method)

        // check quaternion variation
        final var jacobianQ13 = new Matrix(4, 4);
        final var jacobianW13 = new Matrix(4, 3);
        final var result25 = QuaternionPredictor.predict(q, w, dt, true, jacobianQ13, jacobianW13);

        var diff = new double[4];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var q2 = new Quaternion(q.getA() + diff[0], q.getB() + diff[1], q.getC() + diff[2],
                q.getD() + diff[3]);
        final var result26 = QuaternionPredictor.predict(q2, w, dt, true);

        var diffResult = new double[]{
                result26.getA() - result25.getA(),
                result26.getB() - result25.getB(),
                result26.getC() - result25.getC(),
                result26.getD() - result25.getD()
        };
        var diffResult2 = jacobianQ13.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check angular speed variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var w2 = ArrayUtils.sumAndReturnNew(w, diff);
        final var result27 = QuaternionPredictor.predict(q, w2, dt, true);

        diffResult = new double[]{
                result27.getA() - result25.getA(),
                result27.getB() - result25.getB(),
                result27.getC() - result25.getC(),
                result27.getD() - result25.getD()
        };
        diffResult2 = jacobianW13.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    void testPredictDefaultDt() throws WrongSizeException, RotationException {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var q = new Quaternion(roll, pitch, yaw);

        final var wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var w = new double[]{wx, wy, wz};

        final var dt = 1.0;

        // test with parameters and jacobians

        // test exact method
        final var result1 = new Quaternion();
        final var jacobianQ1 = new Matrix(4, 4);
        final var jacobianW1 = new Matrix(4, 3);
        QuaternionPredictor.predict(q, wx, wy, wz, true, result1, jacobianQ1, jacobianW1);

        // check correctness
        final var dtw = new double[]{dt * wx, dt * wy, dt * wz};
        final var tmp = new Quaternion();
        var jacobianW2 = new Matrix(4, 3);
        Quaternion.rotationVectorToQuaternion(dtw, tmp, jacobianW2);
        jacobianW2.multiplyByScalar(dt);
        var result2 = new Quaternion();
        final var jacobianQ2 = new Matrix(4, 4);
        final var jacobianQ2W = new Matrix(4, 4);
        Quaternion.product(q, tmp, result2, jacobianQ2, jacobianQ2W);

        jacobianW2 = jacobianQ2W.multiplyAndReturnNew(jacobianW2);

        assertTrue(result1.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ1.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW1.equals(jacobianW2, ABSOLUTE_ERROR));

        // test Tustin method
        final var result3 = new Quaternion();
        final var jacobianQ3 = new Matrix(4, 4);
        final var jacobianW3 = new Matrix(4, 3);
        QuaternionPredictor.predict(q, wx, wy, wz, false, result3, jacobianQ3, jacobianW3);

        Matrix wMatrix = RotationUtils.w2omega(w);
        Matrix qMatrix = Matrix.newFromArray(q.getValues());
        wMatrix.multiply(qMatrix);
        wMatrix.multiplyByScalar(0.5 * dt);

        final var result4 = new Quaternion(q.getA() + wMatrix.getElementAtIndex(0),
                q.getB() + wMatrix.getElementAtIndex(1),
                q.getC() + wMatrix.getElementAtIndex(2),
                q.getD() + wMatrix.getElementAtIndex(3));

        assertTrue(result3.equals(result4, ABSOLUTE_ERROR));

        // check jacobians
        wMatrix = RotationUtils.w2omega(w);
        final var jacobianQ4 = Matrix.identity(4, 4).addAndReturnNew(wMatrix)
                .multiplyByScalarAndReturnNew(0.5 * dt);

        final var jacobianW4 = RotationUtils.quaternionToConjugatedPiMatrix(q).multiplyByScalarAndReturnNew(0.5 * dt);

        assertTrue(jacobianQ3.equals(jacobianQ4, ABSOLUTE_ERROR));
        assertTrue(jacobianW3.equals(jacobianW4, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, false,
                result3, m, jacobianW3));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, false,
                result3, jacobianQ3, m));

        // test with w array and jacobians

        // test exact method
        QuaternionPredictor.predict(q, w, true, result4, jacobianQ4, jacobianW4);

        // check correctness
        assertTrue(result4.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ4.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW4.equals(jacobianW2, ABSOLUTE_ERROR));

        // test Tustin method
        QuaternionPredictor.predict(q, w, false, result4, jacobianQ4, jacobianW4);

        // check correctness
        assertTrue(result4.equals(result3, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ4.equals(jacobianQ3, ABSOLUTE_ERROR));
        assertTrue(jacobianW4.equals(jacobianW3, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1],
                true, result4, jacobianQ4, jacobianW4));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, true, result4,
                m, jacobianW4));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, true, result4,
                jacobianQ4, m));

        // test with parameters, exact method and jacobians
        final var result5 = new Quaternion();
        final var jacobianQ5 = new Matrix(4, 4);
        final var jacobianW5 = new Matrix(4, 3);
        QuaternionPredictor.predict(q, wx, wy, wz, result5, jacobianQ5, jacobianW5);

        // check correctness
        assertTrue(result5.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ5.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW5.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, result5,
                m, jacobianW5));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, result5,
                jacobianQ5, m));

        // test with array, exact method and jacobians
        final var result6 = new Quaternion();
        final var jacobianQ6 = new Matrix(4, 4);
        final var jacobianW6 = new Matrix(4, 3);
        QuaternionPredictor.predict(q, w, result6, jacobianQ6, jacobianW6);

        // check correctness
        assertTrue(result6.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ6.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW6.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1], result6,
                jacobianQ6, jacobianW6));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, result6, m, jacobianW6));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, result6, jacobianQ6, m));

        // test with parameters, without jacobians

        // test exact method
        final var result7 = new Quaternion();
        QuaternionPredictor.predict(q, wx, wy, wz, true, result7);

        // check correctness
        assertTrue(result7.equals(result2, ABSOLUTE_ERROR));

        // test Tustin method
        final var result8 = new Quaternion();
        QuaternionPredictor.predict(q, wx, wy, wz, false, result8);

        // check correctness
        assertTrue(result8.equals(result3, ABSOLUTE_ERROR));

        // test with array, without jacobians

        // test exact method
        final var result9 = new Quaternion();
        QuaternionPredictor.predict(q, w, true, result9);

        // check correctness
        assertTrue(result9.equals(result2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1],
                true, result9));

        // test Tustin method
        final var result10 = new Quaternion();
        QuaternionPredictor.predict(q, w, false, result10);

        // check correctness
        assertTrue(result10.equals(result3, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1],
                false, result10));

        // test with parameters, exact method, without jacobians
        final var result11 = new Quaternion();
        QuaternionPredictor.predict(q, wx, wy, wz, result11);

        // check correctness
        assertTrue(result11.equals(result2, ABSOLUTE_ERROR));

        // test with array, exact method, without jacobians
        final var result12 = new Quaternion();
        QuaternionPredictor.predict(q, w, result12);

        // check correctness
        assertTrue(result12.equals(result2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1], result12));

        // test with new instance, parameters and jacobians

        // test exact method
        final var jacobianQ7 = new Matrix(4, 4);
        final var jacobianW7 = new Matrix(4, 3);
        final var result13 = QuaternionPredictor.predict(q, wx, wy, wz, true, jacobianQ7, jacobianW7);

        // check correctness
        assertTrue(result13.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ7.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW7.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, true,
                m, jacobianW7));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, true,
                jacobianQ7, m));

        // test Tustin method
        final var jacobianQ8 = new Matrix(4, 4);
        final var jacobianW8 = new Matrix(4, 3);
        final var result14 = QuaternionPredictor.predict(q, wx, wy, wz, false, jacobianQ8, jacobianW8);

        // check correctness
        assertTrue(result14.equals(result3, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ8.equals(jacobianQ3, ABSOLUTE_ERROR));
        assertTrue(jacobianW8.equals(jacobianW3, ABSOLUTE_ERROR));

        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, false,
                m, jacobianW8));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, false,
                jacobianQ8, m));

        // test with new instance, with array and jacobians

        // test exact method
        final var jacobianQ9 = new Matrix(4, 4);
        final var jacobianW9 = new Matrix(4, 3);
        final var result15 = QuaternionPredictor.predict(q, w, true, jacobianQ9, jacobianW9);

        // check correctness
        assertTrue(result15.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ9.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW9.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1],
                true, jacobianQ9, jacobianW9));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, true,
                m, jacobianW9));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, true,
                jacobianQ9, m));

        // test Tustin method
        final var jacobianQ10 = new Matrix(4, 4);
        final var jacobianW10 = new Matrix(4, 3);
        final var result16 = QuaternionPredictor.predict(q, w, false, jacobianQ10, jacobianW10);

        // check correctness
        assertTrue(result16.equals(result3, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ10.equals(jacobianQ3, ABSOLUTE_ERROR));
        assertTrue(jacobianW10.equals(jacobianW3, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1],
                false, jacobianQ10, jacobianW10));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, false,
                m, jacobianW10));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, false,
                jacobianQ10, m));

        // test with new instance, parameters, exact method and jacobians
        final var jacobianQ11 = new Matrix(4, 4);
        final var jacobianW11 = new Matrix(4, 3);
        final var result17 = QuaternionPredictor.predict(q, wx, wy, wz, jacobianQ11, jacobianW11);

        // check correctness
        assertTrue(result17.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ11.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW11.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, m, jacobianW11));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, wx, wy, wz, jacobianQ11, m));

        // test with new instance, array, exact method and jacobians
        final var jacobianQ12 = new Matrix(4, 4);
        final var jacobianW12 = new Matrix(4, 3);
        final var result18 = QuaternionPredictor.predict(q, w, jacobianQ12, jacobianW12);

        // check correctness
        assertTrue(result18.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ12.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW12.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1], jacobianQ12,
                jacobianW12));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, m, jacobianW12));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, w, jacobianQ12, m));

        // test with new instance, with parameters and no jacobians

        // test exact method
        final var result19 = QuaternionPredictor.predict(q, wx, wy, wz, true);

        // check correctness
        assertTrue(result19.equals(result2, ABSOLUTE_ERROR));

        // test Tustin method
        final var result20 = QuaternionPredictor.predict(q, wx, wy, wz, false);

        // check correctness
        assertTrue(result20.equals(result3, ABSOLUTE_ERROR));

        // test with new instance, array and no jacobians

        // test exact method
        final var result21 = QuaternionPredictor.predict(q, w, true);

        // check correctness
        assertTrue(result21.equals(result2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1],
                true));

        // test Tustin method
        final var result22 = QuaternionPredictor.predict(q, w, false);

        // check correctness
        assertTrue(result22.equals(result3, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1],
                false));

        // test with new instance, with parameters, with exact method, without
        // jacobians
        final var result23 = QuaternionPredictor.predict(q, wx, wy, wz);

        // check correctness
        assertTrue(result23.equals(result2, ABSOLUTE_ERROR));

        // test with new instance, array, exact method and no jacobians
        final var result24 = QuaternionPredictor.predict(q, w);

        // check correctness
        assertTrue(result24.equals(result2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predict(q, new double[1]));

        // check correctness of jacobians (for exact method)

        // check quaternion variation
        final var jacobianQ13 = new Matrix(4, 4);
        final var jacobianW13 = new Matrix(4, 3);
        final var result25 = QuaternionPredictor.predict(q, w, true, jacobianQ13, jacobianW13);

        var diff = new double[4];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var q2 = new Quaternion(q.getA() + diff[0], q.getB() + diff[1],
                q.getC() + diff[2], q.getD() + diff[3]);
        result2 = QuaternionPredictor.predict(q2, w, true);

        var diffResult = new double[]{
                result2.getA() - result25.getA(),
                result2.getB() - result25.getB(),
                result2.getC() - result25.getC(),
                result2.getD() - result25.getD()
        };
        var diffResult2 = jacobianQ13.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check angular speed variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var w2 = ArrayUtils.sumAndReturnNew(w, diff);
        result2 = QuaternionPredictor.predict(q, w2, true);

        diffResult = new double[]{
                result2.getA() - result25.getA(),
                result2.getB() - result25.getB(),
                result2.getC() - result25.getC(),
                result2.getD() - result25.getD()
        };
        diffResult2 = jacobianW13.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }

    @Test
    void testPredictWithRotationAdjustment() throws WrongSizeException, RotationException {
        final var randomizer = new UniformRandomizer();

        final var roll1 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var roll2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw2 = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var q = new Quaternion(roll1, pitch1, yaw1);
        final var dq = new Quaternion(roll2, pitch2, yaw2);

        final var wx = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wy = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var wz = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var w = new double[]{wx, wy, wz};

        final var dt = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // test with parameters and jacobians

        // test exact method
        final var result1 = new Quaternion();
        final var jacobianQ1 = new Matrix(4, 4);
        final var jacobianDQ1 = new Matrix(4, 4);
        final var jacobianW1 = new Matrix(4, 3);
        QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx, wy, wz, dt, result1, jacobianQ1, jacobianDQ1,
                jacobianW1);

        // check correctness
        final var dtw = new double[]{dt * wx, dt * wy, dt * wz};
        final var tmp = new Quaternion();
        var jacobianW2 = new Matrix(4, 3);
        Quaternion.rotationVectorToQuaternion(dtw, tmp, jacobianW2);
        final var jacobianQb = new Matrix(4, 4);
        var jacobianDQ2 = new Matrix(4, 4);
        final var tmp2 = new Quaternion();
        Quaternion.product(dq, tmp, tmp2, jacobianDQ2, jacobianQb);

        final var jacobianQ2 = new Matrix(4, 4);
        final var jacobianQ2b = new Matrix(4, 4);
        final var result2 = new Quaternion();
        Quaternion.product(q, tmp2, result2, jacobianQ2, jacobianQ2b);

        jacobianDQ2 = jacobianQ2b.multiplyAndReturnNew(jacobianDQ2);
        jacobianW2 = jacobianQ2b.multiplyAndReturnNew(jacobianQb).multiplyAndReturnNew(jacobianW2);
        jacobianW2.multiplyByScalar(dt);

        assertTrue(result1.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ1.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianDQ1.equals(jacobianDQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW1.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx,
                wy, wz, dt, result1, m, jacobianDQ1, jacobianW1));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx,
                wy, wz, dt, result1, jacobianQ1, m, jacobianW1));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx,
                wy, wz, dt, result1, jacobianQ1, jacobianDQ1, m));

        // test with w array and jacobians
        final var result3 = new Quaternion();
        final var jacobianQ3 = new Matrix(4, 4);
        final var jacobianDQ3 = new Matrix(4, 4);
        final var jacobianW3 = new Matrix(4, 3);
        QuaternionPredictor.predictWithRotationAdjustment(q, dq, w, dt, result3, jacobianQ3, jacobianDQ3, jacobianW3);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q, dq,
                new double[1], dt, result3, jacobianQ3, jacobianDQ3, jacobianW3));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q, dq, w,
                dt, result3, m, jacobianDQ3, jacobianW3));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q, dq, w,
                dt, result3, jacobianQ3, m, jacobianW3));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q, dq, w,
                dt, result3, jacobianQ3, jacobianDQ3, m));

        // check correctness
        assertTrue(result3.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ3.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianDQ3.equals(jacobianDQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW3.equals(jacobianW2, ABSOLUTE_ERROR));

        // test without jacobians
        final var result4 = new Quaternion();
        QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx, wy, wz, dt, result4);

        // check correctness
        assertTrue(result4.equals(result2, ABSOLUTE_ERROR));

        // test with w array without jacobians
        final var result5 = new Quaternion();
        QuaternionPredictor.predictWithRotationAdjustment(q, dq, w, dt, result5);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q, dq,
                new double[1], dt, result5));

        // test with new instance
        final var result6 = QuaternionPredictor.predictWithRotationAdjustment(q, dq,
                wx, wy, wz, dt, jacobianQ3, jacobianDQ3, jacobianW3);

        // check correctness
        assertTrue(result6.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ3.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianDQ3.equals(jacobianDQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW3.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q,
                dq, wx, wy, wz, dt, m, jacobianDQ3, jacobianW3));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q,
                dq, wx, wy, wz, dt, jacobianQ3, m, jacobianW3));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q,
                dq, wx, wy, wz, dt, jacobianQ3, jacobianDQ3, m));

        // test with new instance and array
        final var result7 = QuaternionPredictor.predictWithRotationAdjustment(q, dq,
                w, dt, jacobianQ3, jacobianDQ3, jacobianW3);

        // check correctness
        assertTrue(result7.equals(result2, ABSOLUTE_ERROR));

        // check jacobians
        assertTrue(jacobianQ3.equals(jacobianQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianDQ3.equals(jacobianDQ2, ABSOLUTE_ERROR));
        assertTrue(jacobianW3.equals(jacobianW2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q,
                dq, new double[1], dt, jacobianQ3, jacobianDQ3, jacobianW3));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q,
                dq, w, dt, m, jacobianDQ3, jacobianW3));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q,
                dq, w, dt, jacobianQ3, m, jacobianW3));
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q,
                dq, w, dt, jacobianQ3, jacobianDQ3, m));

        // test with new instance without jacobians
        final var result8 = QuaternionPredictor.predictWithRotationAdjustment(q, dq, wx, wy, wz, dt);

        // check correctness
        assertTrue(result8.equals(result2, ABSOLUTE_ERROR));

        // test with new instance, with array and without jacobians
        final var result9 = QuaternionPredictor.predictWithRotationAdjustment(q, dq, w, dt);

        // check correctness
        assertTrue(result9.equals(result2, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> QuaternionPredictor.predictWithRotationAdjustment(q, dq,
                new double[1], dt));

        // check correctness of jacobians
        final var jacobianQ4 = new Matrix(4, 4);
        final var jacobianDQ4 = new Matrix(4, 4);
        final var jacobianW4 = new Matrix(4, 3);
        final var result10 = QuaternionPredictor.predictWithRotationAdjustment(q, dq,
                w, dt, jacobianQ4, jacobianDQ4, jacobianW4);

        var diff = new double[4];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var q2 = new Quaternion(q.getA() + diff[0], q.getB() + diff[1],
                q.getC() + diff[2], q.getD() + diff[3]);
        final var result11 = QuaternionPredictor.predictWithRotationAdjustment(q2, dq, w, dt);

        var diffResult = new double[]{
                result11.getA() - result10.getA(),
                result11.getB() - result10.getB(),
                result11.getC() - result10.getC(),
                result11.getD() - result10.getD()
        };
        var diffResult2 = jacobianQ4.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check rotation variation
        diff = new double[4];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var dq2 = new Quaternion(dq.getA() + diff[0],
                dq.getB() + diff[1], dq.getC() + diff[2], dq.getD() + diff[3]);
        final var result12 = QuaternionPredictor.predictWithRotationAdjustment(q, dq2, w, dt);

        diffResult = new double[]{
                result12.getA() - result10.getA(),
                result12.getB() - result10.getB(),
                result12.getC() - result10.getC(),
                result12.getD() - result10.getD()
        };
        diffResult2 = jacobianDQ4.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);

        // check angular speed variation
        diff = new double[3];
        randomizer.fill(diff, -JACOBIAN_ERROR, JACOBIAN_ERROR);
        final var w2 = ArrayUtils.sumAndReturnNew(w, diff);
        final var result13 = QuaternionPredictor.predictWithRotationAdjustment(q, dq, w2, dt);

        diffResult = new double[]{
                result13.getA() - result10.getA(),
                result13.getB() - result10.getB(),
                result13.getC() - result10.getC(),
                result13.getD() - result10.getD()
        };
        diffResult2 = jacobianW4.multiplyAndReturnNew(Matrix.newFromArray(diff)).toArray();
        assertArrayEquals(diffResult, diffResult2, ABSOLUTE_ERROR);
    }
}
