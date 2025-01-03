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
import com.irurueta.geometry.RotationUtils;

/**
 * Utility class to predict rotations.
 */
public class QuaternionPredictor {
    /**
     * Number of components on angular speed.
     */
    public static final int ANGULAR_SPEED_COMPONENTS = 3;

    /**
     * Constructor.
     */
    private QuaternionPredictor() {
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x,y,z (roll, pitch, yaw).
     *
     * @param q           quaternion to be updated.
     * @param wx          angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy          angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz          angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param dt          time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @param result      instance where updated quaternion is stored.
     * @param jacobianQ   jacobian wrt input quaternion. Must be 4x4.
     * @param jacobianW   jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double wx, final double wy, final double wz,
            final double dt, final boolean exactMethod, final Quaternion result, final Matrix jacobianQ,
            final Matrix jacobianW) {

        if (jacobianQ != null && (jacobianQ.getRows() != Quaternion.N_PARAMS
                || jacobianQ.getColumns() != Quaternion.N_PARAMS)) {
            throw new IllegalArgumentException("jacobian wrt q must be 4x4");
        }
        if (jacobianW != null && (jacobianW.getRows() != Quaternion.N_PARAMS
                || jacobianW.getColumns() != ANGULAR_SPEED_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt w must be 4x3");
        }

        final var w = new double[]{wx, wy, wz};

        if (exactMethod) {
            // exact jacobians and rotation
            ArrayUtils.multiplyByScalar(w, dt, w);
            Quaternion.rotationVectorToQuaternion(w, result, jacobianW);
            Matrix jacobianQ2 = null;
            if (jacobianW != null) {
                jacobianW.multiplyByScalar(dt);
                try {
                    jacobianQ2 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
                } catch (final WrongSizeException ignore) {
                    // never happens
                }
            }
            Quaternion.product(q, result, result, jacobianQ, jacobianQ2);

            if (jacobianW != null && jacobianQ2 != null) {
                try {
                    jacobianQ2.multiply(jacobianW);
                    jacobianW.copyFrom(jacobianQ2);
                } catch (final WrongSizeException ignore) {
                    // never happens
                }
            }
        } else {
            // tustin integration - fits with jacobians
            var skewW = RotationUtils.w2omega(w);
            try {
                final var qMatrix = new Matrix(Quaternion.N_PARAMS, 1);
                qMatrix.setElementAtIndex(0, q.getA());
                qMatrix.setElementAtIndex(1, q.getB());
                qMatrix.setElementAtIndex(2, q.getC());
                qMatrix.setElementAtIndex(3, q.getD());
                skewW.multiply(qMatrix);
            } catch (final WrongSizeException ignore) {
                // never happens
            }

            skewW.multiplyByScalar(0.5 * dt);

            result.setA(q.getA() + skewW.getElementAtIndex(0));
            result.setB(q.getB() + skewW.getElementAtIndex(1));
            result.setC(q.getC() + skewW.getElementAtIndex(2));
            result.setD(q.getD() + skewW.getElementAtIndex(3));

            if (jacobianQ != null) {
                try {
                    // reset w values to reuse the same array as they might have been
                    // modified
                    w[0] = wx;
                    w[1] = wy;
                    w[2] = wz;

                    skewW = RotationUtils.w2omega(w);
                    jacobianQ.copyFrom(Matrix.identity(Quaternion.N_PARAMS, Quaternion.N_PARAMS));
                    jacobianQ.add(skewW);
                    jacobianQ.multiplyByScalar(0.5 * dt);
                } catch (final WrongSizeException ignore) {
                    // never happens
                }
            }

            if (jacobianW != null) {
                RotationUtils.quaternionToConjugatedPiMatrix(q, jacobianW);
                jacobianW.multiplyByScalar(0.5 * dt);
            }
        }
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     *
     * @param q           quaternion to be updated.
     * @param w           array containing angular speed in the 3 axis (x = roll,
     *                    y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt          time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @param result      instance where update quaternion is stored.
     * @param jacobianQ   jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW   jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size, or w does not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double[] w, final double dt,
            final boolean exactMethod, final Quaternion result, final Matrix jacobianQ, final Matrix jacobianW) {
        if (w.length != ANGULAR_SPEED_COMPONENTS) {
            throw new IllegalArgumentException("w must have length 3");
        }
        predict(q, w[0], w[1], w[2], dt, exactMethod, result, jacobianQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using
     * exact method.
     *
     * @param q         quaternion to be updated.
     * @param wx        angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy        angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz        angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param dt        time interval to compute prediction expressed in seconds.
     * @param result    instance where update quaternion is stored.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double wx, final double wy, final double wz, final double dt,
            final Quaternion result, final Matrix jacobianQ, final Matrix jacobianW) {
        predict(q, wx, wy, wz, dt, true, result, jacobianQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact
     * method.
     *
     * @param q         quaternion to be updated.
     * @param w         array containing angular speed in the 3 axis (x = roll,
     *                  y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt        time interval to compute prediction expressed in seconds.
     * @param result    instance where update quaternion is stored.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double[] w, final double dt, final Quaternion result, final Matrix jacobianQ,
            final Matrix jacobianW) {
        predict(q, w, dt, true, result, jacobianQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     *
     * @param q           quaternion to be updated.
     * @param wx          angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy          angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz          angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param dt          time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @param result      instance where update quaternion is stored.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double wx, final double wy, final double wz, final double dt,
            final boolean exactMethod, final Quaternion result) {
        predict(q, wx, wy, wz, dt, exactMethod, result, null, null);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     *
     * @param q           quaternion to be updated.
     * @param w           array containing angular speed in the 3 axis (x = roll,
     *                    y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt          time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @param result      instance where update quaternion is stored.
     * @throws IllegalArgumentException if w does not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double[] w, final double dt, final boolean exactMethod, final Quaternion result) {
        predict(q, w, dt, exactMethod, result, null, null);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact
     * method.
     *
     * @param q      quaternion to be updated.
     * @param wx     angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy     angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz     angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param dt     time interval to compute prediction expressed in seconds.
     * @param result instance where update quaternion is stored.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double wx, final double wy, final double wz, final double dt,
            final Quaternion result) {
        predict(q, wx, wy, wz, dt, result, null, null);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact
     * method.
     *
     * @param q      quaternion to be updated.
     * @param w      array containing angular speed in the 3 axis (x = roll,
     *               y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt     time interval to compute prediction expressed in seconds.
     * @param result instance where update quaternion is stored.
     * @throws IllegalArgumentException if w does not have length 3
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double[] w, final double dt, final Quaternion result) {
        predict(q, w, dt, result, null, null);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     *
     * @param q           quaternion to be updated.
     * @param wx          angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy          angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz          angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param dt          time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @param jacobianQ   jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW   jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(
            final Quaternion q, final double wx, final double wy, final double wz, final double dt,
            final boolean exactMethod, final Matrix jacobianQ, final Matrix jacobianW) {
        final var result = new Quaternion();
        predict(q, wx, wy, wz, dt, exactMethod, result, jacobianQ, jacobianW);
        return result;
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     *
     * @param q           quaternion to be updated.
     * @param w           array containing angular speed in the 3 axis (x = roll,
     *                    y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt          time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @param jacobianQ   jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW   jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size or w does not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(
            final Quaternion q, final double[] w, final double dt, final boolean exactMethod, final Matrix jacobianQ,
            final Matrix jacobianW) {
        final var result = new Quaternion();
        predict(q, w, dt, exactMethod, result, jacobianQ, jacobianW);
        return result;
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact
     * method.
     *
     * @param q         quaternion to be updated.
     * @param wx        angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy        angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz        angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param dt        time interval to compute prediction expressed in seconds.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(
            final Quaternion q, final double wx, final double wy, final double wz, final double dt,
            final Matrix jacobianQ, final Matrix jacobianW) {
        final var result = new Quaternion();
        predict(q, wx, wy, wz, dt, result, jacobianQ, jacobianW);
        return result;
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact
     * method.
     *
     * @param q         quaternion to be updated.
     * @param w         array containing angular speed in the 3 axis (x = roll,
     *                  y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt        time interval to compute prediction expressed in seconds.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(
            final Quaternion q, final double[] w, final double dt, final Matrix jacobianQ, final Matrix jacobianW) {
        final var result = new Quaternion();
        predict(q, w, dt, result, jacobianQ, jacobianW);
        return result;
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     *
     * @param q           quaternion to be updated.
     * @param wx          angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy          angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz          angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param dt          time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @return a new quaternion containing updated quaternion.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(
            final Quaternion q, final double wx, final double wy, final double wz, final double dt,
            final boolean exactMethod) {
        final var result = new Quaternion();
        predict(q, wx, wy, wz, dt, exactMethod, result);
        return result;
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     *
     * @param q           quaternion to be updated.
     * @param w           array containing angular speed in the 3 axis (x = roll,
     *                    y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt          time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if w does not have length 3
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(final Quaternion q, final double[] w, final double dt, final boolean exactMethod) {
        final var result = new Quaternion();
        predict(q, w, dt, exactMethod, result);
        return result;
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact
     * method.
     *
     * @param q  quaternion to be updated.
     * @param wx angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new quaternion containing updated quaternion.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(final Quaternion q, final double wx, final double wy, final double wz,
                                     final double dt) {
        final var result = new Quaternion();
        predict(q, wx, wy, wz, dt, result);
        return result;
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact
     * method.
     *
     * @param q  quaternion to be updated.
     * @param w  array containing angular speed in the 3 axis (x = roll,
     *           y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if w does not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(final Quaternion q, final double[] w, final double dt) {
        final var result = new Quaternion();
        predict(q, w, dt, result);
        return result;
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second.
     *
     * @param q           quaternion to be updated.
     * @param wx          angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy          angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz          angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @param result      instance where update quaternion is stored.
     * @param jacobianQ   jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW   jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double wx, final double wy, final double wz, final boolean exactMethod,
            final Quaternion result, final Matrix jacobianQ, final Matrix jacobianW) {
        predict(q, wx, wy, wz, 1.0, exactMethod, result, jacobianQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second.
     *
     * @param q           quaternion to be updated.
     * @param w           array containing angular speed in the 3 axis (x = roll,
     *                    y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @param result      instance where update quaternion is stored.
     * @param jacobianQ   jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW   jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double[] w, final boolean exactMethod, final Quaternion result,
            final Matrix jacobianQ, final Matrix jacobianW) {
        predict(q, w, 1.0, exactMethod, result, jacobianQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second and exact method.
     *
     * @param q         quaternion to be updated.
     * @param wx        angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy        angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz        angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param result    instance where update quaternion is stored.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double wx, final double wy, final double wz, final Quaternion result,
            final Matrix jacobianQ, final Matrix jacobianW) {
        predict(q, wx, wy, wz, 1.0, result, jacobianQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second and exact method.
     *
     * @param q         quaternion to be updated.
     * @param w         array containing angular speed in the 3 axis (x = roll,
     *                  y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param result    instance where update quaternion is stored.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double[] w, final Quaternion result, final Matrix jacobianQ,
            final Matrix jacobianW) {
        predict(q, w, 1.0, result, jacobianQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second.
     *
     * @param q           quaternion to be updated.
     * @param wx          angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy          angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz          angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @param result      instance where update quaternion is stored.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double wx, final double wy, final double wz, final boolean exactMethod,
            final Quaternion result) {
        predict(q, wx, wy, wz, 1.0, exactMethod, result);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second.
     *
     * @param q           quaternion to be updated.
     * @param w           array containing angular speed in the 3 axis (x = roll,
     *                    y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @param result      instance where update quaternion is stored.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double[] w, final boolean exactMethod, final Quaternion result) {
        predict(q, w, 1.0, exactMethod, result);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second and exact method.
     *
     * @param q      quaternion to be updated.
     * @param wx     angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy     angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz     angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param result instance where update quaternion is stored.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(
            final Quaternion q, final double wx, final double wy, final double wz, final Quaternion result) {
        predict(q, wx, wy, wz, 1.0, result);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second and exact method.
     *
     * @param q      quaternion to be updated.
     * @param w      array containing angular speed in the 3 axis (x = roll,
     *               y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param result instance where update quaternion is stored.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(final Quaternion q, final double[] w, final Quaternion result) {
        predict(q, w, 1.0, result);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second.
     *
     * @param q           quaternion to be updated.
     * @param wx          angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy          angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz          angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @param jacobianQ   jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW   jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(
            final Quaternion q, final double wx, final double wy, final double wz, final boolean exactMethod,
            final Matrix jacobianQ, final Matrix jacobianW) {
        return predict(q, wx, wy, wz, 1.0, exactMethod, jacobianQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second.
     *
     * @param q           quaternion to be updated.
     * @param w           array containing angular speed in the 3 axis (x = roll,
     *                    y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @param jacobianQ   jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW   jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(
            final Quaternion q, final double[] w, final boolean exactMethod, final Matrix jacobianQ,
            final Matrix jacobianW) {
        return predict(q, w, 1.0, exactMethod, jacobianQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second and exact method.
     *
     * @param q         quaternion to be updated.
     * @param wx        angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy        angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz        angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(
            final Quaternion q, final double wx, final double wy, final double wz, final Matrix jacobianQ,
            final Matrix jacobianW) {
        return predict(q, wx, wy, wz, 1.0, jacobianQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second and exact method.
     *
     * @param q         quaternion to be updated.
     * @param w         array containing angular speed in the 3 axis (x = roll,
     *                  y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(final Quaternion q, final double[] w, final Matrix jacobianQ,
                                     final Matrix jacobianW) {
        return predict(q, w, 1.0, jacobianQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second.
     *
     * @param q           quaternion to be updated.
     * @param wx          angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy          angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz          angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @return a new quaternion containing updated quaternion.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(
            final Quaternion q, final double wx, final double wy, final double wz, final boolean exactMethod) {
        return predict(q, wx, wy, wz, 1.0, exactMethod);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second.
     *
     * @param q           quaternion to be updated.
     * @param w           array containing angular speed in the 3 axis (x = roll,
     *                    y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param exactMethod true to use exact method, false to use "Tustin"
     *                    method.
     * @return a new quaternion containing updated quaternion.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(final Quaternion q, final double[] w, final boolean exactMethod) {
        return predict(q, w, 1.0, exactMethod);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second and exact method.
     *
     * @param q  quaternion to be updated.
     * @param wx angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @return a new quaternion containing updated quaternion.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(final Quaternion q, final double wx, final double wy, final double wz) {
        return predict(q, wx, wy, wz, 1.0);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a
     * time interval of 1 second and exact method.
     *
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll,
     *          y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(final Quaternion q, final double[] w) {
        return predict(q, w, 1.0);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes
     * x,y,z (roll, pitch, yaw).
     *
     * @param q          quaternion to be updated.
     * @param dq         adjustment of rotation to be combined with input quaternion.
     * @param wx         angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy         angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz         angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param dt         time interval to compute prediction expressed in seconds.
     * @param result     instance where updated quaternion is stored.
     * @param jacobianQ  jacobian wrt input quaternion. Must be 4x4.
     * @param jacobianDQ jacobian wrt dq quaternion. Must be 4x4.
     * @param jacobianW  jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     */
    public static void predictWithRotationAdjustment(
            final Quaternion q, final Quaternion dq, final double wx, final double wy, final double wz, final double dt,
            final Quaternion result, final Matrix jacobianQ, final Matrix jacobianDQ, final Matrix jacobianW) {

        if (jacobianQ != null && (jacobianQ.getRows() != Quaternion.N_PARAMS
                || jacobianQ.getColumns() != Quaternion.N_PARAMS)) {
            throw new IllegalArgumentException("jacobian wrt q must be 4x4");
        }
        if (jacobianDQ != null && (jacobianDQ.getRows() != Quaternion.N_PARAMS
                || jacobianDQ.getColumns() != Quaternion.N_PARAMS)) {
            throw new IllegalArgumentException("jacobian wrt dq must be 4x4");
        }
        if (jacobianW != null && (jacobianW.getRows() != Quaternion.N_PARAMS
                || jacobianW.getColumns() != ANGULAR_SPEED_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt w must be 4x3");
        }

        final var w = new double[]{wx, wy, wz};

        ArrayUtils.multiplyByScalar(w, dt, w);
        Quaternion.rotationVectorToQuaternion(w, result, jacobianW);
        Matrix jacobianQ2 = null;
        if (jacobianW != null) {
            jacobianW.multiplyByScalar(dt);
        }
        if (jacobianW != null) {
            try {
                jacobianQ2 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            } catch (final WrongSizeException ignore) {
                // never happens
            }
        }
        Quaternion.product(dq, result, result, jacobianDQ, jacobianQ2);

        Matrix jacobianQ3 = null;
        if (jacobianDQ != null || jacobianW != null) {
            try {
                jacobianQ3 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            } catch (final WrongSizeException ignore) {
                // never happens
            }
        }
        Quaternion.product(q, result, result, jacobianQ, jacobianQ3);

        // chain rule
        if (jacobianQ3 != null) {
            if (jacobianDQ != null) {
                try {
                    final var tmp = jacobianQ3.multiplyAndReturnNew(jacobianDQ);
                    jacobianDQ.copyFrom(tmp);
                } catch (final WrongSizeException ignore) {
                    // never happens
                }
            }

            // chain rule (jacobianW is already multiplied by dt)
            if (jacobianW != null && jacobianQ2 != null) {
                try {
                    jacobianQ3.multiply(jacobianQ2);
                    jacobianQ3.multiply(jacobianW);
                    jacobianW.copyFrom(jacobianQ3);
                } catch (final WrongSizeException ignore) {
                    // never happens
                }
            }
        }
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes x, y,
     * z (roll, pitch, yaw).
     *
     * @param q          quaternion to be updated.
     * @param dq         adjustment of rotation to be combined with input quaternion.
     * @param w          angular speed (x, y, z axes). Expressed in rad/s. Must have
     *                   length 3.
     * @param dt         time interval to compute prediction expressed in seconds.
     * @param result     instance where updated quaternion is stored.
     * @param jacobianQ  jacobian wrt input quaternion. Must be 4x4.
     * @param jacobianDQ jacobian wrt dq quaternion. Must be 4x4.
     * @param jacobianW  jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size or if w does not have length 3.
     */
    public static void predictWithRotationAdjustment(
            final Quaternion q, final Quaternion dq, final double[] w, final double dt, final Quaternion result,
            final Matrix jacobianQ, final Matrix jacobianDQ, final Matrix jacobianW) {
        if (w.length != ANGULAR_SPEED_COMPONENTS) {
            throw new IllegalArgumentException("w must have length 3");
        }
        predictWithRotationAdjustment(q, dq, w[0], w[1], w[2], dt, result, jacobianQ, jacobianDQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes
     * x, y, z (roll, pitch, yaw).
     *
     * @param q      quaternion to be updated.
     * @param dq     adjustment of rotation to be combined with input quaternion.
     * @param wx     angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy     angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz     angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param dt     time interval to compute prediction expressed in seconds.
     * @param result instance where updated quaternion is stored.
     */
    public static void predictWithRotationAdjustment(
            final Quaternion q, final Quaternion dq, final double wx, final double wy, final double wz, final double dt,
            final Quaternion result) {
        predictWithRotationAdjustment(q, dq, wx, wy, wz, dt, result, null, null, null);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes x,y,z
     * (roll, pitch, yaw).
     *
     * @param q      quaternion to be updated.
     * @param dq     adjustment of rotation to be combined with input quaternion.
     * @param w      angular speed (x, y, z axes). Expressed in rad/s. Must have
     *               length 3.
     * @param dt     time interval to compute prediction expressed in seconds.
     * @param result instance where updated quaternion is stored.
     * @throws IllegalArgumentException if w does not have length 3.
     */
    public static void predictWithRotationAdjustment(
            final Quaternion q, final Quaternion dq, final double[] w, final double dt, final Quaternion result) {
        predictWithRotationAdjustment(q, dq, w, dt, result, null, null, null);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes x,y,z
     * (roll, pitch, yaw).
     *
     * @param q          quaternion to be updated.
     * @param dq         adjustment of rotation to be combined with input quaternion.
     * @param wx         angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy         angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz         angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param dt         time interval to compute prediction expressed in seconds.
     * @param jacobianQ  jacobian wrt input quaternion. Must be 4x4.
     * @param jacobianDQ jacobian wrt dq quaternion. Must be 4x4.
     * @param jacobianW  jacobian wrt angular speed. Must be 4x3.
     * @return a new updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     */
    public static Quaternion predictWithRotationAdjustment(
            final Quaternion q, final Quaternion dq, final double wx, final double wy, final double wz, final double dt,
            final Matrix jacobianQ, final Matrix jacobianDQ, final Matrix jacobianW) {
        final var result = new Quaternion();
        predictWithRotationAdjustment(q, dq, wx, wy, wz, dt, result, jacobianQ, jacobianDQ, jacobianW);
        return result;
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes x,y,z
     * (roll, pitch, yaw).
     *
     * @param q          quaternion to be updated.
     * @param dq         adjustment of rotation to be combined with input quaternion.
     * @param w          angular speed (x,y,z axes). Expressed in rad/s. Must have length
     *                   3.
     * @param dt         time interval to compute prediction expressed in seconds.
     * @param jacobianQ  jacobian wrt input quaternion. Must be 4x4.
     * @param jacobianDQ jacobian wrt dq quaternion. Must be 4x4.
     * @param jacobianW  jacobian wrt angular speed. Must be 4x3.
     * @return a new updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     *                                  have proper size.
     */
    public static Quaternion predictWithRotationAdjustment(
            final Quaternion q, final Quaternion dq, final double[] w, final double dt, final Matrix jacobianQ,
            final Matrix jacobianDQ, final Matrix jacobianW) {
        final var result = new Quaternion();
        predictWithRotationAdjustment(q, dq, w, dt, result, jacobianQ, jacobianDQ, jacobianW);
        return result;
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes x,y,z
     * (roll, pitch, yaw).
     *
     * @param q  quaternion to be updated.
     * @param dq adjustment of rotation to be combined with input quaternion.
     * @param wx angular speed in x-axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y-axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z-axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new updated quaternion.
     */
    public static Quaternion predictWithRotationAdjustment(
            final Quaternion q, final Quaternion dq, final double wx, final double wy, final double wz,
            final double dt) {
        final var result = new Quaternion();
        predictWithRotationAdjustment(q, dq, wx, wy, wz, dt, result);
        return result;
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes x,y,z
     * (roll, pitch, yaw).
     *
     * @param q  quaternion to be updated.
     * @param dq adjustment of rotation to be combined with input quaternion.
     * @param w  angular speed (x, y, z axes). Expressed in rad/s. Must have
     *           length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new updated quaternion.
     * @throws IllegalArgumentException if w does not have length 3.
     */
    public static Quaternion predictWithRotationAdjustment(
            final Quaternion q, final Quaternion dq, final double[] w, final double dt) {
        final var result = new Quaternion();
        predictWithRotationAdjustment(q, dq, w, dt, result);
        return result;
    }
}
