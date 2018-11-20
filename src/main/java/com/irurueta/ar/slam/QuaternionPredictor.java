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
@SuppressWarnings("WeakerAccess")
public class QuaternionPredictor {
    /**
     * Number of components on angular speed.
     */
    public static final int ANGULAR_SPEED_COMPONENTS = 3;

    /**
     * Constructor.
     */
    private QuaternionPredictor() { }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x,y,z (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @param result instance where updated quaternion is stored.
     * @param jacobianQ jacobian wrt input quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */    
    public static void predict(Quaternion q, double wx, double wy, double wz,
            double dt, boolean exactMethod, Quaternion result, Matrix jacobianQ,
            Matrix jacobianW) throws IllegalArgumentException {
        
        if(jacobianQ != null && (jacobianQ.getRows() != Quaternion.N_PARAMS ||
                jacobianQ.getColumns() != Quaternion.N_PARAMS)) {
            throw new IllegalArgumentException("jacobian wrt q must be 4x4");
        }
        if(jacobianW != null && (jacobianW.getRows() != Quaternion.N_PARAMS ||
                jacobianW.getColumns() != ANGULAR_SPEED_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt w must be 4x3");
        }
        
        double[] w = new double[]{ wx, wy, wz };        
        
        if (exactMethod) {
            //exact jacobians and rotation
            
            ArrayUtils.multiplyByScalar(w, dt, w);                     
            Quaternion.rotationVectorToQuaternion(w, result, jacobianW);
            Matrix jacobianQ2 = null;
            if(jacobianW != null) {
                jacobianW.multiplyByScalar(dt);
                try {
                    jacobianQ2 = new Matrix(Quaternion.N_PARAMS, 
                            Quaternion.N_PARAMS);
                } catch(WrongSizeException ignore) { }
            }
            Quaternion.product(q, result, result, jacobianQ, jacobianQ2);
            
            if(jacobianW != null && jacobianQ2 != null) {
                try {
                    jacobianQ2.multiply(jacobianW);
                    jacobianW.copyFrom(jacobianQ2);
                } catch(WrongSizeException ignore) { }
            }
        } else {
            //tustin integration - fits with jacobians
            Matrix W = RotationUtils.w2omega(w);
            try {
                Matrix Q = new Matrix(Quaternion.N_PARAMS, 1);
                Q.setElementAtIndex(0, q.getA());
                Q.setElementAtIndex(1, q.getB());
                Q.setElementAtIndex(2, q.getC());
                Q.setElementAtIndex(3, q.getD());
                W.multiply(Q);
            } catch(WrongSizeException ignore) { }
            
            W.multiplyByScalar(0.5 * dt);
            
            result.setA(q.getA() + W.getElementAtIndex(0));
            result.setB(q.getB() + W.getElementAtIndex(1));
            result.setC(q.getC() + W.getElementAtIndex(2));
            result.setD(q.getD() + W.getElementAtIndex(3));
            
            if(jacobianQ != null) {
                try {
                    //reset w values to reuse the same array as they might have been 
                    //modified
                    w[0] = wx;
                    w[1] = wy;
                    w[2] = wz;
                
                    W = RotationUtils.w2omega(w);
                    jacobianQ.copyFrom(Matrix.identity(Quaternion.N_PARAMS, 
                            Quaternion.N_PARAMS));
                    jacobianQ.add(W);
                    jacobianQ.multiplyByScalar(0.5*dt);
                } catch(WrongSizeException ignore) { }
            }
        
            if(jacobianW != null) {
                RotationUtils.quaternionToConjugatedPiMatrix(q, jacobianW);
                jacobianW.multiplyByScalar(0.5*dt);
            }            
        }        
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @param result instance where update quaternion is stored.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size, or w does not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double[] w, double dt,
            boolean exactMethod, Quaternion result, Matrix jacobianQ, 
            Matrix jacobianW) throws IllegalArgumentException {
        if (w.length != ANGULAR_SPEED_COMPONENTS) {
            throw new IllegalArgumentException("w must have length 3");
        }
        predict(q, w[0], w[1], w[2], dt, exactMethod, result, jacobianQ, 
                jacobianW);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using 
     * exact method.
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where update quaternion is stored.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double wx, double wy,
            double wz, double dt, Quaternion result, Matrix jacobianQ, 
            Matrix jacobianW) throws IllegalArgumentException {
        predict(q, wx, wy, wz, dt, true, result, jacobianQ, jacobianW);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact 
     * method.
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where update quaternion is stored.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double[] w, double dt,
            Quaternion result, Matrix jacobianQ, Matrix jacobianW)
            throws IllegalArgumentException {
        predict(q, w, dt, true, result, jacobianQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @param result instance where update quaternion is stored.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double wx, double wy,
            double wz, double dt, boolean exactMethod, Quaternion result) {
        predict(q, wx, wy, wz, dt, exactMethod, result, null, null);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @param result instance where update quaternion is stored.
     * @throws IllegalArgumentException if w does not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double[] w, double dt,
            boolean exactMethod, Quaternion result) 
            throws IllegalArgumentException {
        predict(q, w, dt, exactMethod, result, null, null);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact 
     * method.
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where update quaternion is stored.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double wx, double wy, 
            double wz, double dt, Quaternion result) {
        predict(q, wx, wy, wz, dt, result, null, null);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact 
     * method.
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where update quaternion is stored.
     * @throws IllegalArgumentException if w does not have length 3
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double[] w, double dt,
            Quaternion result) throws IllegalArgumentException {
        predict(q, w, dt, result, null, null);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double wx, 
            double wy, double wz, double dt, boolean exactMethod, 
            Matrix jacobianQ, Matrix jacobianW) 
            throws IllegalArgumentException {
        Quaternion result = new Quaternion();
        predict(q, wx, wy, wz, dt, exactMethod, result, jacobianQ, jacobianW);
        return result;
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size or w does not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double[] w, 
            double dt, boolean exactMethod, Matrix jacobianQ, Matrix jacobianW)
            throws IllegalArgumentException {
        Quaternion result = new Quaternion();
        predict(q, w, dt, exactMethod, result, jacobianQ, jacobianW);
        return result;
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact
     * method.
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double wx, 
            double wy, double wz, double dt, Matrix jacobianQ, Matrix jacobianW) 
            throws IllegalArgumentException {
        Quaternion result = new Quaternion();
        predict(q, wx, wy, wz, dt, result, jacobianQ, jacobianW);
        return result;
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact
     * method.
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double[] w, 
            double dt, Matrix jacobianQ, Matrix jacobianW) 
            throws IllegalArgumentException {
        Quaternion result = new Quaternion();
        predict(q, w, dt, result, jacobianQ, jacobianW);
        return result;
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @return a new quaternion containing updated quaternion.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double wx, 
            double wy, double wz, double dt, boolean exactMethod) {
        Quaternion result = new Quaternion();
        predict(q, wx, wy, wz, dt, exactMethod, result);
        return result;
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt time interval to compute prediction expressed in seconds.
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if w does not have length 3
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double[] w, 
            double dt, boolean exactMethod) throws IllegalArgumentException {
        Quaternion result = new Quaternion();
        predict(q, w, dt, exactMethod, result);
        return result;
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact 
     * method.
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new quaternion containing updated quaternion.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double wx, 
            double wy, double wz, double dt) {
        Quaternion result = new Quaternion();
        predict(q, wx, wy, wz, dt, result);
        return result;
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) using exact 
     * method.
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if w does not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double[] w, 
            double dt) throws IllegalArgumentException {
        Quaternion result = new Quaternion();
        predict(q, w, dt, result);
        return result;
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second.
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @param result instance where update quaternion is stored.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double wx, double wy, 
            double wz, boolean exactMethod, Quaternion result, Matrix jacobianQ,
            Matrix jacobianW) throws IllegalArgumentException {
        predict(q, wx, wy, wz, 1.0, exactMethod, result, jacobianQ, 
                jacobianW);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second.
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @param result instance where update quaternion is stored.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double[] w, 
            boolean exactMethod, Quaternion result, Matrix jacobianQ, 
            Matrix jacobianW) throws IllegalArgumentException {
        predict(q, w, 1.0, exactMethod, result, jacobianQ, jacobianW);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second and exact method.
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param result instance where update quaternion is stored.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double wx, double wy,
            double wz, Quaternion result, Matrix jacobianQ, Matrix jacobianW) 
            throws IllegalArgumentException {
        predict(q, wx, wy, wz, 1.0, result, jacobianQ, jacobianW);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second and exact method.
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param result instance where update quaternion is stored.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double[] w, 
            Quaternion result, Matrix jacobianQ, Matrix jacobianW)
            throws IllegalArgumentException {
        predict(q, w, 1.0, result, jacobianQ, jacobianW);
    }

    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second.
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @param result instance where update quaternion is stored.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double wx, double wy,
            double wz, boolean exactMethod, Quaternion result) {
        predict(q, wx, wy, wz, 1.0, exactMethod, result);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second.
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @param result instance where update quaternion is stored.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double[] w, 
            boolean exactMethod, Quaternion result) 
            throws IllegalArgumentException {
        predict(q, w, 1.0, exactMethod, result);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second and exact method.
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param result instance where update quaternion is stored.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double wx, double wy, 
            double wz, Quaternion result) {
        predict(q, wx, wy, wz, 1.0, result);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second and exact method.
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param result instance where update quaternion is stored.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(Quaternion q, double[] w, 
            Quaternion result) throws IllegalArgumentException {
        predict(q, w, 1.0, result);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second.
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double wx, 
            double wy, double wz, boolean exactMethod, 
            Matrix jacobianQ, Matrix jacobianW) 
            throws IllegalArgumentException {
        return predict(q, wx, wy, wz, 1.0, exactMethod, jacobianQ, jacobianW);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second.
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double[] w, 
            boolean exactMethod, Matrix jacobianQ, Matrix jacobianW)
            throws IllegalArgumentException {
        return predict(q, w, 1.0, exactMethod, jacobianQ, jacobianW);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second and exact method.
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double wx, 
            double wy, double wz, Matrix jacobianQ, Matrix jacobianW) 
            throws IllegalArgumentException {
        return predict(q, wx, wy, wz, 1.0, jacobianQ, jacobianW);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second and exact method.
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param jacobianQ jacobian wrt quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double[] w, 
            Matrix jacobianQ, Matrix jacobianW) 
            throws IllegalArgumentException {
        return predict(q, w, 1.0, jacobianQ, jacobianW);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second.
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @return a new quaternion containing updated quaternion.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double wx, 
            double wy, double wz, boolean exactMethod) {
        return predict(q, wx, wy, wz, 1.0, exactMethod);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second.
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @param exactMethod true to use exact method, false to use "Tustin" 
     * method.
     * @return a new quaternion containing updated quaternion.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double[] w, 
            boolean exactMethod) {
        return predict(q, w, 1.0, exactMethod);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second and exact method.
     * @param q quaternion to be updated.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @return a new quaternion containing updated quaternion.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double wx, 
            double wy, double wz) {
        return predict(q, wx, wy, wz, 1.0);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed 
     * by the rate of rotation along axes x, y, z (roll, pitch, yaw) assuming a 
     * time interval of 1 second and exact method.
     * @param q quaternion to be updated.
     * @param w array containing angular speed in the 3 axis (x = roll, 
     * y = pitch, z = yaw). Expressed in rad/se. Must have length 3
     * @return a new quaternion containing updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">qpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static Quaternion predict(Quaternion q, double[] w) 
            throws IllegalArgumentException {
        return predict(q, w, 1.0);
    }
     
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes 
     * x,y,z (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param dq adjustment of rotation to be combined with input quaternion.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated quaternion is stored.
     * @param jacobianQ jacobian wrt input quaternion. Must be 4x4.
     * @param jacobianDQ jacobian wrt dq quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size.
     */
    public static void predictWithRotationAdjustment(Quaternion q, 
            Quaternion dq, double wx, double wy, double wz, double dt, 
            Quaternion result, Matrix jacobianQ, Matrix jacobianDQ, 
            Matrix jacobianW) throws IllegalArgumentException {
        
        if(jacobianQ != null && (jacobianQ.getRows() != Quaternion.N_PARAMS ||
                jacobianQ.getColumns() != Quaternion.N_PARAMS)) {
            throw new IllegalArgumentException("jacobian wrt q must be 4x4");
        }
        if(jacobianDQ != null && (jacobianDQ.getRows() != Quaternion.N_PARAMS ||
                jacobianDQ.getColumns() != Quaternion.N_PARAMS)) {
            throw new IllegalArgumentException("jacobian wrt dq must be 4x4");
        }
        if(jacobianW != null && (jacobianW.getRows() != Quaternion.N_PARAMS ||
                jacobianW.getColumns() != ANGULAR_SPEED_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt w must be 4x3");
        }
        
        double[] w = new double[]{ wx, wy, wz };
        
        ArrayUtils.multiplyByScalar(w, dt, w);
        Quaternion.rotationVectorToQuaternion(w, result, jacobianW);
        Matrix jacobianQ2 = null;
        if(jacobianW != null) {
            jacobianW.multiplyByScalar(dt);
        }
        if(jacobianW != null) {
            try {
                jacobianQ2 = new Matrix(Quaternion.N_PARAMS, 
                        Quaternion.N_PARAMS);
            } catch(WrongSizeException ignore) { }
        }
        Quaternion.product(dq, result, result, jacobianDQ, jacobianQ2);
        
        Matrix jacobianQ3 = null;
        if(jacobianDQ != null || jacobianW != null) {
            try {
                jacobianQ3 = new Matrix(Quaternion.N_PARAMS,
                        Quaternion.N_PARAMS);
            } catch(WrongSizeException ignore) { }
        }
        Quaternion.product(q, result, result, jacobianQ, jacobianQ3);
        
        //chain rule
        if (jacobianQ3 != null) {
            if (jacobianDQ != null) {
                try {
                    Matrix tmp = jacobianQ3.multiplyAndReturnNew(jacobianDQ);
                    jacobianDQ.copyFrom(tmp);
                } catch (WrongSizeException ignore) { }
            }

            //chain rule (jacobianW is already multiplied by dt)
            if (jacobianW != null && jacobianQ2 != null) {
                try {
                    jacobianQ3.multiply(jacobianQ2);
                    jacobianQ3.multiply(jacobianW);
                    jacobianW.copyFrom(jacobianQ3);
                } catch (WrongSizeException ignore) { }
            }
        }
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes x, y,
     * z (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param dq adjustment of rotation to be combined with input quaternion.
     * @param w angular speed (x, y, z axes). Expressed in rad/s. Must have 
     * length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated quaternion is stored.
     * @param jacobianQ jacobian wrt input quaternion. Must be 4x4.
     * @param jacobianDQ jacobian wrt dq quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size or if w does not have length 3.
     */
    public static void predictWithRotationAdjustment(Quaternion q,
            Quaternion dq, double[] w, double dt, Quaternion result, 
            Matrix jacobianQ, Matrix jacobianDQ, Matrix jacobianW)
            throws IllegalArgumentException {
        if(w.length != ANGULAR_SPEED_COMPONENTS) {
            throw new IllegalArgumentException("w must have length 3");
        }
        predictWithRotationAdjustment(q, dq, w[0], w[1], w[2], dt, result, 
                jacobianQ, jacobianDQ, jacobianW);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes 
     * x, y, z (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param dq adjustment of rotation to be combined with input quaternion.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated quaternion is stored.
     */
    public static void predictWithRotationAdjustment(Quaternion q,
            Quaternion dq, double wx, double wy, double wz, double dt,
            Quaternion result) {
        predictWithRotationAdjustment(q, dq, wx, wy, wz, dt, result, null, null,
                null);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes x,y,z
     * (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param dq adjustment of rotation to be combined with input quaternion.
     * @param w angular speed (x, y, z axes). Expressed in rad/s. Must have 
     * length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated quaternion is stored.
     * @throws IllegalArgumentException if w does not have length 3.
     */
    public static void predictWithRotationAdjustment(Quaternion q,
            Quaternion dq, double[] w, double dt, Quaternion result) 
            throws IllegalArgumentException {
        predictWithRotationAdjustment(q, dq, w, dt, result, null, null, null);
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes x,y,z
     * (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param dq adjustment of rotation to be combined with input quaternion.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianQ jacobian wrt input quaternion. Must be 4x4.
     * @param jacobianDQ jacobian wrt dq quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size.
     */
    public static Quaternion predictWithRotationAdjustment(Quaternion q, 
            Quaternion dq, double wx, double wy, double wz, double dt, 
            Matrix jacobianQ, Matrix jacobianDQ, Matrix jacobianW) 
            throws IllegalArgumentException {
        Quaternion result = new Quaternion();
        predictWithRotationAdjustment(q, dq, wx, wy, wz, dt, result, jacobianQ, 
                jacobianDQ, jacobianW);
        return result;
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes x,y,z
     * (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param dq adjustment of rotation to be combined with input quaternion.
     * @param w angular speed (x,y,z axes). Expressed in rad/s. Must have length 
     * 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianQ jacobian wrt input quaternion. Must be 4x4.
     * @param jacobianDQ jacobian wrt dq quaternion. Must be 4x4.
     * @param jacobianW jacobian wrt angular speed. Must be 4x3.
     * @return a new updated quaternion.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     */
    public static Quaternion predictWithRotationAdjustment(Quaternion q, 
            Quaternion dq, double[] w, double dt, Matrix jacobianQ, 
            Matrix jacobianDQ, Matrix jacobianW) 
            throws IllegalArgumentException {
        Quaternion result = new Quaternion();
        predictWithRotationAdjustment(q, dq, w, dt, result, jacobianQ, 
                jacobianDQ, jacobianW);
        return result;
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes x,y,z
     * (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param dq adjustment of rotation to be combined with input quaternion.
     * @param wx angular speed in x axis (roll axis). Expressed in rad/s.
     * @param wy angular speed in y axis (pitch axis). Expressed in rad/s.
     * @param wz angular speed in z axis (yaw axis). Expressed in rad/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new updated quaternion.
     */
    public static Quaternion predictWithRotationAdjustment(Quaternion q, 
            Quaternion dq, double wx, double wy, double wz, double dt) {
        Quaternion result = new Quaternion();
        predictWithRotationAdjustment(q, dq, wx, wy, wz, dt, result);
        return result;
    }
    
    /**
     * Predicts the updated quaternion after a rotation in body frame expressed
     * by provided rotation dq and by provided rate of rotation along axes x,y,z
     * (roll, pitch, yaw).
     * @param q quaternion to be updated.
     * @param dq adjustment of rotation to be combined with input quaternion.
     * @param w angular speed (x, y, z axes). Expressed in rad/s. Must have
     * length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new updated quaternion.
     * @throws IllegalArgumentException if w does not have length 3.
     */
    public static Quaternion predictWithRotationAdjustment(Quaternion q, 
            Quaternion dq, double[] w, double dt) 
            throws IllegalArgumentException {
        Quaternion result = new Quaternion();
        predictWithRotationAdjustment(q, dq, w, dt, result);
        return result;
    } 
}
