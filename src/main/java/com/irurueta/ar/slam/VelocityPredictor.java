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

import com.irurueta.algebra.Matrix;

/**
 * Utility class to predict velocity of device.
 */
@SuppressWarnings("WeakerAccess")
public class VelocityPredictor {
    
    /**
     * Number of components of speed.
     */
    public static final int SPEED_COMPONENTS = 3;
    
    /**
     * Number of components of acceleration.
     */
    public static final int ACCELERATION_COMPONENTS = 3;
    
    /**
     * Constructor.
     */
    private VelocityPredictor() { }
    
    /**
     * Predicts the updated velocity.
     * @param vx current velocity in x-axis. Expressed in m/s.
     * @param vy current velocity in y-axis. Expressed in m/s.
     * @param vz current velocity in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated velocity is stored. Must have 
     * length 3.
     * @param jacobianV jacobian wrt velocity. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size or if result does not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">vpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(double vx, double vy, double vz, 
            double ax, double ay, double az, double dt, double[] result, 
            Matrix jacobianV, Matrix jacobianA) 
            throws IllegalArgumentException {
        if(result.length != SPEED_COMPONENTS) {
            throw new IllegalArgumentException("result must have length 3");
        }
        if(jacobianV != null && (jacobianV.getRows() != SPEED_COMPONENTS ||
                jacobianV.getColumns() != SPEED_COMPONENTS)) {
            throw new IllegalArgumentException(
                    "jacobian wrt velocity must be 3x3");
        }
        if(jacobianA != null && (jacobianA.getRows() != SPEED_COMPONENTS ||
                jacobianA.getColumns() != ACCELERATION_COMPONENTS)) {
            throw new IllegalArgumentException(
                    "jacobian wrt acceleration must be 3x3");
        }
        
        if (jacobianV != null) {
            //set to the identity
            jacobianV.initialize(0.0);
            jacobianV.setElementAt(0, 0, 1.0);
            jacobianV.setElementAt(1, 1, 1.0);
            jacobianV.setElementAt(2, 2, 1.0);            
        }
        
        if (jacobianA != null) {
            jacobianA.initialize(0.0);
            jacobianA.setElementAt(0, 0, dt);
            jacobianA.setElementAt(1, 1, dt);
            jacobianA.setElementAt(2, 2, dt);
        }
        
        result[0] = vx + ax * dt;
        result[1] = vy + ay * dt;
        result[2] = vz + az * dt;
    }
    
    /**
     * Predicts the updated velocity.
     * @param vx current velocity in x-axis. Expressed in m/s.
     * @param vy current velocity in y-axis. Expressed in m/s.
     * @param vz current velocity in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated velocity is stored.
     * @throws IllegalArgumentException if result does not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">vpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(double vx, double vy, double vz,
            double ax, double ay, double az, double dt, double[] result)
            throws IllegalArgumentException {
        predict(vx, vy, vz, ax, ay, az, dt, result, null, null);
    }
    
    /**
     * Predicts the updated velocity.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param a array containing 3 components of acceleration. Expressed in 
     * m/s^2. Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated velocity is stored.
     * @param jacobianV jacobian wrt velocity. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size or if v, a or result do not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">vpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(double[] v, double[] a, double dt, 
            double[] result, Matrix jacobianV, Matrix jacobianA) 
            throws IllegalArgumentException {
        if(v.length != SPEED_COMPONENTS) {
            throw new IllegalArgumentException("v must have length 3");
        }
        if(a.length != ACCELERATION_COMPONENTS) {
            throw new IllegalArgumentException("a must have length 3");
        }
        predict(v[0], v[1], v[2], a[0], a[1], a[2], dt, result, jacobianV, 
                jacobianA);
    }
    
    /**
     * Predicts the updated velocity.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param a array containing 3 components of acceleration. Expressed in 
     * m/s^2. Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated velocity is stored.
     * @throws IllegalArgumentException if v, a or result do not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">vpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(double[] v, double[] a, double dt,
            double[] result) throws IllegalArgumentException {
        predict(v, a, dt, result, null, null);
    }
    
    /**
     * Predicts the updated velocity.
     * @param vx current velocity in x-axis. Expressed in m/s.
     * @param vy current velocity in y-axis. Expressed in m/s.
     * @param vz current velocity in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianV jacobian wrt velocity. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @return a new instance containing updated velocity.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">vpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static double[] predict(double vx, double vy, double vz,
            double ax, double ay, double az, double dt, Matrix jacobianV, 
            Matrix jacobianA) throws IllegalArgumentException {
        double[] result = new double[SPEED_COMPONENTS];
        predict(vx, vy, vz, ax, ay, az, dt, result, jacobianV, jacobianA);
        return result;
    }
    
    /**
     * Predicts the updated velocity.
     * @param vx current velocity in x-axis. Expressed in m/s.
     * @param vy current velocity in y-axis. Expressed in m/s.
     * @param vz current velocity in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new instance containing updated velocity.
     * @see <a href="https://github.com/joansola/slamtb">vpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static double[] predict(double vx, double vy, double vz,
            double ax, double ay, double az, double dt) {
        double[] result = new double[SPEED_COMPONENTS];
        predict(vx, vy, vz, ax, ay, az, dt, result);
        return result;
    }
    
    /**
     * Predicts the updated velocity.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param a array containing 3 components of acceleration. Expressed in 
     * m/s^2. Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianV jacobian wrt velocity. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @return a new instance containing updated velocity.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size or if v or a do not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">vpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static double[] predict(double[] v, double[] a, double dt,
            Matrix jacobianV, Matrix jacobianA) 
            throws IllegalArgumentException {
        double[] result = new double[SPEED_COMPONENTS];
        predict(v, a, dt, result, jacobianV, jacobianA);
        return result;
    }
    
    /**
     * Predicts the updated velocity.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param a array containing 3 components of acceleration. Expressed in 
     * m/s^2. Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new instance containing updated velocity.
     * @throws IllegalArgumentException if a or v do not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">vpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static double[] predict(double[] v, double[] a, double dt) 
            throws IllegalArgumentException {
        double[] result = new double[SPEED_COMPONENTS];
        predict(v, a, dt, result);
        return result;
    }        
    
    /**
     * Predicts the updated velocity.
     * @param vx current velocity in x-axis. Expressed in m/s.
     * @param vy current velocity in y-axis. Expressed in m/s.
     * @param vz current velocity in z-axis. Expressed in m/s.
     * @param dvx velocity adjustment in x-axis. Expressed in m/s.
     * @param dvy velocity adjustment in y-axis. Expressed in m/s.
     * @param dvz velocity adjustment in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated velocity is stored. Must have 
     * length 3.
     * @param jacobianV jacobian wrt velocity. Must be 3x3.
     * @param jacobianDV jacobian wrt velocity adjustment. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size or if result does not have length 3.
     */
    public static void predictWithVelocityAdjustment(double vx, 
            double vy, double vz, double dvx, double dvy, double dvz,
            double ax, double ay, double az, double dt, double[] result,
            Matrix jacobianV, Matrix jacobianDV, Matrix jacobianA)
            throws IllegalArgumentException {
        if(result.length != SPEED_COMPONENTS) {
            throw new IllegalArgumentException("result must have length 3");
        }
        if(jacobianV != null && (jacobianV.getRows() != SPEED_COMPONENTS ||
                jacobianV.getColumns() != SPEED_COMPONENTS)) {
            throw new IllegalArgumentException(
                    "jacobian wrt velocity must be 3x3");
        }
        if(jacobianDV != null && (jacobianDV.getRows() != SPEED_COMPONENTS ||
                jacobianDV.getColumns() != SPEED_COMPONENTS)) {
            throw new IllegalArgumentException(
                    "jacobian wrt velocity variation must be 3x3");
        }
        if(jacobianA != null && (jacobianA.getRows() != SPEED_COMPONENTS ||
                jacobianA.getColumns() != ACCELERATION_COMPONENTS)) {
            throw new IllegalArgumentException(
                    "jacobian wrt acceleration must be 3x3");
        }
        
        if (jacobianV != null) {
            //set to the identity
            jacobianV.initialize(0.0);
            jacobianV.setElementAt(0, 0, 1.0);
            jacobianV.setElementAt(1, 1, 1.0);
            jacobianV.setElementAt(2, 2, 1.0);
        }
        if (jacobianDV != null) {
            //set to the identity
            jacobianDV.initialize(0.0);
            jacobianDV.setElementAt(0, 0, 1.0);
            jacobianDV.setElementAt(1, 1, 1.0);
            jacobianDV.setElementAt(2, 2, 1.0);
        }
        if (jacobianA != null) {
            jacobianA.initialize(0.0);
            jacobianA.setElementAt(0, 0, dt);
            jacobianA.setElementAt(1, 1, dt);
            jacobianA.setElementAt(2, 2, dt);            
        }
        
        result[0] = vx + dvx + ax * dt;
        result[1] = vy + dvy + ay * dt;
        result[2] = vz + dvz + az * dt;
    }
    
    /**
     * Predicts the updated velocity.
     * @param vx current velocity in x-axis. Expressed in m/s.
     * @param vy current velocity in y-axis. Expressed in m/s.
     * @param vz current velocity in z-axis. Expressed in m/s.
     * @param dvx velocity adjustment in x-axis. Expressed in m/s.
     * @param dvy velocity adjustment in y-axis. Expressed in m/s.
     * @param dvz velocity adjustment in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated velocity is stored. Must have
     * length 3.
     * @throws IllegalArgumentException if result does not have length 3.
     */
    public static void predictWithVelocityAdjustment(double vx, 
            double vy, double vz, double dvx, double dvy, double dvz,
            double ax, double ay, double az, double dt, double[] result)
            throws IllegalArgumentException {
        predictWithVelocityAdjustment(vx, vy, vz, dvx, dvy, dvz, ax, ay,
                az, dt, result, null, null, null);
    }
    
    /**
     * Predicts the updated velocity.
     * @param v current velocity (in x,y,z axes). Expressed in m/s. Must have 
     * length 3.
     * @param dv velocity adjustment (in x,y,z axes). Expressed in m/s. Must 
     * have length 3.
     * @param a acceleration (in x,y,z axes). Expressed in m/s. Must have length 
     * 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated velocity is stored. Must have length
     * 3.
     * @param jacobianV jacobian wrt velocity. Must be 3x3.
     * @param jacobianDV jacobian wrt velocity adjustment. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size or if result, v, dv or a do not have length 3.
     */
    public static void predictWithVelocityAdjustment(double[] v,
            double[] dv, double[] a, double dt, double[] result, 
            Matrix jacobianV, Matrix jacobianDV, Matrix jacobianA)
            throws IllegalArgumentException {
        if(v.length != SPEED_COMPONENTS) {
            throw new IllegalArgumentException("v must have length 3");
        }
        if(dv.length != SPEED_COMPONENTS) {
            throw new IllegalArgumentException("dv must have length 3");
        }        
        if(a.length != ACCELERATION_COMPONENTS) {
            throw new IllegalArgumentException("a must have length 3");
        }        
        predictWithVelocityAdjustment(v[0], v[1], v[2], dv[0], dv[1], 
                dv[2], a[0], a[1], a[2], dt, result, jacobianV, jacobianDV, 
                jacobianA);
    }
    
    /**
     * Predicts the updated velocity.
     * @param v current velocity (in x,y,z axes). Expressed in m/s. Must have 
     * length 3.
     * @param dv velocity adjustment (in x,y,z axes). Expressed in m/s. Must 
     * have length 3.
     * @param a acceleration (in x,y,z axes). Expressed in m/s. Must have length 
     * 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated velocity is stored. Must have length
     * 3.
     * @throws IllegalArgumentException if v, dv, a or result do not have length 
     * 3.
     */
    public static void predictWithVelocityAdjustment(double[] v,
            double[] dv, double[] a, double dt, double[] result) 
            throws IllegalArgumentException {
        predictWithVelocityAdjustment(v, dv, a, dt, result, null, null, null);
    }
    
    /**
     * Predicts the updated velocity.
     * @param vx current velocity in x-axis. Expressed in m/s.
     * @param vy current velocity in y-axis. Expressed in m/s.
     * @param vz current velocity in z-axis. Expressed in m/s.
     * @param dvx velocity adjustment in x-axis. Expressed in m/s.
     * @param dvy velocity adjustment in y-axis. Expressed in m/s.
     * @param dvz velocity adjustment in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianV jacobian wrt velocity. Must be 3x3.
     * @param jacobianDV jacobian wrt velocity adjustment. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @return a new array containing updated velocity.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size.
     */
    public static double[] predictWithVelocityAdjustment(double vx, 
            double vy, double vz, double dvx, double dvy, double dvz, double ax,
            double ay, double az, double dt, Matrix jacobianV, 
            Matrix jacobianDV, Matrix jacobianA) 
            throws IllegalArgumentException {
        double[] result = new double[SPEED_COMPONENTS];
        predictWithVelocityAdjustment(vx, vy, vz, dvx, dvy, dvz, ax, ay, 
                az, dt, result, jacobianV, jacobianDV, jacobianA);
        return result;
    }
    
    /**
     * Predicts the updated velocity.
     * @param vx current velocity in x-axis. Expressed in m/s.
     * @param vy current velocity in y-axis. Expressed in m/s.
     * @param vz current velocity in z-axis. Expressed in m/s.
     * @param dvx velocity adjustment in x-axis. Expressed in m/s.
     * @param dvy velocity adjustment in y-axis. Expressed in m/s.
     * @param dvz velocity adjustment in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new array containing updated velocity.
     */
    public static double[] predictWithVelocityAdjustment(double vx, 
            double vy, double vz, double dvx, double dvy, double dvz, double ax,
            double ay, double az, double dt) {
        double[] result = new double[SPEED_COMPONENTS];
        predictWithVelocityAdjustment(vx, vy, vz, dvx, dvy, dvz, ax, ay,
                az, dt, result);
        return result;
    }
    
    /**
     * Predicts the updated velocity.
     * @param v current velocity (in x,y,z axes). Expressed in m/s. Must have 
     * length 3.
     * @param dv velocity adjustment (in x,y,z axes). Expressed in m/s. Must 
     * have length 3.
     * @param a acceleration (in x,y,z axes). Expressed in m/s. Must have length 
     * 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianV jacobian wrt velocity. Must be 3x3.
     * @param jacobianDV jacobian wrt velocity adjustment. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @return a new array containing updated velocity.
     * @throws IllegalArgumentException if any of provided jacobians does not
     * have proper size or if v, dv or a do not have length 3.
     */
    public static double[] predictWithVelocityAdjustment(double[] v, 
            double[] dv, double[] a, double dt, Matrix jacobianV, 
            Matrix jacobianDV, Matrix jacobianA) 
            throws IllegalArgumentException {
        double[] result = new double[SPEED_COMPONENTS];
        predictWithVelocityAdjustment(v, dv, a, dt, result, jacobianV,
                jacobianDV, jacobianA);
        return result;
    }
    
    /**
     * Predicts the updated velocity.
     * @param v current velocity (in x,y,z axes). Expressed in m/s. Must have 
     * length 3.
     * @param dv velocity adjustment (in x,y,z axes). Expressed in m/s. Must 
     * have length 3.
     * @param a acceleration (in x,y,z axes). Expressed in m/s. Must have length 
     * 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new array containing updated velocity.
     * @throws IllegalArgumentException if v, dv or a do not have length 3.
     */
    public static double[] predictWithVelocityAdjustment(double[] v, 
            double[] dv, double[] a, double dt) 
            throws IllegalArgumentException {
        double[] result = new double[SPEED_COMPONENTS];
        predictWithVelocityAdjustment(v, dv, a, dt, result);
        return result;        
    }
    
}
