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
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;

/**
 * Utility class to predict position of device.
 */
@SuppressWarnings("WeakerAccess")
public class PositionPredictor {
    
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
    private PositionPredictor() { }
    
    /**
     * Predicts the updated position. 
     * @param r current position. Expressed in meters.
     * @param vx velocity in x-axis. Expressed in m/s.
     * @param vy velocity in y-axis. Expressed in m/s.
     * @param vz velocity in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated position is stored.
     * @param jacobianR jacobian wrt position. Must be 3x3.
     * @param jacobianV jacobian wrt speed. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(InhomogeneousPoint3D r, 
            double vx, double vy, double vz, double ax, double ay, double az,
            double dt, InhomogeneousPoint3D result, Matrix jacobianR, 
            Matrix jacobianV, Matrix jacobianA) 
            throws IllegalArgumentException {
        if(jacobianR != null && (jacobianR.getRows() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH ||
                jacobianR.getColumns() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)) {
            throw new IllegalArgumentException("jacobian wrt r must be 3x3");
        }
        if(jacobianV != null && (jacobianV.getRows() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH ||
                jacobianV.getColumns() != SPEED_COMPONENTS)) {
            throw new IllegalArgumentException(
                    "jacobian wrt speed must be 3x3");
        }
        if(jacobianA != null && (jacobianA.getRows() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH ||
                jacobianA.getColumns() != ACCELERATION_COMPONENTS)) {
            throw new IllegalArgumentException(
                    "jacobian wrt acceleration must be 3x3");
        }
        
        if (jacobianR != null) {
            //set to the identity
            jacobianR.initialize(0.0);
            jacobianR.setElementAt(0, 0, 1.0);
            jacobianR.setElementAt(1, 1, 1.0);
            jacobianR.setElementAt(2, 2, 1.0);
        }
        if (jacobianV != null) {
            //identity multiplied by dt
            jacobianV.initialize(0.0);
            jacobianV.setElementAt(0, 0, dt);
            jacobianV.setElementAt(1, 1, dt);
            jacobianV.setElementAt(2, 2, dt);
        }
        
        
        if(ax == 0.0 && ay == 0.0 && az == 0.0) {
            result.setCoordinates(
                    r.getInhomX() + vx * dt, 
                    r.getInhomY() + vy * dt, 
                    r.getInhomZ() + vz * dt);
            
            if(jacobianA != null) {
                jacobianA.initialize(0.0);
            }
        } else {
            result.setCoordinates(
                    r.getInhomX() + vx * dt + 0.5 * ax * dt * dt, 
                    r.getInhomY() + vy * dt + 0.5 * ay * dt * dt, 
                    r.getInhomZ() + vz * dt + 0.5 * az * dt * dt);
            
            if(jacobianA != null) {
                //identity multiplied by 0.5*dt^2
                double value = 0.5 * dt * dt;
                jacobianA.initialize(0.0);
                jacobianA.setElementAt(0, 0, value);
                jacobianA.setElementAt(1, 1, value);
                jacobianA.setElementAt(2, 2, value);
            }
        }
    }
    
    /**
     * Predicts the updated position.
     * @param r current position. Expresed in meters.
     * @param vx velocity in x-axis. Expressed in m/s.
     * @param vy velocity in y-axis. Expressed in m/s.
     * @param vz velocity in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated position is stored.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(InhomogeneousPoint3D r, 
            double vx, double vy, double vz, double ax, double ay, double az,
            double dt, InhomogeneousPoint3D result) {
        predict(r, vx, vy, vz, ax, ay, az, dt, result, null, null, null);
    }
    
    /**
     * Predicts the updated position.
     * @param r point containing current position in 3D. Expressed in meters.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param a array containing 3 components of acceleration. Expressed in 
     * m/s^2. Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated position is stored.
     * @param jacobianR jacobian wrt position. Must be 3x3.
     * @param jacobianV jacobian wrt speed. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size of velocity or acceleration do not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(InhomogeneousPoint3D r, double[] v, 
            double[] a, double dt, InhomogeneousPoint3D result, 
            Matrix jacobianR, Matrix jacobianV, Matrix jacobianA) 
            throws IllegalArgumentException {
        if(v.length != SPEED_COMPONENTS) {
            throw new IllegalArgumentException("v must have length 3");
        }
        if(a.length != ACCELERATION_COMPONENTS) {
            throw new IllegalArgumentException("a must have length 3");
        }
        predict(r, v[0], v[1], v[2], a[0], a[1], a[2], dt, result,
                jacobianR, jacobianV, jacobianA);
    }
    
    /**
     * Predicts the updated position.
     * @param r point containing current position in 3D. Expressed in meters.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param a array containing 3 components of acceleration. Expressed in 
     * m/s^2. Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated position is stored.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(InhomogeneousPoint3D r, double[] v, 
            double[] a, double dt, InhomogeneousPoint3D result) {
        predict(r, v, a, dt, result, null, null, null);
    }
    
    /**
     * Predicts the updated position assuming no acceleration.
     * @param r current position. Expresed in meters.
     * @param vx velocity in x-axis. Expressed in m/s.
     * @param vy velocity in y-axis. Expressed in m/s.
     * @param vz velocity in z-axis. Expressed in m/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated position is stored.
     * @param jacobianR jacobian wrt position. Must be 3x3.
     * @param jacobianV jacobian wrt speed. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(InhomogeneousPoint3D r,
            double vx, double vy, double vz, double dt, 
            InhomogeneousPoint3D result, Matrix jacobianR, Matrix jacobianV, 
            Matrix jacobianA) throws IllegalArgumentException {
        predict(r, vx, vy, vz, 0.0, 0.0, 0.0, dt, result, jacobianR, jacobianV, 
                jacobianA);
    }
    
    /**
     * Predicts the updated position assuming no acceleration.
     * @param r current position. Expresed in meters.
     * @param vx velocity in x-axis. Expressed in m/s.
     * @param vy velocity in y-axis. Expressed in m/s.
     * @param vz velocity in z-axis. Expressed in m/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated position is stored.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(InhomogeneousPoint3D r, 
            double vx, double vy, double vz, double dt, 
            InhomogeneousPoint3D result) {
        predict(r, vx, vy, vz, dt, result, null, null, null);
    }
    
    /**
     * Predicts the updated position assuming no acceleration.
     * @param r point containing current position in 3D. Expressed in meters.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated position is stored.
     * @param jacobianR jacobian wrt position. Must be 3x3.
     * @param jacobianV jacobian wrt speed. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size of velocity does not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(InhomogeneousPoint3D r,
            double[] v, double dt, InhomogeneousPoint3D result, 
            Matrix jacobianR, Matrix jacobianV, Matrix jacobianA) 
            throws IllegalArgumentException {
        if(v.length != SPEED_COMPONENTS) {
            throw new IllegalArgumentException("v must have length 3");
        }
        predict(r, v[0], v[1], v[2], dt, result, jacobianR, jacobianV, 
                jacobianA);
    }
    
    /**
     * Predicts the updated position assuming no acceleration.
     * @param r point containing current position in 3D. Expressed in meters.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated position is stored.
     * @throws IllegalArgumentException if velocity array does not have length 
     * 3.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(InhomogeneousPoint3D r,
            double[] v, double dt, InhomogeneousPoint3D result) 
            throws IllegalArgumentException {
        predict(r, v, dt, result, null, null, null);
    }
    
    /**
     * Predicts the updated position 
     * @param r current position. Expressed in meters.
     * @param vx velocity in x-axis. Expressed in m/s.
     * @param vy velocity in y-axis. Expressed in m/s.
     * @param vz velocity in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianR jacobian wrt position. Must be 3x3.
     * @param jacobianV jacobian wrt speed. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @return a new instance containing the updated position.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static InhomogeneousPoint3D predict(InhomogeneousPoint3D r, 
            double vx, double vy, double vz, double ax, double ay, double az, 
            double dt, Matrix jacobianR, Matrix jacobianV, Matrix jacobianA) 
            throws IllegalArgumentException {
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        predict(r, vx, vy, vz, ax, ay, az, dt, result, jacobianR, jacobianV, 
                jacobianA);
        return result;
    }
    
    /**
     * Predicts the updated position.
     * @param r current position. Expresed in meters.
     * @param vx velocity in x-axis. Expressed in m/s.
     * @param vy velocity in y-axis. Expressed in m/s.
     * @param vz velocity in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new instance containing the updated position.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static InhomogeneousPoint3D predict(InhomogeneousPoint3D r, 
            double vx, double vy, double vz, double ax, double ay, double az, 
            double dt) {
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        predict(r, vx, vy, vz, ax, ay, az, dt, result);
        return result;
    }
    
    /**
     * Predicts the updated position.
     * @param r point containing current position in 3D. Expressed in meters.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param a array containing 3 components of acceleration. Expressed in 
     * m/s^2. Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianR jacobian wrt position. Must be 3x3.
     * @param jacobianV jacobian wrt speed. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @return a new instance containing the updated position.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static InhomogeneousPoint3D predict(InhomogeneousPoint3D r, 
            double[] v, double[] a, double dt, Matrix jacobianR, 
            Matrix jacobianV, Matrix jacobianA) 
            throws IllegalArgumentException {
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        predict(r, v, a, dt, result, jacobianR, jacobianV, jacobianA);
        return result;
    }
    
    /**
     * Predicts the updated position.
     * @param r point containing current position in 3D. Expressed in meters.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param a array containing 3 components of acceleration. Expressed in 
     * m/s^2. Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new instance containing the updated position.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static InhomogeneousPoint3D predict(InhomogeneousPoint3D r, 
            double[] v, double[] a, double dt) {
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        predict(r, v, a, dt, result);
        return result;
    }
    
    /**
     * Predicts the updated position assuming no acceleration.
     * @param r current position. Expresed in meters.
     * @param vx velocity in x-axis. Expressed in m/s.
     * @param vy velocity in y-axis. Expressed in m/s.
     * @param vz velocity in z-axis. Expressed in m/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianR jacobian wrt position. Must be 3x3.
     * @param jacobianV jacobian wrt speed. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @return a new instance containing the updated position.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static InhomogeneousPoint3D predict(InhomogeneousPoint3D r, 
            double vx, double vy, double vz, double dt, Matrix jacobianR, 
            Matrix jacobianV, Matrix jacobianA) 
            throws IllegalArgumentException {
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        predict(r, vx, vy, vz, dt, result, jacobianR, jacobianV, jacobianA);
        return result;
    }
    
    /**
     * Predicts the updated position assuming no acceleration.
     * @param r current position. Expresed in meters.
     * @param vx velocity in x-axis. Expressed in m/s.
     * @param vy velocity in y-axis. Expressed in m/s.
     * @param vz velocity in z-axis. Expressed in m/s.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new instance containing the updated position.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static InhomogeneousPoint3D predict(InhomogeneousPoint3D r, 
            double vx, double vy, double vz, double dt) {
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        predict(r, vx, vy, vz, dt, result);
        return result;
    }
    
    /**
     * Predicts the updated position assuming no acceleration.
     * @param r current position. Expresed in meters.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianR jacobian wrt position. Must be 3x3.
     * @param jacobianV jacobian wrt speed. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @return a new instance containing the updated position.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size of velocity does not have length 3.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static InhomogeneousPoint3D predict(InhomogeneousPoint3D r, 
            double[] v, double dt, Matrix jacobianR, Matrix jacobianV, 
            Matrix jacobianA) throws IllegalArgumentException {
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        predict(r, v, dt, result, jacobianR, jacobianV, jacobianA);
        return result;
    }
    
    /**
     * Predicts the updated position assuming no acceleration.
     * @param r current position. Expresed in meters.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new instance containing the updated position.
     * @throws IllegalArgumentException if size of v array is not 3.
     * @see <a href="https://github.com/joansola/slamtb">rpredict.m at https://github.com/joansola/slamtb</a>
     */
    public static InhomogeneousPoint3D predict(InhomogeneousPoint3D r,
            double[] v, double dt) throws IllegalArgumentException {
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        predict(r, v, dt, result);
        return result;
    }
       
    /**
     * Predicts the updated position by using provided position variation, 
     * current speed and current acceleration.
     * @param r current position. Expressed in meters.
     * @param drx adjustment of position in x-axis. Expressed in meters.
     * @param dry adjustment of position in y-axis. Expressed in meters.
     * @param drz adjustment of position in z-axis. Expressed in meters.
     * @param vx velocity in x-axis. Expressed in m/s.
     * @param vy velocity in y-axis. Expressed in m/s.
     * @param vz velocity in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated position is stored.
     * @param jacobianR jacobian wrt position. Must be 3x3.
     * @param jacobianDR jacobian wrt position adjustment. Must be 3x3.
     * @param jacobianV jacobian wrt speed. Must be 3c3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size.
     */
    public static void predictWithPositionAdjustment(InhomogeneousPoint3D r, 
            double drx, double dry, double drz, double vx, double vy, double vz,
            double ax, double ay, double az, double dt, 
            InhomogeneousPoint3D result, Matrix jacobianR, Matrix jacobianDR, 
            Matrix jacobianV, Matrix jacobianA) 
            throws IllegalArgumentException {
        if(jacobianR != null && (jacobianR.getRows() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH ||
                jacobianR.getColumns() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)) {
            throw new IllegalArgumentException("jacobian wrt r must be 3x3");
        }
        if(jacobianDR != null && (jacobianDR.getRows() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH ||
                jacobianDR.getColumns() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)) {
            throw new IllegalArgumentException("jacobian wrt dr must be 3x3");
        }
        if(jacobianV != null && (jacobianV.getRows() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH ||
                jacobianV.getColumns() != SPEED_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt speed must be 3x3");
        }
        if(jacobianA != null && (jacobianA.getRows() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH ||
                jacobianA.getColumns() != ACCELERATION_COMPONENTS)) {
            throw new IllegalArgumentException(
                    "jacobian wrt acceleration must be 3x3");
        }
        
        if(jacobianR != null) {
            //set to identity
            jacobianR.initialize(0.0);
            jacobianR.setElementAt(0, 0, 1.0);
            jacobianR.setElementAt(1, 1, 1.0);
            jacobianR.setElementAt(2, 2, 1.0);
        }
        if(jacobianDR != null) {
            //set to identity
            jacobianDR.initialize(0.0);
            jacobianDR.setElementAt(0, 0, 1.0);
            jacobianDR.setElementAt(1, 1, 1.0);
            jacobianDR.setElementAt(2, 2, 1.0);
        }
        if(jacobianV != null) {
            //identity multiplied by dt
            jacobianV.initialize(0.0);
            jacobianV.setElementAt(0, 0, dt);
            jacobianV.setElementAt(1, 1, dt);
            jacobianV.setElementAt(2, 2, dt);
        }
        
        
        if(ax == 0.0 && ay == 0.0 && az == 0.0) {
            result.setCoordinates(
                    r.getInhomX() + drx + vx * dt, 
                    r.getInhomY() + dry + vy * dt, 
                    r.getInhomZ() + drz + vz * dt);
            
            if(jacobianA != null) {
                jacobianA.initialize(0.0);
            }
        } else {
            result.setCoordinates(
                    r.getInhomX() + drx + vx * dt + 0.5 * ax * dt * dt, 
                    r.getInhomY() + dry + vy * dt + 0.5 * ay * dt * dt, 
                    r.getInhomZ() + drz + vz * dt + 0.5 * az * dt * dt);
            
            if(jacobianA != null) {
                //identity multiplied by 0.5*dt^2
                double value = 0.5 * dt * dt;
                jacobianA.initialize(0.0);
                jacobianA.setElementAt(0, 0, value);
                jacobianA.setElementAt(1, 1, value);
                jacobianA.setElementAt(2, 2, value);
            }
        }        
    }
    
    /**
     * Predicts the updated position by using provided position variation,
     * current speed and current acceleration.
     * @param r current position. Expressed in meters.
     * @param drx adjustment of position in x-axis. Expressed in meters.
     * @param dry adjustment of position in y-axis. Expressed in meters.
     * @param drz adjustment of position in z-axis. Expressed in meters.
     * @param vx velocity in x-axis. Expressed in m/s.
     * @param vy velocity in y-axis. Expressed in m/s.
     * @param vz velocity in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated position is stored.
     */
    public static void predictWithPositionAdjustment(InhomogeneousPoint3D r, 
            double drx, double dry, double drz, double vx, double vy, double vz, 
            double ax, double ay, double az, double dt, 
            InhomogeneousPoint3D result) {
        predictWithPositionAdjustment(r, drx, dry, drz, vx, vy, vz, ax, ay, az, 
                dt, result, null, null, null, null);
    }
    
    /**
     * Predicts the updated position by using provided position variation, 
     * current speed and current acceleration.
     * @param r current position. Expressed in meters.
     * @param dr adjustment of position (x, y, z). Must have length 3.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param a array containing 3 components of acceleration. Expressed in 
     * m/s^2. Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated position is stored.
     * @param jacobianR jacobian wrt position. Must be 3x3.
     * @param jacobianDR jacobian wrt position adjustment. Must be 3x3.
     * @param jacobianV jacobian wrt speed. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size or dr, v or a do not have length 3.
     */
    public static void predictWithPositionAdjustment(InhomogeneousPoint3D r, 
            double[] dr, double[] v, double[] a, double dt, 
            InhomogeneousPoint3D result, Matrix jacobianR, Matrix jacobianDR, 
            Matrix jacobianV, Matrix jacobianA) 
            throws IllegalArgumentException {
        if(dr.length != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException("dr must have length 3");
        }
        if(v.length != SPEED_COMPONENTS) {
            throw new IllegalArgumentException("v must have length 3");
        }
        if(a.length != ACCELERATION_COMPONENTS) {
            throw new IllegalArgumentException("a must have length 3");
        }
        predictWithPositionAdjustment(r, dr[0], dr[1], dr[2], 
                v[0], v[1], v[2], a[0], a[1], a[2], dt, result, jacobianR, 
                jacobianDR, jacobianV, jacobianA);
    }
    
    /**
     * Predicts the updated position.
     * @param r point containing current position in 3D. Expressed in meters.
     * @param dr adjustment of position (x, y, z). Must have length 3.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param a array containing 3 components of acceleration. Expressed in 
     * m/s^2. Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated position is stored.
     * @throws IllegalArgumentException if dr, v or a do not have length 3.
     */
    public static void predictWithPositionAdjustment(InhomogeneousPoint3D r, 
            double[] dr, double[] v, double[] a, double dt, 
            InhomogeneousPoint3D result) throws IllegalArgumentException {
        predictWithPositionAdjustment(r, dr, v, a, dt, result, null, null, null,
                null);
    }
    
    /**
     * Predicts the updated position by using provided position variation, 
     * current speed and current acceleration.
     * @param r current position. Expressed in meters.
     * @param drx adjustment of position in x-axis. Expressed in meters.
     * @param dry adjustment of position in y-axis. Expressed in meters.
     * @param drz adjustment of position in z-axis. Expressed in meters.
     * @param vx velocity in x-axis. Expressed in m/s.
     * @param vy velocity in y-axis. Expressed in m/s.
     * @param vz velocity in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianR jacobian wrt position. Must be 3x3.
     * @param jacobianDR jacobian wrt position adjustment. Must be 3x3.
     * @param jacobianV jacobian wrt speed. Must be 3c3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @return a new updated position.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size.
     */
    public static InhomogeneousPoint3D predictWithPositionAdjustment(
            InhomogeneousPoint3D r, double drx, double dry, double drz,
            double vx, double vy, double vz, double ax, double ay, double az,
            double dt, Matrix jacobianR, Matrix jacobianDR, Matrix jacobianV,
            Matrix jacobianA) throws IllegalArgumentException {
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        predictWithPositionAdjustment(r, drx, dry, drz, vx, vy, vz, ax, 
                ay, az, dt, result, jacobianR, jacobianDR, jacobianV, 
                jacobianA);
        return result;
    }
    
    /**
     * Predicts the updated position by using provided position variation,
     * current speed and current acceleration.
     * @param r current position. Expressed in meters.
     * @param drx adjustment of position in x-axis. Expressed in meters.
     * @param dry adjustment of position in y-axis. Expressed in meters.
     * @param drz adjustment of position in z-axis. Expressed in meters.
     * @param vx velocity in x-axis. Expressed in m/s.
     * @param vy velocity in y-axis. Expressed in m/s.
     * @param vz velocity in z-axis. Expressed in m/s.
     * @param ax acceleration in x-axis. Expressed in m/s^2.
     * @param ay acceleration in y-axis. Expressed in m/s^2.
     * @param az acceleration in z-axis. Expressed in m/s^2.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new updated position.
     */
    public static InhomogeneousPoint3D predictWithPositionAdjustment(
            InhomogeneousPoint3D r, double drx, double dry, double drz,
            double vx, double vy, double vz, double ax, double ay, double az,
            double dt) {
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        predictWithPositionAdjustment(r, drx, dry, drz, vx, vy, vz, 
                ax, ay, az, dt, result);
        return result;
    }
    
    /**
     * Predicts the updated position by using provided position variation, 
     * current speed and current acceleration.
     * @param r current position. Expressed in meters.
     * @param dr adjustment of position (x, y, z). Must have length 3.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param a array containing 3 components of acceleration. Expressed in 
     * m/s^2. Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianR jacobian wrt position. Must be 3x3.
     * @param jacobianDR jacobian wrt position adjustment. Must be 3x3.
     * @param jacobianV jacobian wrt speed. Must be 3x3.
     * @param jacobianA jacobian wrt acceleration. Must be 3x3.
     * @return a new updated position.
     * @throws IllegalArgumentException if any of provided jacobians does not 
     * have proper size or dr, v or a do not have length 3.
     */
    public static InhomogeneousPoint3D predictWithPositionAdjustment(
            InhomogeneousPoint3D r, double[] dr, double[] v, double[] a,
            double dt, Matrix jacobianR, Matrix jacobianDR, Matrix jacobianV,
            Matrix jacobianA) throws IllegalArgumentException {
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        predictWithPositionAdjustment(r, dr, v, a, dt, result, 
                jacobianR, jacobianDR, jacobianV, jacobianA);
        return result;
    }
    
    /**
     * Predicts the updated position.
     * @param r point containing current position in 3D. Expressed in meters.
     * @param dr adjustment of position (x, y, z). Must have length 3.
     * @param v array containing 3 components of velocity. Expressed in m/s. 
     * Must have length 3.
     * @param a array containing 3 components of acceleration. Expressed in 
     * m/s^2. Must have length 3.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new updated position.
     */
    public static InhomogeneousPoint3D predictWithPositionAdjustment(
            InhomogeneousPoint3D r, double[] dr, double[] v, double[] a, 
            double dt) {
        InhomogeneousPoint3D result = new InhomogeneousPoint3D();
        predictWithPositionAdjustment(r, dr, v, a, dt, result);
        return result;
    }        
}
