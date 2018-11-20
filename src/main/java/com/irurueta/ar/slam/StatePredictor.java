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
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;

/**
 * Utility class to predict device state (position, orientation, linear 
 * velocity, linear acceleration and angular velocity).
 */
@SuppressWarnings("WeakerAccess")
public class StatePredictor {
    
    /**
     * Number of components on angular speed.
     */
    public static final int ANGULAR_SPEED_COMPONENTS = 3;
    
    /**
     * Number of components of speed.
     */
    public static final int SPEED_COMPONENTS = 3;
    
    /**
     * Number of components of acceleration.
     */
    public static final int ACCELERATION_COMPONENTS = 3;    
    
    /**
     * Number of components of constant acceleration model state.
     */
    public static final int STATE_COMPONENTS = 16;
    
    /**
     * Number of components of constant acceleration model control signal.
     */
    public static final int CONTROL_COMPONENTS = 9;
    
    /**
     * Number of components of constant acceleration model state with position 
     * adjustment.
     */
    public static final int STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS = 16;
    
    /**
     * Number of components of constant acceleration model with position 
     * adjustment control signal.
     */
    public static final int CONTROL_WITH_POSITION_ADJUSTMENT_COMPONENTS = 12;    
    
    /**
     * Number of components of constant acceleration model state with rotation 
     * adjustment.
     */
    public static final int STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS = 16;
    
    /**
     * Number of components of constant acceleration model with rotation 
     * adjustment control signal.
     */
    public static final int CONTROL_WITH_ROTATION_ADJUSTMENT_COMPONENTS = 13;
    
    /**
     * Number of components of constant acceleration model with position and 
     * rotation adjustment.
     */
    public static final int STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS = 16;
    
    /**
     * Number of components of constant acceleration model with position and 
     * rotation adjustment control signal.
     */
    public static final int CONTROL_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS = 16;
    
    /**
     * Constructor.
     */
    private StatePredictor() { }
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: linear-velocity-change-x,
     * linear-velocity-change-y, linear-velocity-change-z, 
     * linear-acceleration-change-x, linear-acceleration-change-y,
     * linear-acceleration-change-z, angular-velocity-change-x,
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 9.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must 
     * have length 16.
     * @param jacobianX jacobian wrt system state. Must be 16x16.
     * @param jacobianU jacobian wrt control. must be 16x9.
     * @throws IllegalArgumentException if system state array, control array,
     * result or jacobians do not have proper size.
     */
    public static void predict(double[] x,
            double[] u, double dt, double[] result, Matrix jacobianX, 
            Matrix jacobianU) throws IllegalArgumentException {
        if(x.length != STATE_COMPONENTS) {
            throw new IllegalArgumentException("x must have length 16");
        }
        if(u.length != CONTROL_COMPONENTS) {
            throw new IllegalArgumentException("u must have length 9");
        }
        if(result.length != STATE_COMPONENTS) {
            throw new IllegalArgumentException("result must have length 16");
        }
        if(jacobianX != null && 
                (jacobianX.getRows() != STATE_COMPONENTS ||
                jacobianX.getColumns() != STATE_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt x must be 16x16");
        }
        if(jacobianU != null && 
                (jacobianU.getRows() != STATE_COMPONENTS ||
                jacobianU.getColumns() != CONTROL_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt u must be 16x9");
        }
        
        //position
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x[0], x[1], x[2]);
        
        //orientation
        Quaternion q = new Quaternion(x[3], x[4], x[5], x[6]);
        
        //linear velocity
        double vx = x[7];
        double vy = x[8];
        double vz = x[9];
        
        //linear acceleration
        double ax = x[10];
        double ay = x[11];
        double az = x[12];
        
        //angular velocity
        double wx = x[13];
        double wy = x[14];
        double wz = x[15];
        
        //linear velocity change (control)
        double uvx = u[0];
        double uvy = u[1];
        double uvz = u[2];
        
        //linear acceleration change (control)
        double uax = u[3];
        double uay = u[4];
        double uaz = u[5];
        
        //angular velocity change (control)
        double uwx = u[6];
        double uwy = u[7];
        double uwz = u[8];
        
        try {
            //update velocity
            Matrix Vv = new Matrix(SPEED_COMPONENTS, SPEED_COMPONENTS);
            Matrix Va = new Matrix(SPEED_COMPONENTS, ACCELERATION_COMPONENTS);
            double[] v = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, 
                    Vv, Va);
            
            //update position
            Matrix Rr = null, Rv = null, Ra = null;
            if(jacobianX != null) {
                Rr = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
                Rv = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 
                        SPEED_COMPONENTS);
                Ra = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                        ACCELERATION_COMPONENTS);
            }
            PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt, r, Rr, Rv, 
                    Ra);
            
            //update orientation
            Matrix Qq = null, Qw = null;
            if(jacobianX != null) {
                Qq = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
                Qw = new Matrix(Quaternion.N_PARAMS, ANGULAR_SPEED_COMPONENTS);
            }
            q = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, Qq, Qw);
            
            //set updated linear velocity
            vx = v[0];
            vy = v[1];
            vz = v[2];
            
            //apply control signals
            vx += uvx;
            vy += uvy;
            vz += uvz;
            
            ax += uax;
            ay += uay;
            az += uaz;
            
            wx += uwx;
            wy += uwy;
            wz += uwz;
            
            //set new state
            result[0] = r.getInhomX();
            result[1] = r.getInhomY();
            result[2] = r.getInhomZ();
            
            result[3] = q.getA();
            result[4] = q.getB();
            result[5] = q.getC();
            result[6] = q.getD();
            
            result[7] = vx;
            result[8] = vy;
            result[9] = vz;
            
            result[10] = ax;
            result[11] = ay;
            result[12] = az;
            
            result[13] = wx;
            result[14] = wy;
            result[15] = wz;
            
            //jacobians
            if (jacobianX != null) {
                //[Rr   0   Rv  Ra  0]
                //[0    Qq  0   0   Qw]
                //[0    0   Vv  Va  0]
                //[0    0   0   eye 0]
                //[0    0   0   0   eye]
                jacobianX.initialize(0.0);
                jacobianX.setSubmatrix(0, 0, 2, 2, Rr);
               
                jacobianX.setSubmatrix(3, 3, 6, 6, Qq);
               
                jacobianX.setSubmatrix(0, 7, 2, 9, Rv);
                
                jacobianX.setSubmatrix(7, 7, 9, 9, Vv);
                
                jacobianX.setSubmatrix(0, 10, 2, 12, Ra);
                
                jacobianX.setSubmatrix(7, 10, 9, 12, Va);
                
                jacobianX.setSubmatrix(3, 13, 6, 15, Qw);
               
                for(int i = 10; i < STATE_COMPONENTS; i++) {
                    jacobianX.setElementAt(i, i, 1.0);
                }               
            }
            
            if (jacobianU != null) {
                jacobianU.initialize(0.0);
                
                for(int i = 7, j = 0; i < STATE_COMPONENTS; i++, j++) {
                    jacobianU.setElementAt(i, j, 1.0);
                }
            }
        }catch(WrongSizeException ignore) { /* never thrown */ }        
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: linear-velocity-change-x,
     * linear-velocity-change-y, linear-velocity-change-z, 
     * linear-acceleration-change-x, linear-acceleration-change-y,
     * linear-acceleration-change-z, angular-velocity-change-x,
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 9.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must 
     * have length 16.
     * @throws IllegalArgumentException if system state array, control array
     * or result do not have proper size.
     */
    public static void predict(double[] x, 
            double[] u, double dt, double[] result) 
            throws IllegalArgumentException {
        predict(x, u, dt, result, null, null);
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: linear-velocity-change-x,
     * linear-velocity-change-y, linear-velocity-change-z, 
     * linear-acceleration-change-x, linear-acceleration-change-y,
     * linear-acceleration-change-z, angular-velocity-change-x,
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 9.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianX jacobian wrt system state. Must be 16x16.
     * @param jacobianU jacobian wrt control. must be 16x9.
     * @return a new instance containing the updated system state.
     * @throws IllegalArgumentException if system state array, control array or
     * jacobians do not have proper size.
     */
    public static double[] predict(double[] x,
            double[] u, double dt, Matrix jacobianX, Matrix jacobianU)
            throws IllegalArgumentException {
        double[] result = 
                new double[STATE_COMPONENTS];
        predict(x, u, dt, result, jacobianX, jacobianU);
        return result;
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: linear-velocity-change-x,
     * linear-velocity-change-y, linear-velocity-change-z, 
     * linear-acceleration-change-x, linear-acceleration-change-y,
     * linear-acceleration-change-z, angular-velocity-change-x,
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 9.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new instance containing the updated system state.
     * @throws IllegalArgumentException if system state array or control array
     * do not have proper size.
     */
    public static double[] predict(double[] x,
            double[] u, double dt) throws IllegalArgumentException {
        double[] result = 
                new double[STATE_COMPONENTS];
        predict(x, u, dt, result);
        return result;
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: position-change-x, 
     * position-change-y, position-change-z, linear-velocity-change-x,
     * linear-velocity-change-y, linear-velocity-change-z, 
     * linear-acceleration-change-x, linear-acceleration-change-y,
     * linear-acceleration-change-z, angular-velocity-change-x,
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 
     * 12.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must 
     * have length 16.
     * @param jacobianX jacobian wrt system state. Must be 16x16.
     * @param jacobianU jacobian wrt control. must be 16x12.
     * @throws IllegalArgumentException if system state array, control array,
     * result array or jacobians do not have proper size.
     */
    public static void predictWithPositionAdjustment(double[] x, 
            double[] u, double dt, double[] result, Matrix jacobianX, 
            Matrix jacobianU) throws IllegalArgumentException {
        if(x.length != STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("x must have length 16");
        }
        if(u.length != CONTROL_WITH_POSITION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("u must have length 12");
        }
        if(result.length != STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("result must have length 16");
        }
        if(jacobianX != null && 
                (jacobianX.getRows() != STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS ||
                jacobianX.getColumns() != STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt x must be 16x16");
        }
        if(jacobianU != null && 
                (jacobianU.getRows() != STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS ||
                jacobianU.getColumns() != CONTROL_WITH_POSITION_ADJUSTMENT_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt u must be 16x12");
        }
        
        //position
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x[0], x[1], x[2]);
        
        //orientation
        Quaternion q = new Quaternion(x[3], x[4], x[5], x[6]);
        
        //linear velocity
        double vx = x[7];
        double vy = x[8];
        double vz = x[9];
        
        //linear acceleration
        double ax = x[10];
        double ay = x[11];
        double az = x[12];
        
        //angular velocity
        double wx = x[13];
        double wy = x[14];
        double wz = x[15];
        
        //position change (control)
        double drx = u[0];
        double dry = u[1];
        double drz = u[2];
        
        //linear velocity change (control)
        double uvx = u[3];
        double uvy = u[4];
        double uvz = u[5];
        
        //linear acceleration change (control)
        double uax = u[6];
        double uay = u[7];
        double uaz = u[8];
        
        //angular velocity change (control)
        double uwx = u[9];
        double uwy = u[10];
        double uwz = u[11];
        
        try {
            //update velocity
            Matrix Vv = new Matrix(SPEED_COMPONENTS, SPEED_COMPONENTS);
            Matrix Va = new Matrix(SPEED_COMPONENTS, ACCELERATION_COMPONENTS);
            double[] v = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, 
                    Vv, Va);
            
            //update position
            Matrix Rr = null, Rv = null, Ra = null;
            if(jacobianX != null) {
                Rr = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
                Rv = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 
                        SPEED_COMPONENTS);
                Ra = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                        ACCELERATION_COMPONENTS);
            }
            PositionPredictor.predictWithPositionAdjustment(r, drx, dry, drz, 
                    vx, vy, vz, ax, ay, az, dt, r, Rr, null, Rv, Ra);
            
            //update orientation
            Matrix Qq = null, Qw = null;
            if(jacobianX != null) {
                Qq = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
                Qw = new Matrix(Quaternion.N_PARAMS, ANGULAR_SPEED_COMPONENTS);
            }
            q = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, Qq, Qw);
            
            //set updated linear velocity
            vx = v[0];
            vy = v[1];
            vz = v[2];
            
            //apply control signals
            vx += uvx;
            vy += uvy;
            vz += uvz;
            
            ax += uax;
            ay += uay;
            az += uaz;
            
            wx += uwx;
            wy += uwy;
            wz += uwz;
            
            //set new state
            result[0] = r.getInhomX();
            result[1] = r.getInhomY();
            result[2] = r.getInhomZ();
            
            result[3] = q.getA();
            result[4] = q.getB();
            result[5] = q.getC();
            result[6] = q.getD();
            
            result[7] = vx;
            result[8] = vy;
            result[9] = vz;
            
            result[10] = ax;
            result[11] = ay;
            result[12] = az;
            
            result[13] = wx;
            result[14] = wy;
            result[15] = wz;
            
            //jacobians
            if (jacobianX != null) {
                //[Rr   0   Rv  Ra  0]
                //[0    Qq  0   0   Qw]
                //[0    0   Vv  Va  0]
                //[0    0   0   eye 0]
                //[0    0   0   0   eye]
                jacobianX.initialize(0.0);
                jacobianX.setSubmatrix(0, 0, 2, 2, Rr);
               
                jacobianX.setSubmatrix(3, 3, 6, 6, Qq);
               
                jacobianX.setSubmatrix(0, 7, 2, 9, Rv);
                
                jacobianX.setSubmatrix(7, 7, 9, 9, Vv);
                
                jacobianX.setSubmatrix(0, 10, 2, 12, Ra);
                
                jacobianX.setSubmatrix(7, 10, 9, 12, Va);
                
                jacobianX.setSubmatrix(3, 13, 6, 15, Qw);
               
                for(int i = 10; i < STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS; i++) {
                    jacobianX.setElementAt(i, i, 1.0);
                }               
            }
            
            if (jacobianU != null) {
                jacobianU.initialize(0.0);
                
                //variation of position
                for(int i = 0; i < Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
                    jacobianU.setElementAt(i, i, 1.0);
                }
                //variation of linear speed and acceleration, and angular speed
                for(int i = 7, j = Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH; i < STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS; i++, j++) {
                    jacobianU.setElementAt(i, j, 1.0);
                }
            }
        }catch(WrongSizeException ignore) { /* never thrown */ }        
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: position-change-x, 
     * position-change-y, position-change-z, linear-velocity-change-x,
     * linear-velocity-change-y, linear-velocity-change-z, 
     * linear-acceleration-change-x, linear-acceleration-change-y,
     * linear-acceleration-change-z, angular-velocity-change-x,
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 
     * 12.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must 
     * have length 16.
     * @throws IllegalArgumentException if system state array, control array or
     * result array do not have proper size.
     */
    public static void predictWithPositionAdjustment(double[] x, 
            double[] u, double dt, double[] result) 
            throws IllegalArgumentException {
        predictWithPositionAdjustment(x, u, dt, result, null, null);
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: position-change-x, 
     * position-change-y, position-change-z, linear-velocity-change-x,
     * linear-velocity-change-y, linear-velocity-change-z, 
     * linear-acceleration-change-x, linear-acceleration-change-y,
     * linear-acceleration-change-z, angular-velocity-change-x,
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 
     * 12.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianX jacobian wrt system state. Must be 16x16.
     * @param jacobianU jacobian wrt control. must be 16x12.
     * @return a new array containing updated system model.
     * @throws IllegalArgumentException if system state array, control array
     * or jacobians do not have proper size.
     */
    public static double[] predictWithPositionAdjustment(double[] x,
                double[] u, double dt, Matrix jacobianX, 
                Matrix jacobianU) throws IllegalArgumentException {
        double[] result = new double[STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS];
        predictWithPositionAdjustment(x, u, dt, result, jacobianX, jacobianU);
        return result;
    }
        
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: position-change-x, 
     * position-change-y, position-change-z, linear-velocity-change-x,
     * linear-velocity-change-y, linear-velocity-change-z, 
     * linear-acceleration-change-x, linear-acceleration-change-y,
     * linear-acceleration-change-z, angular-velocity-change-x,
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 
     * 12.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new array containing updated system model.
     * @throws IllegalArgumentException if system state array or control array
     * do not have proper size.
     */
    public static double[] predictWithPositionAdjustment(double[] x,
                double[] u, double dt) throws IllegalArgumentException {
        double[] result = new double[STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS];
        predictWithPositionAdjustment(x, u, dt, result);
        return result;            
    } 
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: quaternion-change-a, 
     * quaternion-change-b, quaternion-change-c, quaternion-change-d, 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, linear-acceleration-change-x, 
     * linear-acceleration-change-y, linear-acceleration-change-z, 
     * angular-velocity-change-x, angular-velocity-change-y, 
     * angular-velocity-change-z. Must have length 13.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must 
     * have length 16.
     * @param jacobianX jacobian wrt system state. Must be 16x16.
     * @param jacobianU jacobian wrt control. must be 16x13.
     * @throws IllegalArgumentException if system state array, control array,
     * result array or jacobians do not have proper size.
     */
    public static void predictWithRotationAdjustment(double[] x, 
            double[] u, double dt, double[] result, Matrix jacobianX, 
            Matrix jacobianU) throws IllegalArgumentException {
        if(x.length != STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("x must have length 16");
        }
        if(u.length != CONTROL_WITH_ROTATION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("u must have length 13");
        }
        if(result.length != STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("result must have length 16");
        }
        if(jacobianX != null && 
                (jacobianX.getRows() != STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS ||
                jacobianX.getColumns() != STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt x must be 16x16");
        }
        if(jacobianU != null && 
                (jacobianU.getRows() != STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS ||
                jacobianU.getColumns() != CONTROL_WITH_ROTATION_ADJUSTMENT_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt u must be 16x13");
        }
        
        //position
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x[0], x[1], x[2]);
        
        //orientation
        Quaternion q = new Quaternion(x[3], x[4], x[5], x[6]);
        
        //linear velocity
        double vx = x[7];
        double vy = x[8];
        double vz = x[9];
        
        //linear accelertion
        double ax = x[10];
        double ay = x[11];
        double az = x[12];
        
        //angular velocity
        double wx = x[13];
        double wy = x[14];
        double wz = x[15];
        
        //rotation change (control)
        Quaternion dq = new Quaternion(u[0], u[1], u[2], u[3]);        
        
        //linear velocity change (control)
        double uvx = u[4];
        double uvy = u[5];
        double uvz = u[6];
        
        //linear acceleration change (control)
        double uax = u[7];
        double uay = u[8];
        double uaz = u[9];
        
        //angular velocity change (control)
        double uwx = u[10];
        double uwy = u[11];
        double uwz = u[12];
        
        try {
            //update velocity
            Matrix Vv = new Matrix(SPEED_COMPONENTS, SPEED_COMPONENTS);
            Matrix Va = new Matrix(SPEED_COMPONENTS, ACCELERATION_COMPONENTS);
            double[] v = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, 
                    Vv, Va);
            
            //update position
            Matrix Rr = null, Rv = null, Ra = null;
            if(jacobianX != null) {
                Rr = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
                Rv = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 
                        SPEED_COMPONENTS);
                Ra = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                        ACCELERATION_COMPONENTS);
            }
            PositionPredictor.predict(r, vx, vy, vz, ax, ay, az, dt, r, Rr, Rv, 
                    Ra);
            
            //update orientation
            Matrix Qq = null, Qdq = null, Qw = null;
            if(jacobianX != null) {
                Qq = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
                Qw = new Matrix(Quaternion.N_PARAMS, ANGULAR_SPEED_COMPONENTS);
            }
            if(jacobianU != null) {
                Qdq = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            }
            q = QuaternionPredictor.predictWithRotationAdjustment(q, dq, 
                    wx, wy, wz, dt, Qq, Qdq, Qw);

            
            //set updated linear velocity
            vx = v[0];
            vy = v[1];
            vz = v[2];
            
            //apply control signals
            vx += uvx;
            vy += uvy;
            vz += uvz;
            
            ax += uax;
            ay += uay;
            az += uaz;
            
            wx += uwx;
            wy += uwy;
            wz += uwz;
            
            //set new state
            result[0] = r.getInhomX();
            result[1] = r.getInhomY();
            result[2] = r.getInhomZ();
            
            result[3] = q.getA();
            result[4] = q.getB();
            result[5] = q.getC();
            result[6] = q.getD();
            
            result[7] = vx;
            result[8] = vy;
            result[9] = vz;
            
            result[10] = ax;
            result[11] = ay;
            result[12] = az;
            
            result[13] = wx;
            result[14] = wy;
            result[15] = wz;
            
            //jacobians
            if (jacobianX != null) {
                //[Rr   0   Rv  Ra  0]
                //[0    Qq  0   0   Qw]
                //[0    0   Vv  Va  0]
                //[0    0   0   eye 0]
                //[0    0   0   0   eye]
                jacobianX.initialize(0.0);
                jacobianX.setSubmatrix(0, 0, 2, 2, Rr);
               
                jacobianX.setSubmatrix(3, 3, 6, 6, Qq);
               
                jacobianX.setSubmatrix(0, 7, 2, 9, Rv);
                
                jacobianX.setSubmatrix(7, 7, 9, 9, Vv);
                
                jacobianX.setSubmatrix(0, 10, 2, 12, Ra);
                
                jacobianX.setSubmatrix(7, 10, 9, 12, Va);
                
                jacobianX.setSubmatrix(3, 13, 6, 15, Qw);
               
                for(int i = 10; i < STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS; i++) {
                    jacobianX.setElementAt(i, i, 1.0);
                }               
            }
            
            if (jacobianU != null) {
                jacobianU.initialize(0.0);
                
                //variation of rotation
                jacobianU.setSubmatrix(3, 0, 6, 3, Qdq);

                //variation of linear speed and acceleration, and angular speed
                for(int i = 7, j = Quaternion.N_PARAMS; i < STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS; i++, j++) {
                    jacobianU.setElementAt(i, j, 1.0);
                }
            }
        }catch(WrongSizeException ignore) { /* never thrown */ }                
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: quaternion-change-a, 
     * quaternion-change-b, quaternion-change-c, quaternion-change-d, 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, linear-acceleration-change-x, 
     * linear-acceleration-change-y, linear-acceleration-change-z, 
     * angular-velocity-change-x, angular-velocity-change-y, 
     * angular-velocity-change-z. Must have length 13.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must 
     * have length 16.
     * @throws IllegalArgumentException if system state array, control array or
     * result array do not have proper size.
     */
    public static void predictWithRotationAdjustment(double[] x, 
            double[] u, double dt, double[] result) 
            throws IllegalArgumentException {
        predictWithRotationAdjustment(x, u, dt, result, null, null);
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: quaternion-change-a, 
     * quaternion-change-b, quaternion-change-c, quaternion-change-d, 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, linear-acceleration-change-x, 
     * linear-acceleration-change-y, linear-acceleration-change-z, 
     * angular-velocity-change-x, angular-velocity-change-y, 
     * angular-velocity-change-z. Must have length 13.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianX jacobian wrt system state. Must be 16x16.
     * @param jacobianU jacobian wrt control. must be 16x13.
     * @return a new array containing updated system model.
     * @throws IllegalArgumentException if system state array, control array or
     * jacobians do not have proper size.
     */
    public static double[] predictWithRotationAdjustment(double[] x, 
            double[] u, double dt, Matrix jacobianX, Matrix jacobianU) 
            throws IllegalArgumentException {
        double[] result = new double[STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS];
        predictWithRotationAdjustment(x, u, dt, result, jacobianX, jacobianU);
        return result;
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: quaternion-change-a, 
     * quaternion-change-b, quaternion-change-c, quaternion-change-d, 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, linear-acceleration-change-x, 
     * linear-acceleration-change-y, linear-acceleration-change-z, 
     * angular-velocity-change-x, angular-velocity-change-y, 
     * angular-velocity-change-z. Must have length 13.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new array containing updated system model.
     * @throws IllegalArgumentException if system state array or control array
     * do not have proper size.
     */
    public static double[] predictWithRotationAdjustment(double[] x, 
            double[] u, double dt) throws IllegalArgumentException {
        double[] result = new double[STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS];
        predictWithRotationAdjustment(x, u, dt, result);
        return result;
    }    
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: position-change-x,
     * position-change-y, position-change-z, quaternion-change-a, 
     * quaternion-change-b, quaternion-change-c, quaternion-change-d, 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, linear-acceleration-change-x, 
     * linear-acceleration-change-y, linear-acceleration-change-z, 
     * angular-velocity-change-x, angular-velocity-change-y, 
     * angular-velocity-change-z. Must have length 16.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must 
     * have length 16.
     * @param jacobianX jacobian wrt system state. Must be 16x16.
     * @param jacobianU jacobian wrt control. must be 16x16.
     * @throws IllegalArgumentException if system state array, control array,
     * result array or jacobians do not have proper size.
     */
    public static void predictWithPositionAndRotationAdjustment(
            double[] x, double[] u, double dt, double[] result, 
            Matrix jacobianX, Matrix jacobianU) 
            throws IllegalArgumentException {
        if(x.length != STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("x must have length 16");
        }
        if(u.length != CONTROL_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("u must have length 13");
        }
        if(result.length != STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("result must have length 16");
        }
        if(jacobianX != null && 
                (jacobianX.getRows() != STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS ||
                jacobianX.getColumns() != STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt x must be 16x16");
        }
        if(jacobianU != null && 
                (jacobianU.getRows() != STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS ||
                jacobianU.getColumns() != CONTROL_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt u must be 16x13");
        }
        
        //position
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x[0], x[1], x[2]);
        
        //orientation
        Quaternion q = new Quaternion(x[3], x[4], x[5], x[6]);
        
        //linear velocity
        double vx = x[7];
        double vy = x[8];
        double vz = x[9];
        
        //linear accelertion
        double ax = x[10];
        double ay = x[11];
        double az = x[12];
        
        //angular velocity
        double wx = x[13];
        double wy = x[14];
        double wz = x[15];
        
        //position change (control)
        double drx = u[0];
        double dry = u[1];
        double drz = u[2];        
        
        //rotation change (control)
        Quaternion dq = new Quaternion(u[3], u[4], u[5], u[6]);        
        
        //linear velocity change (control)
        double uvx = u[7];
        double uvy = u[8];
        double uvz = u[9];
        
        //linear acceleration change (control)
        double uax = u[10];
        double uay = u[11];
        double uaz = u[12];
        
        //angular velocity change (control)
        double uwx = u[13];
        double uwy = u[14];
        double uwz = u[15];
        
        try {
            //update velocity
            Matrix Vv = new Matrix(SPEED_COMPONENTS, SPEED_COMPONENTS);
            Matrix Va = new Matrix(SPEED_COMPONENTS, ACCELERATION_COMPONENTS);
            double[] v = VelocityPredictor.predict(vx, vy, vz, ax, ay, az, dt, 
                    Vv, Va);
            
            //update position
            Matrix Rr = null, Rv = null, Ra = null;
            if(jacobianX != null) {
                Rr = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
                Rv = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 
                        SPEED_COMPONENTS);
                Ra = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                        ACCELERATION_COMPONENTS);
            }
            PositionPredictor.predictWithPositionAdjustment(r, drx, dry, drz, 
                    vx, vy, vz, ax, ay, az, dt, r, Rr, null, Rv, Ra);
            
            //update orientation
            Matrix Qq = null, Qdq = null, Qw = null;
            if(jacobianX != null) {
                Qq = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
                Qw = new Matrix(Quaternion.N_PARAMS, ANGULAR_SPEED_COMPONENTS);
            }
            if(jacobianU != null) {
                Qdq = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            }
            q = QuaternionPredictor.predictWithRotationAdjustment(q, dq, 
                    wx, wy, wz, dt, Qq, Qdq, Qw);

            
            //set updated linear velocity
            vx = v[0];
            vy = v[1];
            vz = v[2];
            
            //apply control signals
            vx += uvx;
            vy += uvy;
            vz += uvz;
            
            ax += uax;
            ay += uay;
            az += uaz;
            
            wx += uwx;
            wy += uwy;
            wz += uwz;
            
            //set new state
            result[0] = r.getInhomX();
            result[1] = r.getInhomY();
            result[2] = r.getInhomZ();
            
            result[3] = q.getA();
            result[4] = q.getB();
            result[5] = q.getC();
            result[6] = q.getD();
            
            result[7] = vx;
            result[8] = vy;
            result[9] = vz;
            
            result[10] = ax;
            result[11] = ay;
            result[12] = az;
            
            result[13] = wx;
            result[14] = wy;
            result[15] = wz;
            
            //jacobians
            if (jacobianX != null) {
                //[Rr   0   Rv  Ra  0]
                //[0    Qq  0   0   Qw]
                //[0    0   Vv  Va  0]
                //[0    0   0   eye 0]
                //[0    0   0   0   eye]
                jacobianX.initialize(0.0);
                jacobianX.setSubmatrix(0, 0, 2, 2, Rr);
               
                jacobianX.setSubmatrix(3, 3, 6, 6, Qq);
               
                jacobianX.setSubmatrix(0, 7, 2, 9, Rv);
                
                jacobianX.setSubmatrix(7, 7, 9, 9, Vv);
                
                jacobianX.setSubmatrix(0, 10, 2, 12, Ra);
                
                jacobianX.setSubmatrix(7, 10, 9, 12, Va);
                
                jacobianX.setSubmatrix(3, 13, 6, 15, Qw);
               
                for(int i = 10; i < STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS; i++) {
                    jacobianX.setElementAt(i, i, 1.0);
                }               
            }
            
            if (jacobianU != null) {
                jacobianU.initialize(0.0);
                
                //variation of position
                for(int i = 0; i < Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
                    jacobianU.setElementAt(i, i, 1.0);
                }
                
                //variation of rotation
                jacobianU.setSubmatrix(3, 3, 6, 6, Qdq);

                //variation of linear speed and acceleration, and angular speed
                for(int i = 7; i < STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS; i++) {
                    jacobianU.setElementAt(i, i, 1.0);
                }
            }
        }catch(WrongSizeException ignore) { /* never thrown */ }                
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: position-change-x,
     * position-change-y, position-change-z, quaternion-change-a, 
     * quaternion-change-b, quaternion-change-c, quaternion-change-d, 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, linear-acceleration-change-x, 
     * linear-acceleration-change-y, linear-acceleration-change-z, 
     * angular-velocity-change-x, angular-velocity-change-y, 
     * angular-velocity-change-z. Must have length 16.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must 
     * have length 16.
     * @throws IllegalArgumentException if system state array, control array or
     * result array do not have proper size.
     */
    public static void predictWithPositionAndRotationAdjustment(
            double[] x, double[] u, double dt, double[] result) 
            throws IllegalArgumentException {
        predictWithPositionAndRotationAdjustment(x, u, dt, result, null, null);
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: position-change-x,
     * position-change-y, position-change-z, quaternion-change-a, 
     * quaternion-change-b, quaternion-change-c, quaternion-change-d, 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, linear-acceleration-change-x, 
     * linear-acceleration-change-y, linear-acceleration-change-z, 
     * angular-velocity-change-x, angular-velocity-change-y, 
     * angular-velocity-change-z. Must have length 16.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianX jacobian wrt system state. Must be 16x16.
     * @param jacobianU jacobian wrt control. must be 16x16.
     * @return a new array containing updated system model.
     * @throws IllegalArgumentException if system state array, control array or
     * jacobians do not have proper size.
     */
    public static double[] predictWithPositionAndRotationAdjustment(
            double[] x, double [] u, double dt, Matrix jacobianX, 
            Matrix jacobianU) throws IllegalArgumentException {
        double[] result = new double[STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS];
        predictWithPositionAndRotationAdjustment(x, u, dt, result, jacobianX, 
                jacobianU);
        return result;
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity,
     * linear acceleration and angular velocity) assuming a constant 
     * acceleration model when no aceeleration or velocity control signal is
     * present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z, 
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, ancular-velocity-z. Must have 
     * length 16.
     * @param u perturbations or control signals: position-change-x,
     * position-change-y, position-change-z, quaternion-change-a, 
     * quaternion-change-b, quaternion-change-c, quaternion-change-d, 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, linear-acceleration-change-x, 
     * linear-acceleration-change-y, linear-acceleration-change-z, 
     * angular-velocity-change-x, angular-velocity-change-y, 
     * angular-velocity-change-z. Must have length 16.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new array containing updated system model.
     * @throws IllegalArgumentException if system state array, or control array
     * do not have proper size.
     */
    public static double[] predictWithPositionAndRotationAdjustment(
            double[] x, double[] u, double dt) throws IllegalArgumentException {
        double[] result = new double[STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS];
        predictWithPositionAndRotationAdjustment(x, u, dt, result);
        return result;        
    }    
}
