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
 * Utility class to predict device state (position, orientation, linear velocity
 * and angular velocity) assuming a constant velocity model (acceleration is
 * assumed zero under no external force).
 */
@SuppressWarnings("WeakerAccess")
public class ConstantVelocityModelStatePredictor {
    
    /**
     * Number of components on angular speed.
     */
    public static final int ANGULAR_SPEED_COMPONENTS = 3;
    
    /**
     * Number of components of speed.
     */
    public static final int SPEED_COMPONENTS = 3;
    
    /**
     * Number of components of constant velocity model state.
     */
    public static final int STATE_COMPONENTS = 13;
    
    /**
     * Number of components of constant velocity model control signal.
     */
    public static final int CONTROL_COMPONENTS = 6;
    
    /**
     * Number of components of constant velocity model state with position 
     * adjustment.
     */
    public static final int STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS = 13;
    
    /**
     * Number of components of constant velocity model with position adjustment 
     * control signal.
     */
    public static final int CONTROL_WITH_POSITION_ADJUSTMENT_COMPONENTS = 9;
    
    /**
     * Number of components of constant velocity model state with rotation 
     * adjustment.
     */
    public static final int STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS = 13;
    
    /**
     * Number of components of constant velocity model with rotation adjustment 
     * control signal.
     */
    public static final int CONTROL_WITH_ROTATION_ADJUSTMENT_COMPONENTS = 10;
    
    /**
     * Number of components of constant velocity model state with position and 
     * rotation adjustment.
     */
    public static final int STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS = 13;
        
    /**
     * Number of components of constant velocity model with position and 
     * rotation adjustment control signal.
     */
    public static final int CONTROL_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS = 13;    
    
    /**
     * Constructor.
     */
    private ConstantVelocityModelStatePredictor() { }
    
    /**
     * Updates the system model (position, orientation, linear velocity and 
     * angular velocity) assuming a constant velocity model (without 
     * acceleration) when no velocity control signal is present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have 
     * length 13.
     * @param u linear and angular velocity perturbations or controls: 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, angulat-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 6.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must 
     * have length 13.
     * @param jacobianX jacobian wrt system state. Must be 13x13.
     * @param jacobianU jacobian wrt control. Must be 13x6.
     * @throws IllegalArgumentException if system state array, control array,
     * result or jacobians do not have proper size.
     * @see <a href="https://github.com/joansola/slamtb">constVel.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(double[] x, double[] u, double dt, 
            double[] result, Matrix jacobianX, Matrix jacobianU)
            throws IllegalArgumentException {
        if(x.length != STATE_COMPONENTS) {
            throw new IllegalArgumentException("x must have length 13");
        }
        if(u.length != CONTROL_COMPONENTS) {
            throw new IllegalArgumentException("u must have length 6");
        }
        if(result.length != STATE_COMPONENTS) {
            throw new IllegalArgumentException("result must have length 13");
        }
        if(jacobianX != null && 
                (jacobianX.getRows() != STATE_COMPONENTS ||
                jacobianX.getColumns() != STATE_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt x must be 13x13");
        }
        if(jacobianU != null && 
                (jacobianU.getRows() != STATE_COMPONENTS ||
                jacobianU.getColumns() != CONTROL_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt u must be 13x6");
        }
        
        //position
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x[0], x[1], x[2]);
        
        //orientation
        Quaternion q = new Quaternion(x[3], x[4], x[5], x[6]);
        
        //linear velocity
        double vx = x[7];
        double vy = x[8];
        double vz = x[9];
        
        //angular velocity
        double wx = x[10];
        double wy = x[11];
        double wz = x[12];
        
        //linear velocity change (control)
        double uvx = u[0];
        double uvy = u[1];
        double uvz = u[2];
        
        //angular velocity change (control)
        double uwx = u[3];
        double uwy = u[4];
        double uwz = u[5];
        
        try {
            //update position
            Matrix Rr = null, Rv = null;
            if(jacobianX != null) {
                Rr = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
                Rv = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 
                        SPEED_COMPONENTS);
            }
            PositionPredictor.predict(r, vx, vy, vz, dt, r, Rr, Rv, null);
            
            //update orientation
            Matrix Qq = null, Qw = null;
            if(jacobianX != null) {
                Qq = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
                Qw = new Matrix(Quaternion.N_PARAMS, ANGULAR_SPEED_COMPONENTS);                
            }
            q = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, Qq, Qw);
            
            //apply control signals
            vx += uvx;
            vy += uvy;
            vz += uvz;
            
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
            
            result[10] = wx;
            result[11] = wy;
            result[12] = wz;
            
            //jacobians
            if (jacobianX != null) {
                //[Rr   0   Rv  0]
                //[0    Qq  0   Qw]
                //[0    0   eye 0]
                //[0    0   0   eye]                
                jacobianX.initialize(0.0);
                jacobianX.setSubmatrix(0, 0, 2, 2, Rr);
               
                jacobianX.setSubmatrix(3, 3, 6, 6, Qq);
               
                jacobianX.setSubmatrix(0, 7, 2, 9, Rv);
               
                for(int i = 7; i < STATE_COMPONENTS; i++) {
                    jacobianX.setElementAt(i, i, 1.0);
                }
               
                jacobianX.setSubmatrix(3, 10, 6, 12, Qw);
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
     * Updates the system model (position, orientation, linear velocity and 
     * angular velocity assuming a constant velocity model (without 
     * acceleration).
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have 
     * length 13.
     * @param u linear and angular velocity perturbations or controls: 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, angulat-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 6.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must
     * have length 13.
     * @throws IllegalArgumentException if system state array or control array 
     * or result do not have proper size.
     * @see <a href="https://github.com/joansola/slamtb">constVel.m at https://github.com/joansola/slamtb</a>
     */
    public static void predict(double[] x, double[] u, double dt, 
            double[] result) throws IllegalArgumentException {
        predict(x, u, dt, result, null, null);
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity and 
     * angular velocity assuming a constant velocity model (without 
     * acceleration).
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have 
     * length 13.
     * @param u linear and angular velocity perturbations or controls: 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, angulat-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 6.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianX jacobian wrt system state. Must be 13x13.
     * @param jacobianU jacobian wrt control. Must be 13x6.
     * @return instance where updated system model will be stored.
     * @throws IllegalArgumentException if system state array, control array or
     * jacobians do not have proper size.
     * @see <a href="https://github.com/joansola/slamtb">constVel.m at https://github.com/joansola/slamtb</a>
     */
    public static double[] predict(double[] x, double[] u, double dt, 
            Matrix jacobianX, Matrix jacobianU) 
            throws IllegalArgumentException {
        double[] result = new double[STATE_COMPONENTS];
        predict(x, u, dt, result, jacobianX, jacobianU);
        return result;
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity and 
     * angular velocity assuming a constant velocity model (without 
     * acceleration).
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have 
     * length 13.
     * @param u linear and angular velocity perturbations or controls: 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, angulat-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 6.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new instance containing the updated system state.
     * @throws IllegalArgumentException if system state array, control array or
     * jacobians do not have proper size.
     * @see <a href="https://github.com/joansola/slamtb">constVel.m at https://github.com/joansola/slamtb</a>
     */
    public static double[] predict(double[] x, double[] u, double dt) 
            throws IllegalArgumentException {
        double[] result = new double[STATE_COMPONENTS];
        predict(x, u, dt, result);
        return result;
    }
       
    /**
     * Updates the system model (position, orientation, linear velocity and 
     * angular velocity) assuming a constant velocity model (without 
     * acceleration) when no velocity control signal is present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have
     * length 13.
     * @param u linear ang angular velocity perturbations or controls and 
     * position perturbations or controls: position-change-x, position-change-y,
     * position-change-z, linear-velocity-change-x, linear-velocity-change-y,
     * linear-velocity-change-z, angular-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 9.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must 
     * have length 13.
     * @param jacobianX jacobian wrt system state. Must be 13x13.
     * @param jacobianU jacobian wrt control. Must be 13x9.
     * @throws IllegalArgumentException if system state array, control array,
     * result or jacobians do not have proper size.
     */
    public static void predictWithPositionAdjustment(
            double[] x, double[] u, double dt, double[] result, 
            Matrix jacobianX, Matrix jacobianU) 
            throws IllegalArgumentException {
        if(x.length != STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("x must have length 13");
        }
        if(u.length != CONTROL_WITH_POSITION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("u must have length 9");
        }
        if(result.length != STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("result must have length 13");
        }
        if(jacobianX != null && (jacobianX.getRows() != STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS ||
                jacobianX.getColumns() != STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt x must be 13x13");
        }
        if(jacobianU != null && (jacobianU.getRows() != STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS ||
                jacobianU.getColumns() != CONTROL_WITH_POSITION_ADJUSTMENT_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt u must be 13x9");
        }
        
        //position
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x[0], x[1], x[2]);
        
        //orientation
        Quaternion q = new Quaternion(x[3], x[4], x[5], x[6]);
        
        //linear velocity
        double vx = x[7];
        double vy = x[8];
        double vz = x[9];
        
        //angular velocity
        double wx = x[10];
        double wy = x[11];
        double wz = x[12];
        
        //position change (control)
        double drx = u[0];
        double dry = u[1];
        double drz = u[2];
        
        //linear velocity change (control)
        double uvx = u[3];
        double uvy = u[4];
        double uvz = u[5];
        
        //angular velocity change (control)
        double uwx = u[6];
        double uwy = u[7];
        double uwz = u[8];
        
        try {
            //update position
            Matrix Rr = null, Rv = null;
            if(jacobianX != null) {
                Rr = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
                Rv = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 
                        SPEED_COMPONENTS);
            }
            PositionPredictor.predictWithPositionAdjustment(r, drx, dry, drz, 
                    vx, vy, vz, 0.0, 0.0, 0.0, dt, r, Rr, null, Rv, null);
            
            //update orientation
            Matrix Qq = null, Qw = null;
            if(jacobianX != null) {
                Qq = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
                Qw = new Matrix(Quaternion.N_PARAMS, ANGULAR_SPEED_COMPONENTS);                
            }
            q = QuaternionPredictor.predict(q, wx, wy, wz, dt, true, Qq, Qw);
            
            //apply control signals
            vx += uvx;
            vy += uvy;
            vz += uvz;
            
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
            
            result[10] = wx;
            result[11] = wy;
            result[12] = wz;
            
            //jacobians
            if (jacobianX != null) {
                //[Rr   0   Rv  0]
                //[0    Qq  0   Qw]
                //[0    0   eye 0]
                //[0    0   0   eye]                
                jacobianX.initialize(0.0);
                jacobianX.setSubmatrix(0, 0, 2, 2, Rr);
               
                jacobianX.setSubmatrix(3, 3, 6, 6, Qq);
               
                jacobianX.setSubmatrix(0, 7, 2, 9, Rv);
               
                for(int i = 7; i < STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS; i++) {
                    jacobianX.setElementAt(i, i, 1.0);
                }
               
                jacobianX.setSubmatrix(3, 10, 6, 12, Qw);
            }
            
            if (jacobianU != null) {
                jacobianU.initialize(0.0);
                //variation of position
                for(int i = 0; i < Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
                    jacobianU.setElementAt(i, i, 1.0);
                }
                //variation of linear and angular speed
                for(int i = 7, j = Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH; i < STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS; i++, j++) {
                    jacobianU.setElementAt(i, j, 1.0);
                }
            }
        }catch (WrongSizeException ignore) { /* never thrown */ }
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity and 
     * angular velocity) assuming a constant velocity model (without 
     * acceleration) when no velocity control signal is present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have
     * length 13.
     * @param u linear ang angular velocity perturbations or controls and 
     * position perturbations or controls: position-change-x, position-change-y,
     * position-change-z, linear-velocity-change-x, linear-velocity-change-y,
     * linear-velocity-change-z, angular-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 9.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must 
     * have length 13.
     * @throws IllegalArgumentException if system state array, control array 
     * or result array do not have proper size.
     */
    public static void predictWithPositionAdjustment(
            double[] x, double[] u, double dt, double[] result) 
            throws IllegalArgumentException {
        predictWithPositionAdjustment(x, u, dt, result, null, null);
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity and 
     * angular velocity) assuming a constant velocity model (without 
     * acceleration) when no velocity control signal is present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have
     * length 13.
     * @param u linear ang angular velocity perturbations or controls and 
     * position perturbations or controls: position-change-x, position-change-y,
     * position-change-z, linear-velocity-change-x, linear-velocity-change-y,
     * linear-velocity-change-z, angular-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 9.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianX jacobian wrt system state. Must be 13x13.
     * @param jacobianU jacobian wrt control. Must be 13x9.
     * @return a new instance containing updated system model.
     * @throws IllegalArgumentException if system state array, control array
     * or jacobians do not have proper size.
     */
    public static double[] predictWithPositionAdjustment(double[] x, 
                double[] u, double dt, Matrix jacobianX, Matrix jacobianU)
                throws IllegalArgumentException {
        double[] result = new double[STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS];
        predictWithPositionAdjustment(x, u, dt, result, jacobianX, 
                jacobianU);
        return result;
    }

    /**
     * Updates the system model (position, orientation, linear velocity and 
     * angular velocity) assuming a constant velocity model (without 
     * acceleration) when no velocity control signal is present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have
     * length 13.
     * @param u linear ang angular velocity perturbations or controls and 
     * position perturbations or controls: position-change-x, position-change-y,
     * position-change-z, linear-velocity-change-x, linear-velocity-change-y,
     * linear-velocity-change-z, angular-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 9.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new instance containing updated system model.
     * @throws IllegalArgumentException if system state or control array do not
     * have proper size.
     */
    public static double[] predictWithPositionAdjustment(double[] x, 
                double[] u, double dt) throws IllegalArgumentException {
        double[] result = new double[STATE_WITH_POSITION_ADJUSTMENT_COMPONENTS];
        predictWithPositionAdjustment(x, u, dt, result);
        return result;
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity and
     * angular velocity) assuming a constant velocity model (without 
     * acceleration) when no velocity control signal is present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have
     * length 13.
     * @param u linear and angular velocity perturbations or controls, and 
     * rotation perturbations or controls: quaternion-change-a, 
     * quaternion-change-b, quaternion-change-c, quaternion-change-d, 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, angular-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 
     * 10.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must
     * have length 13.
     * @param jacobianX jacobian wrt system state. Must be 13x13.
     * @param jacobianU jacobian wrt control. Must be 13x10.
     * @throws IllegalArgumentException if system state array, control array,
     * result or jacobians do not have proper size.
     */
    public static void predictWithRotationAdjustment(double[] x, 
            double[] u, double dt, double[] result, Matrix jacobianX, 
            Matrix jacobianU) throws IllegalArgumentException {
        if(x.length != STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("x must have length 13");
        }
        if(u.length != CONTROL_WITH_ROTATION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("u must have length 10");
        }
        if(result.length != STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("result must have length 13");
        }
        if(jacobianX != null && (jacobianX.getRows() != STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS ||
                jacobianX.getColumns() != STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt x must be 13x13");
        }
        if(jacobianU != null && (jacobianU.getRows() != STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS ||
                jacobianU.getColumns() != CONTROL_WITH_ROTATION_ADJUSTMENT_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt u must be 13x10");
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
        
        //angular velocity
        double wx = x[10];
        double wy = x[11];
        double wz = x[12];

        //rotation change (control)
        Quaternion dq = new Quaternion(u[0], u[1], u[2], u[3]);
        
        //linear velocity change (control)
        double uvx = u[4];
        double uvy = u[5];
        double uvz = u[6];
        
        //angular velocity change (control)
        double uwx = u[7];
        double uwy = u[8];
        double uwz = u[9];
        
        try {
            //update position
            Matrix Rr = null, Rv = null;
            if(jacobianX != null) {
                Rr = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
                Rv = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                        SPEED_COMPONENTS);
            }
            PositionPredictor.predict(r, vx, vy, vz, dt, r, Rr, Rv, null);

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
            
            //apply control signals
            vx += uvx;
            vy += uvy;
            vz += uvz;
            
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
            
            result[10] = wx;
            result[11] = wy;
            result[12] = wz;
            
            //jacobians
            if (jacobianX != null) {
                //[Rr   0   Rv  0]
                //[0    Qq  0   Qw]
                //[0    0   eye 0]
                //[0    0   0   eye]                
                jacobianX.initialize(0.0);
                jacobianX.setSubmatrix(0, 0, 2, 2, Rr);
               
                jacobianX.setSubmatrix(3, 3, 6, 6, Qq);
               
                jacobianX.setSubmatrix(0, 7, 2, 9, Rv);
               
                for(int i = 7; i < STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS; i++) {
                    jacobianX.setElementAt(i, i, 1.0);
                }
               
                jacobianX.setSubmatrix(3, 10, 6, 12, Qw);
            }
            
            if (jacobianU != null) {
                jacobianU.initialize(0.0);
                
                //variation of rotation
                jacobianU.setSubmatrix(3, 0, 6, 3, Qdq);
                
                //variation of linear and angular speed
                for(int i = 7, j = Quaternion.N_PARAMS; i < STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS; i++, j++) {
                    jacobianU.setElementAt(i, j, 1.0);
                }
            }
            
        } catch (WrongSizeException ignore) { /* never thrown */ }
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity and
     * angular velocity) assuming a constant velocity model (without 
     * acceleration) when no velocity control signal is present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have
     * length 13.
     * @param u linear and angular velocity perturbations or controls, and 
     * rotation perturbations or controls: quaternion-change-a, 
     * quaternion-change-b, quaternion-change-c, quaternion-change-d, 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, angular-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 
     * 10.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must
     * have length 13.
     * @throws IllegalArgumentException if system state array, control array or
     * result do not have proper length.
     */
    public static void predictWithRotationAdjustment(double[] x, 
            double[] u, double dt, double[] result) 
            throws IllegalArgumentException {
        predictWithRotationAdjustment(x, u, dt, result, null, null);
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity and
     * angular velocity) assuming a constant velocity model (without 
     * acceleration) when no velocity control signal is present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have
     * length 13.
     * @param u linear and angular velocity perturbations or controls, and 
     * rotation perturbations or controls: quaternion-change-a, 
     * quaternion-change-b, quaternion-change-c, quaternion-change-d, 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, angular-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 
     * 10.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianX jacobian wrt system state. Must be 13x13.
     * @param jacobianU jacobian wrt control. Must be 13x10.
     * @return a new array containing updated system model.
     * @throws IllegalArgumentException if system state array, control array
     * or jacobians do not have proper size.
     */
    public static double[] predictWithRotationAdjustment(double[] x, 
                double[] u, double dt, Matrix jacobianX, Matrix jacobianU)
                throws IllegalArgumentException {
        double[] result = new double[
                STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS];
        predictWithRotationAdjustment(x, u, dt, result, jacobianX, jacobianU);
        return result;
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity and
     * angular velocity) assuming a constant velocity model (without 
     * acceleration) when no velocity control signal is present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have
     * length 13.
     * @param u linear and angular velocity perturbations or controls, and 
     * rotation perturbations or controls: quaternion-change-a, 
     * quaternion-change-b, quaternion-change-c, quaternion-change-d, 
     * linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, angular-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 
     * 10.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new array containing updated system model.
     * @throws IllegalArgumentException if system state array or control array
     * do not have proper size.
     */
    public static double[] predictWithRotationAdjustment(double[] x, 
                double[] u, double dt) throws IllegalArgumentException {
        double[] result = new double[
                STATE_WITH_ROTATION_ADJUSTMENT_COMPONENTS];
        predictWithRotationAdjustment(x, u, dt, result);
        return result;
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity and
     * angular velocity) assuming a constant velocity model (without 
     * acceleration) when no velocity control signal is present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have
     * length 13.
     * @param u linear and angular velocity perturbations or controls, position
     * perturbations or controls and rotation perturbation or control: 
     * position-change-x, position-change-y, position-change-z, 
     * quaternion-change-a, quaternion-change-b, quaternion-change-c, 
     * quaternion-change-d, linear-velocity-change-x, linear-velocity-change-y,
     * linear-velocity-change-z, angular-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 
     * 12.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must
     * have length 13.
     * @param jacobianX jacobian wrt system state. Must be 13x13.
     * @param jacobianU jacobian wrt control. Must be 13x13.
     * @throws IllegalArgumentException if system state array, control array,
     * result or jacobians do not have proper size.
     */
    public static void predictWithPositionAndRotationAdjustment(double[] x, 
            double[] u, double dt, double[] result, Matrix jacobianX, 
            Matrix jacobianU) throws IllegalArgumentException {
        if(x.length != STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("x must have length 13");
        }
        if(u.length != CONTROL_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("u must have length 13");
        }
        if(result.length != STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS) {
            throw new IllegalArgumentException("result must have length 13");
        }
        if(jacobianX != null && (jacobianX.getRows() != STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS ||
                jacobianX.getColumns() != STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt x must be 13x13");
        }
        if(jacobianU != null && (jacobianU.getRows() != STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS ||
                jacobianU.getColumns() != CONTROL_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS)) {
            throw new IllegalArgumentException("jacobian wrt u must be 13x13");
        }
        
        //position
        InhomogeneousPoint3D r = new InhomogeneousPoint3D(x[0], x[1], x[2]);
        
        //orientation
        Quaternion q = new Quaternion(x[3], x[4], x[5], x[6]);
        
        //linear velocity
        double vx = x[7];
        double vy = x[8];
        double vz = x[9];
        
        //angular velocity
        double wx = x[10];
        double wy = x[11];
        double wz = x[12];

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
        
        //angular velocity change (control)
        double uwx = u[10];
        double uwy = u[11];
        double uwz = u[12];
        
        try {
            //update position
            Matrix Rr = null, Rv = null;
            if(jacobianX != null) {
                Rr = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
                Rv = new Matrix(
                        Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                        SPEED_COMPONENTS);
            }
            PositionPredictor.predictWithPositionAdjustment(r, drx, dry, drz, 
                    vx, vy, vz, 0.0, 0.0, 0.0, dt, r, Rr, null, Rv, null);

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
            
            //apply control signals
            vx += uvx;
            vy += uvy;
            vz += uvz;
            
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
            
            result[10] = wx;
            result[11] = wy;
            result[12] = wz;
            
            //jacobians
            if (jacobianX != null) {
                //[Rr   0   Rv  0]
                //[0    Qq  0   Qw]
                //[0    0   eye 0]
                //[0    0   0   eye]                
                jacobianX.initialize(0.0);
                jacobianX.setSubmatrix(0, 0, 2, 2, Rr);
               
                jacobianX.setSubmatrix(3, 3, 6, 6, Qq);
               
                jacobianX.setSubmatrix(0, 7, 2, 9, Rv);
               
                for(int i = 7; i < STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS; i++) {
                    jacobianX.setElementAt(i, i, 1.0);
                }
               
                jacobianX.setSubmatrix(3, 10, 6, 12, Qw);
            }
            
            if (jacobianU != null) {
                jacobianU.initialize(0.0);
                //variation of position
                for(int i = 0; i < Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
                    jacobianU.setElementAt(i, i, 1.0);
                }
                
                //variation of rotation
                jacobianU.setSubmatrix(3, 3, 6, 6, Qdq);
                
                //variation of linear and angular speed
                for(int i = 7; i < STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS; i++) {
                    jacobianU.setElementAt(i, i, 1.0);
                }
            }
            
        } catch (WrongSizeException ignore) { /* never thrown */ }
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity and
     * angular velocity) assuming a constant velocity model (without 
     * acceleration) when no velocity control signal is present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have
     * length 13.
     * @param u linear and angular velocity perturbations or controls, position
     * perturbations or controls and rotation perturbation or control: 
     * position-change-x, position-change-y, position-change-z, 
     * quaternion-change-a, quaternion-change-b, quaternion-change-c, 
     * quaternion-change-d, linear-velocity-change-x, linear-velocity-change-y,
     * linear-velocity-change-z, angular-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 
     * 12.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param result instance where updated system model will be stored. Must
     * have length 13.
     * @throws IllegalArgumentException if system state array, control array,
     * result or jacobians do not have proper size.
     */
    public static void predictWithPositionAndRotationAdjustment(
                    double[] x, double[] u, double dt, double[] result)
                    throws IllegalArgumentException {
        predictWithPositionAndRotationAdjustment(x, u, dt, result, null, null);
    }
    
    /**
     * Updates the system model (position, orientation, linear velocity and
     * angular velocity) assuming a constant velocity model (without 
     * acceleration) when no velocity control signal is present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have
     * length 13.
     * @param u linear and angular velocity perturbations or controls, position
     * perturbations or controls and rotation perturbation or control: 
     * position-change-x, position-change-y, position-change-z, 
     * quaternion-change-a, quaternion-change-b, quaternion-change-c, 
     * quaternion-change-d, linear-velocity-change-x, linear-velocity-change-y,
     * linear-velocity-change-z, angular-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 
     * 12.
     * @param dt time interval to compute prediction expressed in seconds.
     * @param jacobianX jacobian wrt system state. Must be 13x13.
     * @param jacobianU jacobian wrt control. Must be 13x13.
     * @return a new array containing updated system model.
     * @throws IllegalArgumentException if system state array, control array
     * or jacobians do not have proper size.
     */
    public static double[] predictWithPositionAndRotationAdjustment(double[] x, 
            double[] u, double dt, Matrix jacobianX, Matrix jacobianU) 
            throws IllegalArgumentException {
        double[] result = new double[
                STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS];
        predictWithPositionAndRotationAdjustment(x, u, dt, result, jacobianX, 
                jacobianU);
        return result;
    }
            
    /**
     * Updates the system model (position, orientation, linear velocity and
     * angular velocity) assuming a constant velocity model (without 
     * acceleration) when no velocity control signal is present.
     * @param x initial system state containing: position-x, position-y, 
     * position-z, quaternion-a, quaternion-b, quaternion-c, quaternion-d,
     * linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z. Must have
     * length 13.
     * @param u linear and angular velocity perturbations or controls, position
     * perturbations or controls and rotation perturbation or control: 
     * position-change-x, position-change-y, position-change-z, 
     * quaternion-change-a, quaternion-change-b, quaternion-change-c, 
     * quaternion-change-d, linear-velocity-change-x, linear-velocity-change-y,
     * linear-velocity-change-z, angular-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z. Must have length 
     * 12.
     * @param dt time interval to compute prediction expressed in seconds.
     * @return a new array containing updated system model. Must have length 13.
     * @throws IllegalArgumentException if system state array, control array or
     * result do not have proper size.
     */
    public static double[] predictWithPositionAndRotationAdjustment(
                    double[] x, double[] u, double dt) 
                    throws IllegalArgumentException {
        double[] result = new double[
                STATE_WITH_POSITION_AND_ROTATION_ADJUSTMENT_COMPONENTS];
        predictWithPositionAndRotationAdjustment(x, u, dt, result);
        return result;
    }    
        
}
