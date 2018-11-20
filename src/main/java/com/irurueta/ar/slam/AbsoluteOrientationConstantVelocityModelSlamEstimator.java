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
import com.irurueta.numerical.signal.processing.KalmanFilter;
import com.irurueta.numerical.signal.processing.SignalProcessingException;
import com.irurueta.statistics.InvalidCovarianceMatrixException;
import com.irurueta.statistics.MultivariateNormalDist;

import java.io.Serializable;

/**
 * Estimates position, velocity, acceleration, orientation and angular speed
 * using data from accelerometer, gyroscope and magnetic field, assuming a 
 * constant velocity model.
 * This implementation of BaseSlamEstimator is an improvement respect 
 * SlamEstimator to be able to take into account absolute orientation respect
 * Earth frame instead of just having a relative orientation respect to the 
 * start time of the estimator.
 */
public class AbsoluteOrientationConstantVelocityModelSlamEstimator extends 
        AbsoluteOrientationBaseSlamEstimator<AbsoluteOrientationConstantVelocityModelSlamCalibrationData> 
        implements Serializable {

    /**
     * Internal state array length.
     */
    protected static final int STATE_LENGTH = 13;
    
    /**
     * Length of control array (changes in acceleration and angular speed).
     */
    protected static final int CONTROL_LENGTH = 10;
    
    /**
     * Length of position measurement, to correct any possible deviations of the
     * system after doing multiple predictions.
     */
    private static final int MEASUREMENT_LENGTH = 3;
    
    /**
     * Contains device status containing the following values: position-x,
     * position-y, position-z, quaternion-a, quaternion-b, quaternion-c, 
     * quaternion-d, linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z.
     */
    private double[] mX;
    
    /**
     * Control signals containing the following values: 
     * quaternion-change-a, quaternion-change-b, quaternion-change-c, 
     * quaternion-change-d, linear-velocity-change-x, linear-velocity-change-y, 
     * linear-velocity-change-z, angular-velocity-change-x, 
     * angular-velocity-change-y, angular-velocity-change-z.
     */
    private double[] mU;
    
    /**
     * Jacobian respect x state during prediction (16x16).
     */
    private Matrix mJacobianPredictionX;
    
    /**
     * Jacobian respect u control during state prediction (16x13).
     */
    private Matrix mJacobianPredictionU;
    
    /**
     * Column matrix containing mU values to be passed as control values during
     * Kalman filter prediction.
     */
    private Matrix mControl;
    
    /**
     * Kalman's filter to remove effects of noise.
     */
    private KalmanFilter mKalmanFilter;
    
    /**
     * Matrix of size 3x13 relating system status with obtained measures.
     * [1   0   0   0   0   0   0   0   0   0   0   0   0]
     * [0   1   0   0   0   0   0   0   0   0   0   0   0]
     * [0   0   1   0   0   0   0   0   0   0   0   0   0]
     */    
    private Matrix mMeasurementMatrix;
    
    /**
     * Measurement data for the Kalman filter in a column matrix.
     * Contains data in the following order:
     * [accelerationX]
     * [accelerationY]
     * [accelerationZ]
     * [angularSpeedX]
     * [angularSpeedY]
     * [angularSpeedZ]
     */
    private Matrix mMeasurement;
    
    /**
     * Last sample of absolute orientation.
     */
    private Quaternion mLastOrientation = new Quaternion();
    
    /**
     * Variation of orientation respect to last sample.
     */
    private Quaternion mDeltaOrientation = new Quaternion();
    
    /**
     * Current state orientation.
     */
    private Quaternion mStateOrientation = new Quaternion();
    
    /**
     * Last sample of angular speed along x-axis.
     */
    private double mLastAngularSpeedX;
    
    /**
     * Last sample of angular speed along y-axis.
     */
    private double mLastAngularSpeedY;
    
    /**
     * Last sample of angular speed along z-axis.
     */
    private double mLastAngularSpeedZ;
    
    /**
     * Last timestamp of a full sample expressed in nanoseconds since the epoch 
     * time..
     */
    private long mLastTimestampNanos = -1;
    
    /**
     * Indicates whether a prediction has been made to initialize the internal 
     * Kalman filter. Corrections can only be requested to Kalman filter once
     * a prediction has been made. Attempts to request corrections before having
     * a prediction will be ignored.
     */
    private boolean mPredictionAvailable;    
    
    /**
     * Constructor.
     */    
    public AbsoluteOrientationConstantVelocityModelSlamEstimator() {
        super();
        mX = new double[STATE_LENGTH];
        mX[3] = 1.0; //initial value of quaternion.
        mU = new double[CONTROL_LENGTH];
        try {
            mJacobianPredictionX = new Matrix(STATE_LENGTH, STATE_LENGTH);
            mJacobianPredictionU = new Matrix(STATE_LENGTH, CONTROL_LENGTH);
            mControl = new Matrix(CONTROL_LENGTH, 1);
            mMeasurement = new Matrix(MEASUREMENT_LENGTH, 1);
            mMeasurementMatrix = Matrix.identity(MEASUREMENT_LENGTH, 
                    STATE_LENGTH);
                        
        } catch (WrongSizeException ignore) { /* never thrown */ }
        
        try {
            mKalmanFilter = new KalmanFilter(STATE_LENGTH, MEASUREMENT_LENGTH, 
                    CONTROL_LENGTH);
            //setup matrix relating position measures with internal status.
            mKalmanFilter.setMeasurementMatrix(mMeasurementMatrix);
        } catch (SignalProcessingException ignore) { /* never thrown */ }
        
        try {
            //set initial Kalman filter state (state pre and pro must be two 
            //different instances!)
            mKalmanFilter.getStatePre().fromArray(mX);
            mKalmanFilter.getStatePost().fromArray(mX);

        } catch (WrongSizeException ignore) { /* never thrown */ }        
    }

    /**
     * Gets covariance matrix of state variables (position, velocity, acceleration, orientation and angular speed).
     * Diagonal elements of matrix returned by this method are in the following order:
     * position-x, position-y, position-z, quaternion-a, quaternion-b, quaternion-c,
     * quaternion-d, linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z.
     * Off-diagonal elements correspond to cross-correlation values of diagonal ones.
     * @return covariance matrix of state variables.
     */
    @Override
    public Matrix getStateCovariance() {
        return mKalmanFilter.getStatePre();
    }

    /**
     * Updates covariance matrix of position measures.
     * If null is provided, covariance matrix is not updated.
     * @param positionCovariance new position covariance determining position
     * accuracy or null if last available covariance does not need to be 
     * updated.
     * @throws IllegalArgumentException if provided covariance matrix is not 
     * 3x3.
     */
    @Override
    public void setPositionCovarianceMatrix(Matrix positionCovariance)
            throws IllegalArgumentException {
        if(positionCovariance != null) {
            mKalmanFilter.setMeasurementNoiseCov(positionCovariance);        
        }
    }    
    
    /**
     * Gets current covariance matrix of position measures determining current
     * accuracy of provided position measures.
     * @return covariance matrix of position measures.
     */
    @Override
    public Matrix getPositionCovarianceMatrix() {
        return mKalmanFilter.getMeasurementNoiseCov();
    }
    
    /**
     * Corrects system state with provided position measure using current
     * position accuracy.
     * @param positionX new position along x axis expressed in meters (m).
     * @param positionY new position along y axis expressed in meters (m).
     * @param positionZ new position along z axis expressed in meters (m).
     */
    @Override
    public void correctWithPositionMeasure(double positionX, double positionY, 
            double positionZ) {
        if (!mPredictionAvailable) {
            return;
        }
        
        if(mListener != null) {
            mListener.onCorrectWithPositionMeasure(this);
        }
        
        try {
            mMeasurement.setElementAtIndex(0, positionX);
            mMeasurement.setElementAtIndex(1, positionY);
            mMeasurement.setElementAtIndex(2, positionZ);
            
            updateCorrectedState(mKalmanFilter.correct(mMeasurement));
            
            //copy corrected state to predicted state
            mKalmanFilter.getStatePost().copyTo(mKalmanFilter.getStatePre());
            mKalmanFilter.getErrorCovPost().copyTo(
                    mKalmanFilter.getErrorCovPre());
            
        } catch (SignalProcessingException e) {
            mError = true;
        }
        
        if(mListener != null) {
            mListener.onCorrectedWithPositionMeasure(this);
        }        
    }
    
    /**
     * Creates an instance of a calibrator to be used with this SLAM estimator.
     * @return a calibrator.
     */
    public static AbsoluteOrientationConstantVelocityModelSlamCalibrator 
        createCalibrator() {
        return new AbsoluteOrientationConstantVelocityModelSlamCalibrator();
    }
            
    /**
     * Processes a full sample (accelerometer + gyroscope) to update system 
     * state.
     */
    @Override
    protected void processFullSample() {
        if(mListener != null) {
            mListener.onFullSampleReceived(this);
        }
        
        long timestamp = getMostRecentTimestampNanos();
        if (mLastTimestampNanos < 0) {
            //first time receiving control data, we just set linear acceleration 
            //and angular speed into system state
            mStateAccelerationX = mAccumulatedAccelerationSampleX;
            mStateAccelerationY = mAccumulatedAccelerationSampleY;
            mStateAccelerationZ = mAccumulatedAccelerationSampleZ;
            mLastAngularSpeedX = mStateAngularSpeedX = mX[10] = 
                    mAccumulatedAngularSpeedSampleX;
            mLastAngularSpeedY = mStateAngularSpeedY = mX[11] = 
                    mAccumulatedAngularSpeedSampleY;
            mLastAngularSpeedZ = mStateAngularSpeedZ = mX[12] = 
                    mAccumulatedAngularSpeedSampleZ;

            try {
                mKalmanFilter.getStatePre().fromArray(mX);
                mKalmanFilter.getStatePost().fromArray(mX);
            } catch(WrongSizeException ignore) { /* never thrown */ }
                        
            mLastTimestampNanos = timestamp;
            
            if(mListener != null) {
                mListener.onFullSampleProcessed(this);
            }
            
            return;
        }
        
        mAccumulatedOrientation.normalize();
        
        mLastOrientation.inverse(mDeltaOrientation);
        mDeltaOrientation.combine(mAccumulatedOrientation);
        mDeltaOrientation.normalize();
        
        double deltaAngularSpeedX = mAccumulatedAngularSpeedSampleX -
                mLastAngularSpeedX;
        double deltaAngularSpeedY = mAccumulatedAngularSpeedSampleY -
                mLastAngularSpeedY;
        double deltaAngularSpeedZ = mAccumulatedAngularSpeedSampleZ -
                mLastAngularSpeedZ;
        double deltaTimestamp = (timestamp - mLastTimestampNanos) * 
                NANOS_TO_SECONDS;
        
        //when a full sample is received, we update the data model
        mU[0] = mDeltaOrientation.getA();
        mU[1] = mDeltaOrientation.getB();
        mU[2] = mDeltaOrientation.getC();
        mU[3] = mDeltaOrientation.getD();
        mU[4] = mU[5] = mU[6] = 0.0; //change in linear speed
        mU[7] = deltaAngularSpeedX;
        mU[8] = deltaAngularSpeedY;
        mU[9] = deltaAngularSpeedZ;
        
        if(mCalibrationData != null && 
                mCalibrationData.getControlMean() != null) {
            //if calibrator is available, remove mean to correct possible biases
            ArrayUtils.subtract(mU, mCalibrationData.getControlMean(), mU);
        }
        
        ConstantVelocityModelStatePredictor.predictWithRotationAdjustment(mX, mU, deltaTimestamp, mX,
                mJacobianPredictionX, mJacobianPredictionU);
        
        //update Kalman filter transition matrix taking into account current 
        //state
        mKalmanFilter.setTransitionMatrix(mJacobianPredictionX);
        
        //update control matrix from control vector jacobian
        mKalmanFilter.setControlMatrix(mJacobianPredictionU);
        
        if(mCalibrationData != null && 
                mCalibrationData.getControlMean() != null &&
                mCalibrationData.getControlCovariance() != null) {
            //if calibrator is available, propagate covariance to set process
            //covariance matrix
            if(mNormalDist == null) {
                mNormalDist = new MultivariateNormalDist(STATE_LENGTH);
            }
            
            try {
                mCalibrationData.propagateWithControlJacobian(
                        mJacobianPredictionU, mNormalDist);
                //update kalman filter process noise
                Matrix processNoise = mKalmanFilter.getProcessNoiseCov();
                
                //copy normal dist covariance into processNoise
                mNormalDist.getCovariance(processNoise);
            } catch (InvalidCovarianceMatrixException e) {
                /* ignore */
            }
        }
        
        try {
            //also predict the state using Kalman filter with current control 
            //data
            mControl.fromArray(mU, true);
            updatePredictedState(mKalmanFilter.predict(mControl));
            
            //coopy predicted state to corrected state
            mKalmanFilter.getStatePre().copyTo(mKalmanFilter.getStatePost());
            mKalmanFilter.getErrorCovPre().copyTo(
                    mKalmanFilter.getErrorCovPost());
            
            mPredictionAvailable = true;
        } catch (Exception e) {
            mError = true;
        }
        
        mLastOrientation.fromRotation(mStateOrientation);
        mLastAngularSpeedX = mStateAngularSpeedX;
        mLastAngularSpeedY = mStateAngularSpeedY;
        mLastAngularSpeedZ = mStateAngularSpeedZ;
        mLastTimestampNanos = timestamp;     
        
        if(mListener != null) {
            mListener.onFullSampleProcessed(this);
        }        
    }
    
    /**
     * Resets position, linear velocity, linear acceleration, orientation and
     * angular speed to provided values.
     * This method implementation also resets Kalman filter state.
     * @param statePositionX position along x-axis expressed in meters (m).
     * @param statePositionY position along y-axis expressed in meters (m).
     * @param statePositionZ position along z-axis expressed in meters (m).
     * @param stateVelocityX linear velocity along x-axis expressed in meters 
     * per second (m/s).
     * @param stateVelocityY linear velocity along y-axis expressed in meters 
     * per second (m/s).
     * @param stateVelocityZ linear velocity along z-axis expressed in meters 
     * per second (m/s).
     * @param stateAccelerationX linear acceleration along x-axis expressed in
     * meters per squared second (m/s^2).
     * @param stateAccelerationY linear acceleration along y-axis expressed in
     * meters per squared second (m/s^2).
     * @param stateAccelerationZ linear acceleration along z-axis expressed in
     * meters per squared second (m/s^2).
     * @param stateQuaternionA A value of orientation quaternion.
     * @param stateQuaternionB B value of orientation quaternion.
     * @param stateQuaternionC C value of orientation quaternion.
     * @param stateQuaternionD D value of orientation quaternion.
     * @param stateAngularSpeedX angular speed along x-axis expressed in radians
     * per second (rad/s).
     * @param stateAngularSpeedY angular speed along y-axis expressed in radians
     * per second (rad/s).
     * @param stateAngularSpeedZ angular speed along z-axis expressed in radians
     * per second (rad/s).
     */
    @Override
    protected void reset(double statePositionX, double statePositionY,
            double statePositionZ, double stateVelocityX, double stateVelocityY,
            double stateVelocityZ, double stateAccelerationX, 
            double stateAccelerationY, double stateAccelerationZ, 
            double stateQuaternionA, double stateQuaternionB, 
            double stateQuaternionC, double stateQuaternionD, 
            double stateAngularSpeedX, double stateAngularSpeedY,
            double stateAngularSpeedZ) {
        super.reset(statePositionX, statePositionY, statePositionZ,
                stateVelocityX, stateVelocityY, stateVelocityZ,
                stateAccelerationX, stateAccelerationY, stateAccelerationZ,
                stateQuaternionA, stateQuaternionB, stateQuaternionC, stateQuaternionD,
                stateAngularSpeedX, stateAngularSpeedY, stateAngularSpeedZ);                
        if(mStateOrientation != null) {
            mStateOrientation.setA(stateQuaternionA);
            mStateOrientation.setB(stateQuaternionB);
            mStateOrientation.setC(stateQuaternionC);
            mStateOrientation.setD(stateQuaternionD);            
        }
        
        if(mLastOrientation != null) {
            mLastOrientation.setA(1.0);
            mLastOrientation.setB(0.0);
            mLastOrientation.setC(0.0);
            mLastOrientation.setD(0.0);
        }
        
        if(mX != null) {
            //position
            mX[0] = statePositionX;
            mX[1] = statePositionY;
            mX[2] = statePositionZ;
            
            //quaternion
            mX[3] = stateQuaternionA;
            mX[4] = stateQuaternionB;
            mX[5] = stateQuaternionC;
            mX[6] = stateQuaternionD;
            
            //velocity
            mX[7] = stateVelocityX;
            mX[8] = stateVelocityY;
            mX[9] = stateVelocityZ;
            
            //angular speed
            mX[10] = stateAngularSpeedX;
            mX[11] = stateAngularSpeedY;
            mX[12] = stateAngularSpeedZ;                        

            try {
                //set initial Kalman filter state (state pre and pro must be two 
                //different instances!)
                mKalmanFilter.getStatePre().fromArray(mX);
                mKalmanFilter.getStatePost().fromArray(mX);
            } catch (WrongSizeException ignore) { /* never thrown */ }
        }

        mError = false;    
        mLastTimestampNanos = -1;
        mPredictionAvailable = false;
    }  
    
    /**
     * Updates state data of the device by using state matrix obtained after 
     * prediction from Kalman filter.
     * to ensure that state follows proper values (specially on quaternions),
     * we keep x values, which have been predicted using the state predictor,
     * which uses analytical values.
     * We then updated x using latest Kalman filter state for next iteration
     * on state predictor.
     * @param state state matrix obtained from Kalman filter.
     */
    private void updatePredictedState(Matrix state) {
        //position
        mStatePositionX = mX[0];
        mX[0] = state.getElementAtIndex(0);
        mStatePositionY = mX[1];
        mX[1] = state.getElementAtIndex(1);
        mStatePositionZ = mX[2];
        mX[2] = state.getElementAtIndex(2);

        //quaternion (state predictor is more reliable than Kalman filter, for
        //that reason we ignore predicted quaternion values on Kalman filter and
        //simply keep predicted ones. Besides, typically gyroscope samples are
        //much more reliable than accelerometer ones. For that reason state
        //elements corresponding to quaternion (3 to 6) are not copied into mX 
        //array.
        mStateQuaternionA = mX[3];
        mStateQuaternionB = mX[4];
        mStateQuaternionC = mX[5];
        mStateQuaternionD = mX[6];
        
        mStateOrientation.setA(mStateQuaternionA);
        mStateOrientation.setB(mStateQuaternionB);
        mStateOrientation.setC(mStateQuaternionC);
        mStateOrientation.setD(mStateQuaternionD);                    

        //velocity
        mStateVelocityX = mX[7];
        mX[7] = state.getElementAtIndex(7);
        mStateVelocityY = mX[8];
        mX[8] = state.getElementAtIndex(8);
        mStateVelocityZ = mX[9];
        mX[9] = state.getElementAtIndex(9);

        //linear acceleration
        mStateAccelerationX = mAccumulatedAccelerationSampleX;
        mStateAccelerationY = mAccumulatedAccelerationSampleY;
        mStateAccelerationZ = mAccumulatedAccelerationSampleZ;

        //angular velocity
        mStateAngularSpeedX = mX[10];
        mX[10] = state.getElementAtIndex(10);
        mStateAngularSpeedY = mX[11];
        mX[11] = state.getElementAtIndex(11);
        mStateAngularSpeedZ = mX[12];
        mX[12] = state.getElementAtIndex(12);
    }
    
    /**
     * Updates state data of the device by using state matrix obtained from 
     * Kalman filter after correction..
     * @param state state matrix obtained from Kalman filter.
     */    
    private void updateCorrectedState(Matrix state) {
        //position
        mStatePositionX = mX[0] = 
                state.getElementAtIndex(0);
        mStatePositionY = mX[1] = 
                state.getElementAtIndex(1);
        mStatePositionZ = mX[2] = 
                state.getElementAtIndex(2);

        //quaternion
        mStateQuaternionA = mX[3] = 
                state.getElementAtIndex(3);
        mStateQuaternionB = mX[4] = 
                state.getElementAtIndex(4);
        mStateQuaternionC = mX[5] = 
                state.getElementAtIndex(5);
        mStateQuaternionD = mX[6] = 
                state.getElementAtIndex(6);

        mStateOrientation.setA(mStateQuaternionA);
        mStateOrientation.setB(mStateQuaternionB);
        mStateOrientation.setC(mStateQuaternionC);
        mStateOrientation.setD(mStateQuaternionD);   
        mStateOrientation.normalize();
        
        //velocity
        mStateVelocityX = mX[7] = 
                state.getElementAtIndex(7);
        mStateVelocityY = mX[8] = 
                state.getElementAtIndex(8);
        mStateVelocityZ = mX[9] = 
                state.getElementAtIndex(9);

        //linear acceleration (is not contained in state, thus it is not 
        //corrected)

        //angular velocity
        mStateAngularSpeedX = mX[10] = 
                state.getElementAtIndex(10);
        mStateAngularSpeedY = mX[10] = 
                state.getElementAtIndex(10);
        mStateAngularSpeedZ = mX[10] = 
                state.getElementAtIndex(10);
    }        
}
