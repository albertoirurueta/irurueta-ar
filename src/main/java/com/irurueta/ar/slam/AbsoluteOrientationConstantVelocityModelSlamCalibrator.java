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

import com.irurueta.geometry.Quaternion;

import java.io.Serializable;

/**
 * Processes data to estimate calibration for absolute orientation with contant 
 * velocity model SLAM estimator.
 * This class must be used while gathering data for a system being kept constant
 * (under no motion).
 */
public class AbsoluteOrientationConstantVelocityModelSlamCalibrator extends 
        AbsoluteOrientationBaseSlamCalibrator<AbsoluteOrientationConstantVelocityModelSlamCalibrationData> 
        implements Serializable {
    
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
     * time.
     */
    private long mLastTimestampNanos = -1;
    
    /**
     * Last sample of absolute orientation.
     */
    private Quaternion mLastOrientation = new Quaternion();
    
    /**
     * Variation of orientation respect to last sample.
     */
    private Quaternion mDeltaOrientation = new Quaternion();   
    
    /**
     * Constructor.
     */
    public AbsoluteOrientationConstantVelocityModelSlamCalibrator() {
        super(AbsoluteOrientationConstantVelocityModelSlamEstimator.
                CONTROL_LENGTH);
    }
    
    /**
     * Resets calibrator.
     */
    @Override
    public void reset() {
        super.reset();
        mLastOrientation = new Quaternion();
        mLastAngularSpeedX = mLastAngularSpeedY = mLastAngularSpeedZ = 0.0;
        mLastTimestampNanos = -1;
    }    

    /**
     * Obtains the number of state parameters in associated SLAM estimator.
     * @return number of state parameters.
     */
    @Override
    protected int getEstimatorStateLength() {
        return AbsoluteOrientationConstantVelocityModelSlamEstimator.
                STATE_LENGTH;
    }   
    
    /**
     * Gets a new instance containing calibration data estimated by this
     * calibrator.
     * @return a new calibration data instance.
     */            
    @Override
    public AbsoluteOrientationConstantVelocityModelSlamCalibrationData getCalibrationData() {
        AbsoluteOrientationConstantVelocityModelSlamCalibrationData result =
                new AbsoluteOrientationConstantVelocityModelSlamCalibrationData();
        getCalibrationData(result);
        return result;
    }    
    
    /**
     * Processes a full sample of accelerometer and gyroscope data to compute
     * statistics such as mean and covariance of variations.
     */
    @Override
    protected void processFullSample() {
        if(mListener != null) {
            mListener.onFullSampleReceived(this);
        }
        
        long timestamp = getMostRecentTimestampNanos();
        if(mLastTimestampNanos < 0) {
            //first time receiving control data we cannot determine its 
            //variation
            mLastOrientation.fromQuaternion(mAccumulatedOrientation);
            
            mLastAngularSpeedX = mAccumulatedAngularSpeedSampleX;
            mLastAngularSpeedY = mAccumulatedAngularSpeedSampleY;
            mLastAngularSpeedZ = mAccumulatedAngularSpeedSampleZ;
            
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
               
        mSample[0] = mDeltaOrientation.getA();
        mSample[1] = mDeltaOrientation.getB();
        mSample[2] = mDeltaOrientation.getC();
        mSample[3] = mDeltaOrientation.getD();                
        mSample[4] = mAccumulatedAccelerationSampleX * deltaTimestamp;
        mSample[5] = mAccumulatedAccelerationSampleY * deltaTimestamp;
        mSample[6] = mAccumulatedAccelerationSampleZ * deltaTimestamp;
        mSample[7] = deltaAngularSpeedX;
        mSample[8] = deltaAngularSpeedY;
        mSample[9] = deltaAngularSpeedZ;
        updateSample();
        
        mLastOrientation.combine(mDeltaOrientation);
        mLastAngularSpeedX = mAccumulatedAngularSpeedSampleX;
        mLastAngularSpeedY = mAccumulatedAngularSpeedSampleY;
        mLastAngularSpeedZ = mAccumulatedAngularSpeedSampleZ; 
        
        mLastTimestampNanos = timestamp;
        
        if(mListener != null) {
            mListener.onFullSampleProcessed(this);
        }
    }            
}
