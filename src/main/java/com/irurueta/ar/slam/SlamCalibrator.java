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

import java.io.Serializable;

/**
 * Processes data to estimate calibration for SLAM estimator.
 * This class must be used while gathering data for a system being kept constant
 * (no motion).
 */
public class SlamCalibrator extends BaseSlamCalibrator<SlamCalibrationData> 
        implements Serializable {
    
    /**
     * Last sample of linear acceleration along x-axis.
     */
    private double mLastAccelerationX;
    
    /**
     * Last sample of linear acceleration along y-axis.
     */
    private double mLastAccelerationY;
    
    /**
     * Last sample of linear acceleration along z-axis.
     */
    private double mLastAccelerationZ;
    
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
     * Constructor.
     */
    public SlamCalibrator() {
        super(SlamEstimator.CONTROL_LENGTH);
    }
    
    /**
     * Resets calibrator.
     */
    @Override
    public void reset() {
        super.reset();
        mLastAccelerationX = mLastAccelerationY = mLastAccelerationZ =
                mLastAngularSpeedX = mLastAngularSpeedY = mLastAngularSpeedZ = 
                0.0;
        mLastTimestampNanos = -1;
    }  
    
    /**
     * Obtains the number of state parameters in associated SLAM estimator.
     * @return number of state parameters.
     */
    @Override
    protected int getEstimatorStateLength() {
        return SlamEstimator.STATE_LENGTH;
    }
    
    /**
     * Gets a new instance containing calibration data estimated by this
     * calibrator.
     * @return a new calibration data instance.
     */    
    @Override
    public SlamCalibrationData getCalibrationData() {
        SlamCalibrationData result = new SlamCalibrationData();
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
            mLastAccelerationX = mAccumulatedAccelerationSampleX;
            mLastAccelerationY = mAccumulatedAccelerationSampleY;
            mLastAccelerationZ = mAccumulatedAccelerationSampleZ;
            
            mLastAngularSpeedX = mAccumulatedAngularSpeedSampleX;
            mLastAngularSpeedY = mAccumulatedAngularSpeedSampleY;
            mLastAngularSpeedZ = mAccumulatedAngularSpeedSampleZ;
            
            mLastTimestampNanos = timestamp;
            
            if(mListener != null) {
                mListener.onFullSampleProcessed(this);
            }
            
            return;
        }
        
        double deltaAccelerationX = mAccumulatedAccelerationSampleX - 
                mLastAccelerationX;
        double deltaAccelerationY = mAccumulatedAccelerationSampleY -
                mLastAccelerationY;
        double deltaAccelerationZ = mAccumulatedAccelerationSampleZ -
                mLastAccelerationZ;
        double deltaAngularSpeedX = mAccumulatedAngularSpeedSampleX -
                mLastAngularSpeedX;
        double deltaAngularSpeedY = mAccumulatedAngularSpeedSampleY -
                mLastAngularSpeedY;
        double deltaAngularSpeedZ = mAccumulatedAngularSpeedSampleZ -
                mLastAngularSpeedZ;
        
        mSample[0] = mSample[1] = mSample[2] = 0.0;
        mSample[3] = deltaAccelerationX;
        mSample[4] = deltaAccelerationY;
        mSample[5] = deltaAccelerationZ;
        mSample[6] = deltaAngularSpeedX;
        mSample[7] = deltaAngularSpeedY;
        mSample[8] = deltaAngularSpeedZ;
        updateSample();
        
        mLastAccelerationX = mAccumulatedAccelerationSampleX;
        mLastAccelerationY = mAccumulatedAccelerationSampleY;
        mLastAccelerationZ = mAccumulatedAccelerationSampleZ;
        
        mLastAngularSpeedX = mAccumulatedAngularSpeedSampleX;
        mLastAngularSpeedY = mAccumulatedAngularSpeedSampleY;
        mLastAngularSpeedZ = mAccumulatedAngularSpeedSampleZ;        

        mLastTimestampNanos = timestamp;
        
        if(mListener != null) {
            mListener.onFullSampleProcessed(this);
        }
    }    
}
