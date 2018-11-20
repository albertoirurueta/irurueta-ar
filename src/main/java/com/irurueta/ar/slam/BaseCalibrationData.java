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
import com.irurueta.statistics.InvalidCovarianceMatrixException;
import com.irurueta.statistics.MultivariateNormalDist;

import java.io.Serializable;

/**
 * Contains control calibration data for a SLAM estimator during
 * Kalman filtering prediction stage.
 */
public abstract class BaseCalibrationData implements Serializable {
    
    /**
     * Length of control signal.
     */
    private int mControlLength;
    
    /**
     * Length of state in SLAM estimator.
     */
    private int mStateLength;
    
    /**
     * Control signal mean to correct biases in control signal.
     */
    private double[] mControlMean;
    
    /**
     * Control signal covariance to take into account for estimation of process 
     * noise during Kalman prediction stage.
     */
    private Matrix mControlCovariance;
    
    /**
     * Evaluator for distribution propagation.
     */
    private MultivariateNormalDist.JacobianEvaluator mEvaluator;
    
    /**
     * Constructor.
     * @param controlLength length of control signal.
     * @param stateLength length of state in SLAM estimator.
     * @throws IllegalArgumentException if provided length is not greater than 
     * zero.
     */
    public BaseCalibrationData(int controlLength, int stateLength) 
            throws IllegalArgumentException {
        if (controlLength < 1 || stateLength < 1) {
            throw new IllegalArgumentException(
                    "length must be greater than zero");
        }
        
        mControlLength = controlLength;
        mStateLength = stateLength;
    }
    
    /**
     * Gets length of control signal.
     * @return length of control signal.
     */
    public int getControlLength() {
        return mControlLength;
    }
    
    /**
     * Gets length of state in SLAM estimator.
     * @return length of state in SLAM estimator.
     */
    public int getStateLength() {
        return mStateLength;
    }
    
    /**
     * Gets control signal mean to correct biases in control signal.
     * @return control signal mean.
     */
    public double[] getControlMean() {
        return mControlMean;
    }
    
    /**
     * Sets control signal mean to correct biases in control signal.
     * @param controlMean control signal mean.
     * @throws IllegalArgumentException if provided array does not have expected 
     * length.
     */
    public void setControlMean(double[] controlMean) 
            throws IllegalArgumentException {
        if(controlMean.length != mControlLength) {
            throw new IllegalArgumentException("wrong mean length");
        }
        
        mControlMean = controlMean;
    }
    
    /**
     * Gets control signal covariance to take into account for estimation of 
     * process noise during Kalman prediction stage.
     * @return control signal covariance.
     */
    public Matrix getControlCovariance() {
        return mControlCovariance;
    }
    
    /**
     * Sets control signal covariance to take into account for estimation of
     * process noise during Kalman prediction stage.
     * @param controlCovariance control signal covariance.
     * @throws IllegalArgumentException if provided covariance size is wrong.
     */
    public void setControlCovariance(Matrix controlCovariance)
            throws IllegalArgumentException {
        if(controlCovariance.getRows() != mControlLength ||
                controlCovariance.getColumns() != mControlLength) {
            throw new IllegalArgumentException("wrong covariance size");
        }
        
        mControlCovariance = controlCovariance;
    }
    
    /**
     * Sets control signal mean and covariange to correct biases in control 
     * signal and to take into account for estimation process noise during 
     * Kalman prediction stage.
     * @param controlMean control signal mean.
     * @param controlCovariance control signal covariance.
     * @throws IllegalArgumentException if provided mean or covariance do not 
     * have proper size or length.
     */
    public void setControlMeanAndCovariance(double[] controlMean,
            Matrix controlCovariance) throws IllegalArgumentException {
        if(controlMean.length != mControlLength) {
            throw new IllegalArgumentException("wrong mean length");
        }
        if(controlCovariance.getRows() != mControlLength ||
                controlCovariance.getColumns() != mControlLength) {
            throw new IllegalArgumentException("wrong covariance size");
        }
        
        
        mControlMean = controlMean;        
        mControlCovariance = controlCovariance;
    }
    
    /**
     * Propagates calibrated control signal covariance using current control
     * jacobian matrix.
     * The propagated distribution can be used during prediction stage in Kalman 
     * filtering.
     * @param controlJacobian current control jacobian matrix.
     * @return propagated distribution.
     * @throws InvalidCovarianceMatrixException if estimated covariance is not 
     * valid.
     * @throws IllegalArgumentException if provided jacobian has invalid size.
     */
    public MultivariateNormalDist propagateWithControlJacobian(
            Matrix controlJacobian) throws InvalidCovarianceMatrixException,
            IllegalArgumentException {
        MultivariateNormalDist dist = new MultivariateNormalDist();
        propagateWithControlJacobian(controlJacobian, dist);
        return dist;
    }
    
    /**
     * Propagates calibrated control signal covariance using current control
     * jacobian matrix.
     * The propagated distribution can be used during prediction stage in Kalman
     * filtering.
     * @param controlJacobian current control jacobian matrix.
     * @param result instance where propagated distribution will be stored.
     * @throws InvalidCovarianceMatrixException if estimated covariance is not 
     * valid.
     * @throws IllegalArgumentException if provided jacobian has invalid size.
     */
    public void propagateWithControlJacobian(final Matrix controlJacobian,
            MultivariateNormalDist result) 
            throws InvalidCovarianceMatrixException, IllegalArgumentException {
        if(controlJacobian.getRows() != mStateLength ||
                controlJacobian.getColumns() != mControlLength) {
            throw new IllegalArgumentException("wrong control jacobian size");
        }
        
        if (mEvaluator == null) {
            mEvaluator = new MultivariateNormalDist.JacobianEvaluator() {
                @Override
                public void evaluate(double[] x, double[] y, Matrix jacobian) {
                    controlJacobian.copyTo(jacobian);
                }

                @Override
                public int getNumberOfVariables() {
                    return mStateLength;
                }
            };
        }
        
        try {
            MultivariateNormalDist.propagate(mEvaluator, mControlMean, 
                    mControlCovariance, result);
        } catch (WrongSizeException e) { 
            throw new InvalidCovarianceMatrixException(e);
        }
    }
}
