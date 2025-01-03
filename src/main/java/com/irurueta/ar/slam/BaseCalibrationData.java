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
    private final int controlLength;

    /**
     * Length of state in SLAM estimator.
     */
    private final int stateLength;

    /**
     * Control signal mean to correct biases in control signal.
     */
    private double[] controlMean;

    /**
     * Control signal covariance to take into account for estimation of process
     * noise during Kalman prediction stage.
     */
    private Matrix controlCovariance;

    /**
     * Evaluator for distribution propagation.
     */
    private transient MultivariateNormalDist.JacobianEvaluator evaluator;

    /**
     * Constructor.
     *
     * @param controlLength length of control signal.
     * @param stateLength   length of state in SLAM estimator.
     * @throws IllegalArgumentException if provided length is not greater than
     *                                  zero.
     */
    protected BaseCalibrationData(final int controlLength, final int stateLength) {
        if (controlLength < 1 || stateLength < 1) {
            throw new IllegalArgumentException("length must be greater than zero");
        }

        this.controlLength = controlLength;
        this.stateLength = stateLength;
    }

    /**
     * Gets length of control signal.
     *
     * @return length of control signal.
     */
    public int getControlLength() {
        return controlLength;
    }

    /**
     * Gets length of state in SLAM estimator.
     *
     * @return length of state in SLAM estimator.
     */
    public int getStateLength() {
        return stateLength;
    }

    /**
     * Gets control signal mean to correct biases in control signal.
     *
     * @return control signal mean.
     */
    public double[] getControlMean() {
        return controlMean;
    }

    /**
     * Sets control signal mean to correct biases in control signal.
     *
     * @param controlMean control signal mean.
     * @throws IllegalArgumentException if provided array does not have expected
     *                                  length.
     */
    public void setControlMean(final double[] controlMean) {
        if (controlMean.length != controlLength) {
            throw new IllegalArgumentException("wrong mean length");
        }

        this.controlMean = controlMean;
    }

    /**
     * Gets control signal covariance to take into account for estimation of
     * process noise during Kalman prediction stage.
     *
     * @return control signal covariance.
     */
    public Matrix getControlCovariance() {
        return controlCovariance;
    }

    /**
     * Sets control signal covariance to take into account for estimation of
     * process noise during Kalman prediction stage.
     *
     * @param controlCovariance control signal covariance.
     * @throws IllegalArgumentException if provided covariance size is wrong.
     */
    public void setControlCovariance(final Matrix controlCovariance) {
        if (controlCovariance.getRows() != controlLength || controlCovariance.getColumns() != controlLength) {
            throw new IllegalArgumentException("wrong covariance size");
        }

        this.controlCovariance = controlCovariance;
    }

    /**
     * Sets control signal mean and covariance to correct biases in control
     * signal and to take into account for estimation process noise during
     * Kalman prediction stage.
     *
     * @param controlMean       control signal mean.
     * @param controlCovariance control signal covariance.
     * @throws IllegalArgumentException if provided mean or covariance do not
     *                                  have proper size or length.
     */
    public void setControlMeanAndCovariance(final double[] controlMean, final Matrix controlCovariance) {
        if (controlMean.length != controlLength) {
            throw new IllegalArgumentException("wrong mean length");
        }
        if (controlCovariance.getRows() != controlLength || controlCovariance.getColumns() != controlLength) {
            throw new IllegalArgumentException("wrong covariance size");
        }

        this.controlMean = controlMean;
        this.controlCovariance = controlCovariance;
    }

    /**
     * Propagates calibrated control signal covariance using current control
     * jacobian matrix.
     * The propagated distribution can be used during prediction stage in Kalman
     * filtering.
     *
     * @param controlJacobian current control jacobian matrix.
     * @return propagated distribution.
     * @throws InvalidCovarianceMatrixException if estimated covariance is not
     *                                          valid.
     * @throws IllegalArgumentException         if provided jacobian has invalid size.
     */
    public MultivariateNormalDist propagateWithControlJacobian(final Matrix controlJacobian)
            throws InvalidCovarianceMatrixException {
        final var dist = new MultivariateNormalDist();
        propagateWithControlJacobian(controlJacobian, dist);
        return dist;
    }

    /**
     * Propagates calibrated control signal covariance using current control
     * jacobian matrix.
     * The propagated distribution can be used during prediction stage in Kalman
     * filtering.
     *
     * @param controlJacobian current control jacobian matrix.
     * @param result          instance where propagated distribution will be stored.
     * @throws InvalidCovarianceMatrixException if estimated covariance is not
     *                                          valid.
     * @throws IllegalArgumentException         if provided jacobian has invalid size.
     */
    public void propagateWithControlJacobian(
            final Matrix controlJacobian, final MultivariateNormalDist result) throws InvalidCovarianceMatrixException {
        if (controlJacobian.getRows() != stateLength || controlJacobian.getColumns() != controlLength) {
            throw new IllegalArgumentException("wrong control jacobian size");
        }

        if (evaluator == null) {
            evaluator = new MultivariateNormalDist.JacobianEvaluator() {
                @Override
                public void evaluate(final double[] x, final double[] y, final Matrix jacobian) {
                    controlJacobian.copyTo(jacobian);
                }

                @Override
                public int getNumberOfVariables() {
                    return stateLength;
                }
            };
        }

        try {
            MultivariateNormalDist.propagate(evaluator, controlMean, controlCovariance, result);
        } catch (final WrongSizeException e) {
            throw new InvalidCovarianceMatrixException(e);
        }
    }
}
