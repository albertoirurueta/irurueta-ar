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
 * using data from accelerometer, gyroscope and magnetic field.
 * This implementation of BaseSlamEstimator is an improvement respect
 * SlamEstimator to be able to take into account absolute orientation respect
 * Earth frame instead of just having a relative orientation respect to the
 * start time of the estimator.
 */
@SuppressWarnings("DuplicatedCode")
public class AbsoluteOrientationSlamEstimator extends
        AbsoluteOrientationBaseSlamEstimator<AbsoluteOrientationSlamCalibrationData> implements Serializable {

    /**
     * Internal state array length.
     */
    protected static final int STATE_LENGTH = 16;

    /**
     * Length of control array (changes in acceleration and angular speed).
     */
    protected static final int CONTROL_LENGTH = 13;

    /**
     * Length of position measurement, to correct any possible deviations of the
     * system after doing multiple predictions.
     */
    private static final int MEASUREMENT_LENGTH = 3;

    /**
     * Contains device status containing the following values: position-x,
     * position-y, position-z, quaternion-a, quaternion-b, quaternion-c,
     * quaternion-d, linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z.
     */
    private final double[] x;

    /**
     * Control signals containing the following values:
     * quaternion-change-a, quaternion-change-b, quaternion-change-c,
     * quaternion-change-d, linear-velocity-change-x, linear-velocity-change-y,
     * linear-velocity-change-z, linear-acceleration-change-x,
     * linear-acceleration-change-y, linear-acceleration-change-z,
     * angular-velocity-change-x, angular-velocity-change-y,
     * angular-velocity-change-z.
     */
    private final double[] u;

    /**
     * Jacobian respect x state during prediction (16x16).
     */
    private Matrix jacobianPredictionX;

    /**
     * Jacobian respect u control during state prediction (16x13).
     */
    private Matrix jacobianPredictionU;

    /**
     * Column matrix containing mU values to be passed as control values during
     * Kalman filter prediction.
     */
    private Matrix control;

    /**
     * Kalman's filter to remove effects of noise.
     */
    private KalmanFilter kalmanFilter;

    /**
     * Matrix of size 3x16 relating system status with obtained measures.
     * [1   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0]
     * [0   1   0   0   0   0   0   0   0   0   0   0   0   0   0   0]
     * [0   0   1   0   0   0   0   0   0   0   0   0   0   0   0   0]
     */
    private Matrix measurementMatrix;

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
    private Matrix measurement;

    /**
     * Last sample of absolute orientation.
     */
    private final Quaternion lastOrientation = new Quaternion();

    /**
     * Variation of orientation respect to last sample.
     */
    private final Quaternion deltaOrientation = new Quaternion();

    /**
     * Current state orientation.
     */
    private final Quaternion stateOrientation = new Quaternion();

    /**
     * Last sample of linear acceleration along x-axis.
     */
    private double lastAccelerationX;

    /**
     * Last sample of linear acceleration along y-axis.
     */
    private double lastAccelerationY;

    /**
     * Last sample of linear acceleration along z-axis.
     */
    private double lastAccelerationZ;

    /**
     * Last sample of angular speed along x-axis.
     */
    private double lastAngularSpeedX;

    /**
     * Last sample of angular speed along y-axis.
     */
    private double lastAngularSpeedY;

    /**
     * Last sample of angular speed along z-axis.
     */
    private double lastAngularSpeedZ;

    /**
     * Last timestamp of a full sample expressed in nanoseconds since the epoch
     * time.
     */
    private long lastTimestampNanos = -1;

    /**
     * Indicates whether a prediction has been made to initialize the internal
     * Kalman filter. Corrections can only be requested to Kalman filter once
     * a prediction has been made. Attempts to request corrections before having
     * a prediction will be ignored.
     */
    private boolean predictionAvailable;

    /**
     * Constructor.
     */
    public AbsoluteOrientationSlamEstimator() {
        super();
        x = new double[STATE_LENGTH];
        // initial value of quaternion.
        x[3] = 1.0;
        u = new double[CONTROL_LENGTH];
        try {
            jacobianPredictionX = new Matrix(STATE_LENGTH, STATE_LENGTH);
            jacobianPredictionU = new Matrix(STATE_LENGTH, CONTROL_LENGTH);
            control = new Matrix(CONTROL_LENGTH, 1);
            measurement = new Matrix(MEASUREMENT_LENGTH, 1);
            measurementMatrix = Matrix.identity(MEASUREMENT_LENGTH, STATE_LENGTH);

        } catch (final WrongSizeException ignore) {
            // never thrown
        }

        try {
            kalmanFilter = new KalmanFilter(STATE_LENGTH, MEASUREMENT_LENGTH, CONTROL_LENGTH);
            // setup matrix relating position measures with internal status.
            kalmanFilter.setMeasurementMatrix(measurementMatrix);
        } catch (final SignalProcessingException ignore) {
            // never thrown
        }

        try {
            // set initial Kalman filter state (state pre and pro must be two
            // different instances!)
            kalmanFilter.getStatePre().fromArray(x);
            kalmanFilter.getStatePost().fromArray(x);

        } catch (final WrongSizeException ignore) {
            // never thrown
        }
    }

    /**
     * Gets covariance matrix of state variables (position, velocity, acceleration, orientation and
     * angular speed).
     * Diagonal elements of matrix returned by this method are in the following order:
     * position-x, position-y, position-z, quaternion-a, quaternion-b, quaternion-c,
     * quaternion-d, linear-velocity-x, linear-velocity-y, linear-velocity-z,
     * linear-acceleration-x, linear-acceleration-y, linear-acceleration-z,
     * angular-velocity-x, angular-velocity-y, angular-velocity-z
     * Off-diagonal elements correspond to cross-correlation values of diagonal ones.
     *
     * @return covariance matrix of state variables.
     */
    @Override
    public Matrix getStateCovariance() {
        return kalmanFilter.getStatePre();
    }


    /**
     * Updates covariance matrix of position measures.
     * If null is provided, covariance matrix is not updated.
     *
     * @param positionCovariance new position covariance determining position
     *                           accuracy or null if last available covariance does not need to be
     *                           updated.
     * @throws IllegalArgumentException if provided covariance matrix is not
     *                                  3x3.
     */
    @Override
    public void setPositionCovarianceMatrix(final Matrix positionCovariance) {
        if (positionCovariance != null) {
            kalmanFilter.setMeasurementNoiseCov(positionCovariance);
        }
    }

    /**
     * Gets current covariance matrix of position measures determining current
     * accuracy of provided position measures.
     *
     * @return covariance matrix of position measures.
     */
    @Override
    public Matrix getPositionCovarianceMatrix() {
        return kalmanFilter.getMeasurementNoiseCov();
    }

    /**
     * Corrects system state with provided position measure using current
     * position accuracy.
     *
     * @param positionX new position along x-axis expressed in meters (m).
     * @param positionY new position along y-axis expressed in meters (m).
     * @param positionZ new position along z-axis expressed in meters (m).
     */
    @Override
    public void correctWithPositionMeasure(final double positionX, final double positionY, final double positionZ) {
        if (!predictionAvailable) {
            return;
        }

        if (listener != null) {
            listener.onCorrectWithPositionMeasure(this);
        }

        try {
            measurement.setElementAtIndex(0, positionX);
            measurement.setElementAtIndex(1, positionY);
            measurement.setElementAtIndex(2, positionZ);

            updateCorrectedState(kalmanFilter.correct(measurement));

            // copy corrected state to predicted state
            kalmanFilter.getStatePost().copyTo(kalmanFilter.getStatePre());
            kalmanFilter.getErrorCovPost().copyTo(kalmanFilter.getErrorCovPre());

        } catch (final SignalProcessingException e) {
            error = true;
        }

        if (listener != null) {
            listener.onCorrectedWithPositionMeasure(this);
        }
    }

    /**
     * Creates an instance of a calibrator to be used with this SLAM estimator.
     *
     * @return a calibrator.
     */
    public static AbsoluteOrientationSlamCalibrator createCalibrator() {
        return new AbsoluteOrientationSlamCalibrator();
    }

    /**
     * Processes a full sample (accelerometer + gyroscope) to update system
     * state.
     */
    @Override
    protected void processFullSample() {
        if (listener != null) {
            listener.onFullSampleReceived(this);
        }

        final var timestamp = getMostRecentTimestampNanos();
        if (lastTimestampNanos < 0) {
            // first time receiving control data, we just set linear acceleration
            // and angular speed into system state
            lastAccelerationX = stateAccelerationX = x[10] = accumulatedAccelerationSampleX;
            lastAccelerationY = stateAccelerationY = x[11] = accumulatedAccelerationSampleY;
            lastAccelerationZ = stateAccelerationZ = x[12] = accumulatedAccelerationSampleZ;
            lastAngularSpeedX = stateAngularSpeedX = x[13] = accumulatedAngularSpeedSampleX;
            lastAngularSpeedY = stateAngularSpeedY = x[14] = accumulatedAngularSpeedSampleY;
            lastAngularSpeedZ = stateAngularSpeedZ = x[15] = accumulatedAngularSpeedSampleZ;

            try {
                kalmanFilter.getStatePre().fromArray(x);
                kalmanFilter.getStatePost().fromArray(x);
            } catch (final WrongSizeException ignore) { /* never thrown */ }

            lastTimestampNanos = timestamp;

            if (listener != null) {
                listener.onFullSampleProcessed(this);
            }

            return;
        }

        accumulatedOrientation.normalize();

        lastOrientation.inverse(deltaOrientation);
        deltaOrientation.combine(accumulatedOrientation);
        deltaOrientation.normalize();

        final var deltaAccelerationX = accumulatedAccelerationSampleX - lastAccelerationX;
        final var deltaAccelerationY = accumulatedAccelerationSampleY - lastAccelerationY;
        final var deltaAccelerationZ = accumulatedAccelerationSampleZ - lastAccelerationZ;
        final var deltaAngularSpeedX = accumulatedAngularSpeedSampleX - lastAngularSpeedX;
        final var deltaAngularSpeedY = accumulatedAngularSpeedSampleY - lastAngularSpeedY;
        final var deltaAngularSpeedZ = accumulatedAngularSpeedSampleZ - lastAngularSpeedZ;
        final var deltaTimestamp = (timestamp - lastTimestampNanos) * NANOS_TO_SECONDS;

        // when a full sample is received, we update the data model
        u[0] = deltaOrientation.getA();
        u[1] = deltaOrientation.getB();
        u[2] = deltaOrientation.getC();
        u[3] = deltaOrientation.getD();
        // change in linear speed
        u[4] = u[5] = u[6] = 0.0;
        u[7] = deltaAccelerationX;
        u[8] = deltaAccelerationY;
        u[9] = deltaAccelerationZ;
        u[10] = deltaAngularSpeedX;
        u[11] = deltaAngularSpeedY;
        u[12] = deltaAngularSpeedZ;

        if (calibrationData != null && calibrationData.getControlMean() != null) {
            // if calibrator is available, remove mean to correct possible biases
            ArrayUtils.subtract(u, calibrationData.getControlMean(), u);
        }

        StatePredictor.predictWithRotationAdjustment(x, u, deltaTimestamp, x, jacobianPredictionX, jacobianPredictionU);

        // update Kalman filter transition matrix taking into account current
        // state
        kalmanFilter.setTransitionMatrix(jacobianPredictionX);

        // update control matrix from control vector jacobian
        kalmanFilter.setControlMatrix(jacobianPredictionU);

        if (calibrationData != null && calibrationData.getControlMean() != null
                && calibrationData.getControlCovariance() != null) {
            // if calibrator is available, propagate covariance to set process
            // covariance matrix
            if (normalDist == null) {
                normalDist = new MultivariateNormalDist(STATE_LENGTH);
            }

            try {
                calibrationData.propagateWithControlJacobian(jacobianPredictionU, normalDist);
                // update kalman filter process noise
                final var processNoise = kalmanFilter.getProcessNoiseCov();

                // copy normal dist covariance into processNoise
                normalDist.getCovariance(processNoise);
            } catch (final InvalidCovarianceMatrixException e) {
                // ignore
            }
        }

        try {
            // also predict the state using Kalman filter with current control
            // data
            control.fromArray(u, true);
            updatePredictedState(kalmanFilter.predict(control));

            // copy predicted state to corrected state
            kalmanFilter.getStatePre().copyTo(kalmanFilter.getStatePost());
            kalmanFilter.getErrorCovPre().copyTo(kalmanFilter.getErrorCovPost());

            predictionAvailable = true;
        } catch (final Exception e) {
            error = true;
        }

        lastOrientation.fromRotation(stateOrientation);
        lastAccelerationX = stateAccelerationX;
        lastAccelerationY = stateAccelerationY;
        lastAccelerationZ = stateAccelerationZ;
        lastAngularSpeedX = stateAngularSpeedX;
        lastAngularSpeedY = stateAngularSpeedY;
        lastAngularSpeedZ = stateAngularSpeedZ;
        lastTimestampNanos = timestamp;

        if (listener != null) {
            listener.onFullSampleProcessed(this);
        }
    }

    /**
     * Resets position, linear velocity, linear acceleration, orientation and
     * angular speed to provided values.
     * This method implementation also resets Kalman filter state.
     *
     * @param statePositionX     position along x-axis expressed in meters (m).
     * @param statePositionY     position along y-axis expressed in meters (m).
     * @param statePositionZ     position along z-axis expressed in meters (m).
     * @param stateVelocityX     linear velocity along x-axis expressed in meters
     *                           per second (m/s).
     * @param stateVelocityY     linear velocity along y-axis expressed in meters
     *                           per second (m/s).
     * @param stateVelocityZ     linear velocity along z-axis expressed in meters
     *                           per second (m/s).
     * @param stateAccelerationX linear acceleration along x-axis expressed in
     *                           meters per squared second (m/s^2).
     * @param stateAccelerationY linear acceleration along y-axis expressed in
     *                           meters per squared second (m/s^2).
     * @param stateAccelerationZ linear acceleration along z-axis expressed in
     *                           meters per squared second (m/s^2).
     * @param stateQuaternionA   A value of orientation quaternion.
     * @param stateQuaternionB   B value of orientation quaternion.
     * @param stateQuaternionC   C value of orientation quaternion.
     * @param stateQuaternionD   D value of orientation quaternion.
     * @param stateAngularSpeedX angular speed along x-axis expressed in radians
     *                           per second (rad/s).
     * @param stateAngularSpeedY angular speed along y-axis expressed in radians
     *                           per second (rad/s).
     * @param stateAngularSpeedZ angular speed along z-axis expressed in radians
     *                           per second (rad/s).
     */
    @Override
    protected void reset(
            final double statePositionX, final double statePositionY, final double statePositionZ,
            final double stateVelocityX, final double stateVelocityY, final double stateVelocityZ,
            final double stateAccelerationX, final double stateAccelerationY,
            final double stateAccelerationZ, final double stateQuaternionA, final double stateQuaternionB,
            final double stateQuaternionC, final double stateQuaternionD,
            final double stateAngularSpeedX, final double stateAngularSpeedY,
            final double stateAngularSpeedZ) {
        super.reset(statePositionX, statePositionY, statePositionZ,
                stateVelocityX, stateVelocityY, stateVelocityZ,
                stateAccelerationX, stateAccelerationY, stateAccelerationZ,
                stateQuaternionA, stateQuaternionB, stateQuaternionC, stateQuaternionD,
                stateAngularSpeedX, stateAngularSpeedY, stateAngularSpeedZ);
        //noinspection ConstantValue
        if (stateOrientation != null) {
            stateOrientation.setA(stateQuaternionA);
            stateOrientation.setB(stateQuaternionB);
            stateOrientation.setC(stateQuaternionC);
            stateOrientation.setD(stateQuaternionD);
        }

        //noinspection ConstantValue
        if (lastOrientation != null) {
            lastOrientation.setA(1.0);
            lastOrientation.setB(0.0);
            lastOrientation.setC(0.0);
            lastOrientation.setD(0.0);
        }

        if (x != null) {
            // position
            x[0] = statePositionX;
            x[1] = statePositionY;
            x[2] = statePositionZ;

            // quaternion
            x[3] = stateQuaternionA;
            x[4] = stateQuaternionB;
            x[5] = stateQuaternionC;
            x[6] = stateQuaternionD;

            // velocity
            x[7] = stateVelocityX;
            x[8] = stateVelocityY;
            x[9] = stateVelocityZ;

            // linear acceleration
            x[10] = stateAccelerationX;
            x[11] = stateAccelerationY;
            x[12] = stateAccelerationZ;

            // angular speed
            x[13] = stateAngularSpeedX;
            x[14] = stateAngularSpeedY;
            x[15] = stateAngularSpeedZ;

            try {
                // set initial Kalman filter state (state pre and pro must be two
                // different instances!)
                kalmanFilter.getStatePre().fromArray(x);
                kalmanFilter.getStatePost().fromArray(x);
            } catch (final WrongSizeException ignore) {
                // never thrown
            }
        }

        error = false;
        lastTimestampNanos = -1;
        predictionAvailable = false;
    }

    /**
     * Updates state data of the device by using state matrix obtained after
     * prediction from Kalman filter.
     * to ensure that state follows proper values (specially on quaternions),
     * we keep x values, which have been predicted using the state predictor,
     * which uses analytical values.
     * We then updated x using latest Kalman filter state for next iteration
     * on state predictor.
     *
     * @param state state matrix obtained from Kalman filter.
     */
    private void updatePredictedState(final Matrix state) {
        // position
        statePositionX = x[0];
        x[0] = state.getElementAtIndex(0);
        statePositionY = x[1];
        x[1] = state.getElementAtIndex(1);
        statePositionZ = x[2];
        x[2] = state.getElementAtIndex(2);

        // quaternion state predictor is more reliable than Kalman filter, for
        // that reason we ignore predicted quaternion values on Kalman filter and
        // simply keep predicted ones. Besides, typically gyroscope samples are
        // much more reliable than accelerometer ones. For that reason state
        // elements corresponding to quaternion (3 to 6) are not copied into mX
        // array.
        stateQuaternionA = x[3];
        stateQuaternionB = x[4];
        stateQuaternionC = x[5];
        stateQuaternionD = x[6];

        stateOrientation.setA(stateQuaternionA);
        stateOrientation.setB(stateQuaternionB);
        stateOrientation.setC(stateQuaternionC);
        stateOrientation.setD(stateQuaternionD);

        // velocity
        stateVelocityX = x[7];
        x[7] = state.getElementAtIndex(7);
        stateVelocityY = x[8];
        x[8] = state.getElementAtIndex(8);
        stateVelocityZ = x[9];
        x[9] = state.getElementAtIndex(9);

        // linear acceleration
        stateAccelerationX = x[10];
        x[10] = state.getElementAtIndex(10);
        stateAccelerationY = x[11];
        x[11] = state.getElementAtIndex(11);
        stateAccelerationZ = x[12];
        x[12] = state.getElementAtIndex(12);

        // angular velocity
        stateAngularSpeedX = x[13];
        x[13] = state.getElementAtIndex(13);
        stateAngularSpeedY = x[14];
        x[14] = state.getElementAtIndex(14);
        stateAngularSpeedZ = x[15];
        x[15] = state.getElementAtIndex(15);
    }

    /**
     * Updates state data of the device by using state matrix obtained from
     * Kalman filter after correction.
     *
     * @param state state matrix obtained from Kalman filter.
     */
    private void updateCorrectedState(final Matrix state) {
        // position
        statePositionX = x[0] = state.getElementAtIndex(0);
        statePositionY = x[1] = state.getElementAtIndex(1);
        statePositionZ = x[2] = state.getElementAtIndex(2);

        // quaternion
        stateQuaternionA = x[3] = state.getElementAtIndex(3);
        stateQuaternionB = x[4] = state.getElementAtIndex(4);
        stateQuaternionC = x[5] = state.getElementAtIndex(5);
        stateQuaternionD = x[6] = state.getElementAtIndex(6);

        stateOrientation.setA(stateQuaternionA);
        stateOrientation.setB(stateQuaternionB);
        stateOrientation.setC(stateQuaternionC);
        stateOrientation.setD(stateQuaternionD);
        stateOrientation.normalize();

        // velocity
        stateVelocityX = x[7] = state.getElementAtIndex(7);
        stateVelocityY = x[8] = state.getElementAtIndex(8);
        stateVelocityZ = x[9] = state.getElementAtIndex(9);

        // linear acceleration
        stateAccelerationX = x[10] = state.getElementAtIndex(10);
        stateAccelerationY = x[11] = state.getElementAtIndex(11);
        stateAccelerationZ = x[12] = state.getElementAtIndex(12);

        // angular velocity
        stateAngularSpeedX = x[13] = state.getElementAtIndex(13);
        stateAngularSpeedY = x[14] = state.getElementAtIndex(14);
        stateAngularSpeedZ = x[15] = state.getElementAtIndex(15);
    }
}
