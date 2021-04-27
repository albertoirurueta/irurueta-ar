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
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class ConstantVelocityModelSlamCalibrationDataTest {

    public static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void testConstructorGetControlLengthAndGetStateLength() {
        final ConstantVelocityModelSlamCalibrationData data =
                new ConstantVelocityModelSlamCalibrationData();

        // check initial values
        assertEquals(data.getControlLength(),
                ConstantVelocityModelSlamEstimator.CONTROL_LENGTH);
        assertEquals(data.getStateLength(),
                ConstantVelocityModelSlamEstimator.STATE_LENGTH);
        assertNull(data.getControlMean());
        assertNull(data.getControlCovariance());
    }

    @Test
    public void testGetSetControlMean() {
        final ConstantVelocityModelSlamCalibrationData data =
                new ConstantVelocityModelSlamCalibrationData();

        // check initial value
        assertNull(data.getControlMean());

        // set new value
        final double[] mean = new double[
                ConstantVelocityModelSlamEstimator.CONTROL_LENGTH];
        data.setControlMean(mean);

        // check correctness
        assertSame(data.getControlMean(), mean);

        // Force IllegalArgumentException
        final double[] wrong = new double[1];
        try {
            data.setControlMean(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetControlCovariance() throws WrongSizeException {
        final ConstantVelocityModelSlamCalibrationData data =
                new ConstantVelocityModelSlamCalibrationData();

        // check initial value
        assertNull(data.getControlCovariance());

        // set new value
        final Matrix cov = new Matrix(
                ConstantVelocityModelSlamEstimator.CONTROL_LENGTH,
                ConstantVelocityModelSlamEstimator.CONTROL_LENGTH);
        data.setControlCovariance(cov);

        // check correctness
        assertSame(data.getControlCovariance(), cov);

        // Force IllegalArgumentException
        final Matrix wrong = new Matrix(1, 1);
        try {
            data.setControlCovariance(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSetControlMeanAndCovariance() throws WrongSizeException {
        final ConstantVelocityModelSlamCalibrationData data =
                new ConstantVelocityModelSlamCalibrationData();

        // set new values
        final double[] mean = new double[
                ConstantVelocityModelSlamEstimator.CONTROL_LENGTH];
        final Matrix cov = new Matrix(
                ConstantVelocityModelSlamEstimator.CONTROL_LENGTH,
                ConstantVelocityModelSlamEstimator.CONTROL_LENGTH);
        data.setControlMeanAndCovariance(mean, cov);

        // check correctness
        assertSame(data.getControlMean(), mean);
        assertSame(data.getControlCovariance(), cov);

        // Force IllegalArgumentException
        final double[] wrongMean = new double[1];
        final Matrix wrongCov = new Matrix(1, 1);

        try {
            data.setControlMeanAndCovariance(wrongMean, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            data.setControlMeanAndCovariance(mean, wrongCov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testPropagateWithControlJacobian() throws WrongSizeException,
            InvalidCovarianceMatrixException {

        final Matrix cov = Matrix.identity(
                ConstantVelocityModelSlamEstimator.CONTROL_LENGTH,
                ConstantVelocityModelSlamEstimator.CONTROL_LENGTH).
                multiplyByScalarAndReturnNew(1e-3);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] mean = new double[
                ConstantVelocityModelSlamEstimator.CONTROL_LENGTH];
        randomizer.fill(mean);

        final ConstantVelocityModelSlamCalibrationData data =
                new ConstantVelocityModelSlamCalibrationData();
        data.setControlMeanAndCovariance(mean, cov);

        final Matrix jacobian = Matrix.identity(
                ConstantVelocityModelSlamEstimator.STATE_LENGTH,
                ConstantVelocityModelSlamEstimator.CONTROL_LENGTH).
                multiplyByScalarAndReturnNew(2.0);

        final MultivariateNormalDist dist = data.propagateWithControlJacobian(
                jacobian);
        final MultivariateNormalDist dist2 = new MultivariateNormalDist();
        data.propagateWithControlJacobian(jacobian, dist2);

        // check correctness
        final Matrix propagatedCov = jacobian.multiplyAndReturnNew(cov).
                multiplyAndReturnNew(jacobian.transposeAndReturnNew());

        assertTrue(dist.getCovariance().equals(propagatedCov, ABSOLUTE_ERROR));
        assertTrue(dist2.getCovariance().equals(propagatedCov, ABSOLUTE_ERROR));

        assertArrayEquals(dist.getMean(),
                new double[ConstantVelocityModelSlamEstimator.STATE_LENGTH],
                0.0);
        assertArrayEquals(dist2.getMean(),
                new double[ConstantVelocityModelSlamEstimator.STATE_LENGTH],
                0.0);
    }
}
