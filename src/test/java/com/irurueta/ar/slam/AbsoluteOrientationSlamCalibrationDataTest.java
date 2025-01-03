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
import com.irurueta.ar.SerializationHelper;
import com.irurueta.statistics.InvalidCovarianceMatrixException;
import com.irurueta.statistics.MultivariateNormalDist;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class AbsoluteOrientationSlamCalibrationDataTest {

    public static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    void testConstructorGetControlLengthAndGetStateLength() {
        final var data = new AbsoluteOrientationSlamCalibrationData();

        // check initial values
        assertEquals(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH, data.getControlLength());
        assertEquals(AbsoluteOrientationSlamEstimator.STATE_LENGTH, data.getStateLength());
        assertNull(data.getControlMean());
        assertNull(data.getControlCovariance());
    }

    @Test
    void testGetSetControlMean() {
        final var data = new AbsoluteOrientationSlamCalibrationData();

        // check initial value
        assertNull(data.getControlMean());

        // set new value
        final var mean = new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH];
        data.setControlMean(mean);

        // check correctness
        assertSame(mean, data.getControlMean());

        // Force IllegalArgumentException
        final var wrong = new double[1];
        assertThrows(IllegalArgumentException.class, () -> data.setControlMean(wrong));
    }

    @Test
    void testGetSetControlCovariance() throws WrongSizeException {
        final var data = new AbsoluteOrientationSlamCalibrationData();

        // check initial value
        assertNull(data.getControlCovariance());

        // set new value
        final var cov = new Matrix(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH);
        data.setControlCovariance(cov);

        // check correctness
        assertSame(cov, data.getControlCovariance());

        // Force IllegalArgumentException
        final var wrong = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> data.setControlCovariance(wrong));
    }

    @Test
    void testSetControlMeanAndCovariance() throws WrongSizeException {
        final var data = new AbsoluteOrientationSlamCalibrationData();

        // set new values
        final var mean = new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH];
        final var cov = new Matrix(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH);
        data.setControlMeanAndCovariance(mean, cov);

        // check correctness
        assertSame(mean, data.getControlMean());
        assertSame(cov, data.getControlCovariance());

        // Force IllegalArgumentException
        final var wrongMean = new double[1];
        final var wrongCov = new Matrix(1, 1);

        assertThrows(IllegalArgumentException.class, () -> data.setControlMeanAndCovariance(wrongMean, cov));
        assertThrows(IllegalArgumentException.class, () -> data.setControlMeanAndCovariance(mean, wrongCov));
    }


    @Test
    void testPropagateWithControlJacobian() throws WrongSizeException, InvalidCovarianceMatrixException {

        final var cov = Matrix.identity(
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH).
                multiplyByScalarAndReturnNew(1e-3);

        final var randomizer = new UniformRandomizer();
        final var mean = new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH];
        randomizer.fill(mean);

        final var data = new AbsoluteOrientationSlamCalibrationData();
        data.setControlMeanAndCovariance(mean, cov);

        final var jacobian = Matrix.identity(
                AbsoluteOrientationSlamEstimator.STATE_LENGTH,
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH).
                multiplyByScalarAndReturnNew(2.0);

        final var dist = data.propagateWithControlJacobian(jacobian);
        final var dist2 = new MultivariateNormalDist();
        data.propagateWithControlJacobian(jacobian, dist2);

        // check correctness
        final var propagatedCov = jacobian.multiplyAndReturnNew(cov).multiplyAndReturnNew(
                jacobian.transposeAndReturnNew());

        assertTrue(dist.getCovariance().equals(propagatedCov, ABSOLUTE_ERROR));
        assertTrue(dist2.getCovariance().equals(propagatedCov, ABSOLUTE_ERROR));

        assertArrayEquals(new double[AbsoluteOrientationSlamEstimator.STATE_LENGTH],
                dist.getMean(), 0.0);
        assertArrayEquals(new double[AbsoluteOrientationSlamEstimator.STATE_LENGTH],
                dist2.getMean(), 0.0);
    }

    @Test
    void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var data1 = new AbsoluteOrientationSlamCalibrationData();

        // set new values
        final var randomizer = new UniformRandomizer();

        final var mean = new double[AbsoluteOrientationSlamEstimator.CONTROL_LENGTH];
        randomizer.fill(mean);
        data1.setControlMean(mean);
        final var cov = new Matrix(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationSlamEstimator.CONTROL_LENGTH);
        data1.setControlCovariance(cov);

        // check
        assertSame(data1.getControlMean(), mean);
        assertSame(data1.getControlCovariance(), cov);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(data1);
        final var data2 = SerializationHelper.<AbsoluteOrientationSlamCalibrationData>deserialize(bytes);

        // check
        assertArrayEquals(data1.getControlMean(), data2.getControlMean(), 0.0);
        assertEquals(data1.getControlCovariance(), data2.getControlCovariance());
    }
}
