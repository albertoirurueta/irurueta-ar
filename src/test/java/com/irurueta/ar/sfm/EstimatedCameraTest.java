/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.ar.sfm;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.PinholeCamera;
import org.junit.Test;

import static org.junit.Assert.*;

public class EstimatedCameraTest {

    @Test
    public void testConstructor() {
        final EstimatedCamera ec = new EstimatedCamera();

        // check default values
        assertNull(ec.getId());
        assertEquals(ec.getViewId(), 0);
        assertNull(ec.getCamera());
        assertEquals(ec.getQualityScore(),
                EstimatedCamera.DEFAULT_QUALITY_SCORE, 0.0);
        assertNull(ec.getCovariance());
    }

    @Test
    public void testGetSetId() {
        final EstimatedCamera ec = new EstimatedCamera();

        // check default value
        assertNull(ec.getId());

        // set new value
        ec.setId("id");

        // check correctness
        assertEquals(ec.getId(), "id");
    }

    @Test
    public void testGetSetViewId() {
        final EstimatedCamera ec = new EstimatedCamera();

        // check default value
        assertEquals(ec.getViewId(), 0);

        // set new value
        ec.setViewId(1);

        // check
        assertEquals(ec.getViewId(), 1);
    }

    @Test
    public void testGetSetCamera() {
        final EstimatedCamera ec = new EstimatedCamera();

        // check default value
        assertNull(ec.getCamera());

        // set new value
        final PinholeCamera camera = new PinholeCamera();
        ec.setCamera(camera);

        // check correctness
        assertSame(ec.getCamera(), camera);
    }

    @Test
    public void testGetSetQualityScore() {
        final EstimatedCamera ec = new EstimatedCamera();

        // check default value
        assertEquals(ec.getQualityScore(),
                EstimatedCamera.DEFAULT_QUALITY_SCORE, 0.0);

        // set new value
        ec.setQualityScore(5.0);

        // check correctness
        assertEquals(ec.getQualityScore(), 5.0, 0.0);
    }

    @Test
    public void testGetSetCovariance() throws WrongSizeException {
        final EstimatedCamera ec = new EstimatedCamera();

        // check default value
        assertNull(ec.getCovariance());

        // set new value
        final Matrix cov = new Matrix(12, 12);
        ec.setCovariance(cov);

        // check correctness
        assertSame(ec.getCovariance(), cov);
    }
}
