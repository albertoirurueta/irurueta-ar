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
package com.irurueta.ar.sfm;

import com.irurueta.ar.SerializationHelper;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.BitSet;

import static org.junit.jupiter.api.Assertions.*;

class MatchedSamplesTest {

    @Test
    void testConstructor() {
        final var samples = new MatchedSamples();

        // check default values
        assertNull(samples.getSamples());
        assertNull(samples.getCameras());
        assertNull(samples.getViewIds());
        assertNull(samples.getReconstructedPoint());
        assertEquals(MatchedSamples.DEFAULT_QUALITY_SCORE, samples.getQualityScore(), 0.0);
        assertNull(samples.getInliers());
    }

    @Test
    void testGetSetSamples() {
        final var samples = new MatchedSamples();

        // check default value
        assertNull(samples.getSamples());

        // set new value
        final var s = new Sample2D[1];
        samples.setSamples(s);

        // check correctness
        assertSame(s, samples.getSamples());
    }

    @Test
    void testGetSetCameras() {
        final var samples = new MatchedSamples();

        // check default value
        assertNull(samples.getCameras());

        // set new value
        final var cams = new EstimatedCamera[1];
        samples.setCameras(cams);

        // check correctness
        assertSame(cams, samples.getCameras());
    }

    @Test
    void testGetSetViewIds() {
        final var samples = new MatchedSamples();

        // check default value
        assertNull(samples.getViewIds());

        // set new value
        final var ids = new int[1];
        samples.setViewIds(ids);

        // check correctness
        assertSame(ids, samples.getViewIds());
    }

    @Test
    void testGetSetReconstructedPoint() {
        final var samples = new MatchedSamples();

        // check default value
        assertNull(samples.getReconstructedPoint());

        // set new value
        var rp = new ReconstructedPoint3D();
        samples.setReconstructedPoint(rp);

        // check correctness
        assertSame(rp, samples.getReconstructedPoint());

        // set samples
        final var s = new Sample2D[1];
        s[0] = new Sample2D();
        samples.setSamples(s);

        // set new value again
        rp = new ReconstructedPoint3D();
        samples.setReconstructedPoint(rp);

        // check correctness
        assertSame(rp, samples.getReconstructedPoint());
        assertSame(rp, s[0].getReconstructedPoint());
    }

    @Test
    void testGetSetQualityScore() {
        final var samples = new MatchedSamples();

        // check default value
        assertEquals(MatchedSamples.DEFAULT_QUALITY_SCORE, samples.getQualityScore(), 0.0);

        // set new value
        samples.setQualityScore(20.0);

        // check correctness
        assertEquals(20.0, samples.getQualityScore(), 0.0);
    }

    @Test
    void testGetSetInliers() {
        final var samples = new MatchedSamples();

        // check default value
        assertNull(samples.getInliers());

        // set new value
        final var inliers = new BitSet();
        samples.setInliers(inliers);

        // check correctness
        assertSame(inliers, samples.getInliers());
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var samples1 = new MatchedSamples();

        // set new values
        final var s = new Sample2D[1];
        s[0] = new Sample2D();
        samples1.setSamples(s);
        final var cams = new EstimatedCamera[1];
        samples1.setCameras(cams);
        final var ids = new int[1];
        samples1.setViewIds(ids);
        final var rp = new ReconstructedPoint3D();
        samples1.setReconstructedPoint(rp);
        samples1.setQualityScore(20.0);
        final var inliers = new BitSet();
        samples1.setInliers(inliers);

        // check
        assertSame(s, samples1.getSamples());
        assertSame(cams, samples1.getCameras());
        assertSame(samples1.getViewIds(), ids);
        assertSame(samples1.getReconstructedPoint(), rp);
        assertEquals(20.0, samples1.getQualityScore(), 0.0);
        assertSame(samples1.getInliers(), inliers);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(samples1);
        final var samples2 = SerializationHelper.<MatchedSamples>deserialize(bytes);

        // check
        assertNotSame(samples1.getSamples(), samples2.getSamples());
        assertEquals(samples1.getSamples().length, samples2.getSamples().length);
        assertArrayEquals(samples1.getCameras(), samples2.getCameras());
        assertArrayEquals(samples1.getViewIds(), samples2.getViewIds());
        assertNotSame(samples1.getReconstructedPoint(), samples2.getReconstructedPoint());
        assertEquals(samples1.getQualityScore(), samples2.getQualityScore(), 0.0);
        assertEquals(samples1.getInliers(), samples2.getInliers());
    }
}
