/*
 * Copyright (C) 2015 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class RobustSinglePoint3DTriangulatorTest {

    @Test
    void testCreate() {
        // create with method

        // RANSAC
        var triangulator = RobustSinglePoint3DTriangulator.create(RobustEstimatorMethod.RANSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.RANSAC, triangulator.getMethod());
        assertInstanceOf(RANSACRobustSinglePoint3DTriangulator.class, triangulator);

        // LMedS
        triangulator = RobustSinglePoint3DTriangulator.create(RobustEstimatorMethod.LMEDS);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.LMEDS, triangulator.getMethod());
        assertInstanceOf(LMedSRobustSinglePoint3DTriangulator.class, triangulator);

        // MSAC
        triangulator = RobustSinglePoint3DTriangulator.create(RobustEstimatorMethod.MSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.MSAC, triangulator.getMethod());
        assertInstanceOf(MSACRobustSinglePoint3DTriangulator.class, triangulator);

        // PROSAC
        triangulator = RobustSinglePoint3DTriangulator.create(RobustEstimatorMethod.PROSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertInstanceOf(PROSACRobustSinglePoint3DTriangulator.class, triangulator);

        // PROMedS
        triangulator = RobustSinglePoint3DTriangulator.create(RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertInstanceOf(PROMedSRobustSinglePoint3DTriangulator.class, triangulator);

        // create with points, cameras and methods

        final var points = new ArrayList<Point2D>();
        points.add(Point2D.create());
        points.add(Point2D.create());

        final var cameras = new ArrayList<PinholeCamera>();
        cameras.add(new PinholeCamera());
        cameras.add(new PinholeCamera());

        // RANSAC
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, RobustEstimatorMethod.RANSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.RANSAC, triangulator.getMethod());
        assertInstanceOf(RANSACRobustSinglePoint3DTriangulator.class, triangulator);

        // LMedS
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, RobustEstimatorMethod.LMEDS);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.LMEDS, triangulator.getMethod());
        assertInstanceOf(LMedSRobustSinglePoint3DTriangulator.class, triangulator);

        // MSAC
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, RobustEstimatorMethod.MSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.MSAC, triangulator.getMethod());
        assertInstanceOf(MSACRobustSinglePoint3DTriangulator.class, triangulator);

        // PROSAC
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, RobustEstimatorMethod.PROSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertInstanceOf(PROSACRobustSinglePoint3DTriangulator.class, triangulator);

        // PROMedS
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertInstanceOf(PROMedSRobustSinglePoint3DTriangulator.class, triangulator);

        // create with points, cameras, quality scores and method
        final var qualityScores = new double[2];

        // RANSAC
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, qualityScores,
                RobustEstimatorMethod.RANSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.RANSAC, triangulator.getMethod());
        assertInstanceOf(RANSACRobustSinglePoint3DTriangulator.class, triangulator);

        // LMedS
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, qualityScores,
                RobustEstimatorMethod.LMEDS);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.LMEDS, triangulator.getMethod());
        assertInstanceOf(LMedSRobustSinglePoint3DTriangulator.class, triangulator);

        // MSAC
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, qualityScores,
                RobustEstimatorMethod.MSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.MSAC, triangulator.getMethod());
        assertInstanceOf(MSACRobustSinglePoint3DTriangulator.class, triangulator);

        // PROSAC
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, qualityScores,
                RobustEstimatorMethod.PROSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertInstanceOf(PROSACRobustSinglePoint3DTriangulator.class, triangulator);

        // PROMedS
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, qualityScores,
                RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertInstanceOf(PROMedSRobustSinglePoint3DTriangulator.class, triangulator);

        // test without arguments
        triangulator = RobustSinglePoint3DTriangulator.create();

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_ROBUST_METHOD, triangulator.getMethod());
        assertInstanceOf(PROMedSRobustSinglePoint3DTriangulator.class, triangulator);

        // test with points and cameras
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_ROBUST_METHOD, triangulator.getMethod());
        assertInstanceOf(PROMedSRobustSinglePoint3DTriangulator.class, triangulator);

        // test with points, cameras and quality scores
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, qualityScores);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA, triangulator.getProgressDelta(),
                0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE, triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS, triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_ROBUST_METHOD, triangulator.getMethod());
        assertInstanceOf(PROMedSRobustSinglePoint3DTriangulator.class, triangulator);
    }
}
