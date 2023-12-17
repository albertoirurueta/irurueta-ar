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
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class RobustSinglePoint3DTriangulatorTest {

    @Test
    public void testCreate() {
        // create with method

        // RANSAC
        RobustSinglePoint3DTriangulator triangulator = RobustSinglePoint3DTriangulator.create(
                RobustEstimatorMethod.RANSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.RANSAC, triangulator.getMethod());
        assertTrue(triangulator instanceof RANSACRobustSinglePoint3DTriangulator);

        // LMedS
        triangulator = RobustSinglePoint3DTriangulator.create(RobustEstimatorMethod.LMEDS);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.LMEDS, triangulator.getMethod());
        assertTrue(triangulator instanceof LMedSRobustSinglePoint3DTriangulator);

        // MSAC
        triangulator = RobustSinglePoint3DTriangulator.create(RobustEstimatorMethod.MSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.MSAC, triangulator.getMethod());
        assertTrue(triangulator instanceof MSACRobustSinglePoint3DTriangulator);

        // PROSAC
        triangulator = RobustSinglePoint3DTriangulator.create(RobustEstimatorMethod.PROSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertTrue(triangulator instanceof PROSACRobustSinglePoint3DTriangulator);

        // PROMedS
        triangulator = RobustSinglePoint3DTriangulator.create(RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertTrue(triangulator instanceof PROMedSRobustSinglePoint3DTriangulator);

        // create with points, cameras and methods

        final List<Point2D> points = new ArrayList<>();
        points.add(Point2D.create());
        points.add(Point2D.create());

        final List<PinholeCamera> cameras = new ArrayList<>();
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
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(triangulator.getPoints2D(), points);
        assertSame(triangulator.getCameras(), cameras);
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.RANSAC, triangulator.getMethod());
        assertTrue(triangulator instanceof RANSACRobustSinglePoint3DTriangulator);

        // LMedS
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, RobustEstimatorMethod.LMEDS);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.LMEDS, triangulator.getMethod());
        assertTrue(triangulator instanceof LMedSRobustSinglePoint3DTriangulator);

        // MSAC
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras,
                RobustEstimatorMethod.MSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.MSAC, triangulator.getMethod());
        assertTrue(triangulator instanceof MSACRobustSinglePoint3DTriangulator);

        // PROSAC
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, RobustEstimatorMethod.PROSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertTrue(triangulator instanceof PROSACRobustSinglePoint3DTriangulator);

        // PROMedS
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertTrue(triangulator instanceof PROMedSRobustSinglePoint3DTriangulator);

        // create with points, cameras, quality scores and method
        final double[] qualityScores = new double[2];

        // RANSAC
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras,
                qualityScores, RobustEstimatorMethod.RANSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.RANSAC, triangulator.getMethod());
        assertTrue(triangulator instanceof RANSACRobustSinglePoint3DTriangulator);

        // LMedS
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras,
                qualityScores, RobustEstimatorMethod.LMEDS);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.LMEDS, triangulator.getMethod());
        assertTrue(triangulator instanceof LMedSRobustSinglePoint3DTriangulator);

        // MSAC
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras,
                qualityScores, RobustEstimatorMethod.MSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.MSAC, triangulator.getMethod());
        assertTrue(triangulator instanceof MSACRobustSinglePoint3DTriangulator);

        // PROSAC
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras,
                qualityScores, RobustEstimatorMethod.PROSAC);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.PROSAC, triangulator.getMethod());
        assertTrue(triangulator instanceof PROSACRobustSinglePoint3DTriangulator);

        // PROMedS
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras,
                qualityScores, RobustEstimatorMethod.PROMEDS);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustEstimatorMethod.PROMEDS, triangulator.getMethod());
        assertTrue(triangulator instanceof PROMedSRobustSinglePoint3DTriangulator);

        // test without arguments
        triangulator = RobustSinglePoint3DTriangulator.create();

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertNull(triangulator.getPoints2D());
        assertNull(triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_ROBUST_METHOD,
                triangulator.getMethod());
        assertTrue(triangulator instanceof PROMedSRobustSinglePoint3DTriangulator);

        // test with points and cameras
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertNull(triangulator.getQualityScores());
        assertFalse(triangulator.isReady());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_ROBUST_METHOD,
                triangulator.getMethod());
        assertTrue(triangulator instanceof PROMedSRobustSinglePoint3DTriangulator);

        // test with points, cameras and quality scores
        triangulator = RobustSinglePoint3DTriangulator.create(points, cameras, qualityScores);

        // check correctness
        assertNull(triangulator.getListener());
        assertFalse(triangulator.isListenerAvailable());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION,
                triangulator.isUseHomogeneousSolution());
        assertFalse(triangulator.isLocked());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_PROGRESS_DELTA,
                triangulator.getProgressDelta(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE,
                triangulator.getConfidence(), 0.0);
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS,
                triangulator.getMaxIterations());
        assertSame(points, triangulator.getPoints2D());
        assertSame(cameras, triangulator.getCameras());
        assertSame(qualityScores, triangulator.getQualityScores());
        assertTrue(triangulator.isReady());
        assertEquals(RobustSinglePoint3DTriangulator.DEFAULT_ROBUST_METHOD,
                triangulator.getMethod());
        assertTrue(triangulator instanceof PROMedSRobustSinglePoint3DTriangulator);
    }
}
