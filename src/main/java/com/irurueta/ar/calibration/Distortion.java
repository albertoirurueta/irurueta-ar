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
package com.irurueta.ar.calibration;

import com.irurueta.geometry.NotSupportedException;
import com.irurueta.geometry.Point2D;

import java.util.ArrayList;
import java.util.List;

/**
 * This class accounts for any possible distortion that might occur on 2D
 * points. This might occur for instance when using cameras that produce
 * additional distortion (such as radial distortion) not modelled in the general
 * camera model.
 * This class is abstract, specific implementations for each kind of supported
 * distortion exists.
 */
public abstract class Distortion {

    /**
     * Distorts provided 2D point.
     *
     * @param undistortedPoint undistorted point to be distorted.
     * @return distorted point.
     * @throws NotSupportedException raised if distortion implementation does
     *                               not support distorting points.
     * @throws DistortionException   raised if distortion computation failed.
     */
    public Point2D distort(final Point2D undistortedPoint) throws NotSupportedException, DistortionException {
        final var distortedPoint = Point2D.create();
        distort(undistortedPoint, distortedPoint);
        return distortedPoint;
    }

    /**
     * Distorts provided 2D points.
     *
     * @param undistortedPoints list of undistorted points to be distorted.
     * @return distorted points.
     * @throws NotSupportedException raised if distortion implementation does
     *                               not support distorting points.
     * @throws DistortionException   raised if distortion computation failed.
     */
    public List<Point2D> distort(final List<Point2D> undistortedPoints) throws NotSupportedException,
            DistortionException {
        final var size = undistortedPoints.size();
        final var distortedPoints = new ArrayList<Point2D>(size);
        for (var i = 0; i < size; i++) {
            distortedPoints.add(Point2D.create());
        }
        distort(undistortedPoints, distortedPoints);
        return distortedPoints;
    }

    /**
     * Distorts provided 2D points and stores them into provided distorted
     * points list.
     *
     * @param undistortedPoints list of undistorted points to be distorted.
     * @param distortedPoints   distorted points where result is stored.
     * @throws IllegalArgumentException if both lists don't have the same size.
     * @throws NotSupportedException    raised if distortion implementation does
     *                                  not support distorting points.
     * @throws DistortionException      raised if distortion computation failed.
     */
    public void distort(final List<Point2D> undistortedPoints, final List<Point2D> distortedPoints)
            throws NotSupportedException, DistortionException {
        if (undistortedPoints.size() != distortedPoints.size()) {
            throw new IllegalArgumentException();
        }

        final var it1 = undistortedPoints.iterator();
        final var it2 = distortedPoints.iterator();
        while (it1.hasNext() && it2.hasNext()) {
            var undistortedPoint = it1.next();
            var distortedPoint = it2.next();
            distort(undistortedPoint, distortedPoint);
        }
    }

    /**
     * Un-distorts provided 2D point.
     *
     * @param distortedPoint distorted point to be undistorted
     * @return undistorted point.
     * @throws NotSupportedException raised if distortion implementation does
     *                               not support un-distorting points.
     * @throws DistortionException   raised if un-distortion computation failed.
     */
    public Point2D undistort(final Point2D distortedPoint) throws NotSupportedException, DistortionException {
        final var undistortedPoint = Point2D.create();
        undistort(distortedPoint, undistortedPoint);
        return undistortedPoint;
    }

    /**
     * Un-distorts provided 2D points.
     *
     * @param distortedPoints list of distorted points to be undistorted
     * @return undistorted points.
     * @throws NotSupportedException raised if distortion implementation does
     *                               not support un-distorting points.
     * @throws DistortionException   raised if un-distortion computation failed.
     */
    public List<Point2D> undistort(final List<Point2D> distortedPoints) throws NotSupportedException,
            DistortionException {
        final var size = distortedPoints.size();
        final var undistortedPoints = new ArrayList<Point2D>(size);
        for (var i = 0; i < size; i++) {
            undistortedPoints.add(Point2D.create());
        }
        undistort(distortedPoints, undistortedPoints);
        return undistortedPoints;
    }

    /**
     * Un-distorts provided 2D points and stores them into provided undistorted
     * points list.
     *
     * @param distortedPoints   list of distorted points to be undistorted.
     * @param undistortedPoints undistorted points where result is stored.
     * @throws IllegalArgumentException if both lists don't have the same size.
     * @throws NotSupportedException    raised if distortion implementation does
     *                                  not support un-distorting points.
     * @throws DistortionException      raised if un-distortion computation failed.
     */
    public void undistort(
            final List<Point2D> distortedPoints, final List<Point2D> undistortedPoints) throws NotSupportedException,
            DistortionException {
        if (distortedPoints.size() != undistortedPoints.size()) {
            throw new IllegalArgumentException();
        }

        final var it1 = distortedPoints.iterator();
        final var it2 = undistortedPoints.iterator();
        while (it1.hasNext() && it2.hasNext()) {
            final var distortedPoint = it1.next();
            final var undistortedPoint = it2.next();
            undistort(distortedPoint, undistortedPoint);
        }
    }

    /**
     * Distorts provided 2D point and stores result into provided distorted
     * point.
     *
     * @param undistortedPoint undistorted point to be undistorted.
     * @param distortedPoint   distorted point where result is stored.
     * @throws NotSupportedException raised if distortion implementation does
     *                               not support distorting points.
     * @throws DistortionException   raised if distortion computation failed.
     */
    public abstract void distort(final Point2D undistortedPoint, final Point2D distortedPoint)
            throws NotSupportedException, DistortionException;

    /**
     * Un-distorts provided 2D point and stores result into provided undistorted
     * point.
     *
     * @param distortedPoint   distorted point to be undistorted.
     * @param undistortedPoint undistorted point where result is stored.
     * @throws NotSupportedException raised if distortion implementation does
     *                               not support un-distorting points.
     * @throws DistortionException   raised if un-distortion computation failed.
     */
    public abstract void undistort(final Point2D distortedPoint, final Point2D undistortedPoint)
            throws NotSupportedException, DistortionException;

    /**
     * Indicates whether this instance can distort points.
     *
     * @return true if points can be distorted, false otherwise.
     */
    public abstract boolean canDistort();

    /**
     * Indicates whether this instance can un-distort points.
     *
     * @return true if points can be undistorted, false otherwise.
     */
    public abstract boolean canUndistort();

    /**
     * Returns kind of distortion.
     *
     * @return kind of distortion.
     */
    public abstract DistortionKind getKind();
}
