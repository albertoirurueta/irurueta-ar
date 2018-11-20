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
import java.util.Iterator;
import java.util.List;

/**
 * This class accounts for any possible distortion that might occur on 2D 
 * points. This might occur for instance when using cameras that produce 
 * additional distortion (such as radial distortion) not modelled in the general
 * camera model.
 * This class is abstract, specific implementations for each kind of supported
 * distortion exists.
 */
@SuppressWarnings("WeakerAccess")
public abstract class Distortion {

    /**
     * Distorts provided 2D point.
     * @param undistortedPoint undistorted point to be distorted.
     * @return distorted point.
     * @throws NotSupportedException raised if distortion implementation does
     * not support distorting points.
     * @throws DistortionException raised if distortion computation failed.
     */
    public Point2D distort(Point2D undistortedPoint) 
            throws NotSupportedException, DistortionException {
        Point2D distortedPoint = Point2D.create();
        distort(undistortedPoint, distortedPoint);
        return distortedPoint;
    }
    
    /**
     * Distorts provided 2D points.
     * @param undistortedPoints list of undistorted points to be distorted.
     * @return distorted points.
     * @throws NotSupportedException raised if distortion implementation does
     * not support distorting points.
     * @throws DistortionException raised if distortion computation failed.
     */
    public List<Point2D> distort(List<Point2D> undistortedPoints) 
            throws NotSupportedException, DistortionException {
        int size = undistortedPoints.size();
        List<Point2D> distortedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            distortedPoints.add(Point2D.create());
        }
        distort(undistortedPoints, distortedPoints);
        return distortedPoints;
    }
    
    /**
     * Distorts provided 2D points and stores them into provided distorted
     * points list.
     * @param undistortedPoints list of undistorted points to be distorted.
     * @param distortedPoints distorted points where result is stored.
     * @throws IllegalArgumentException if both lists don't have the same size.
     * @throws NotSupportedException raised if distortion implementation does
     * not support distorting points.
     * @throws DistortionException raised if distortion computation failed.
     */
    public void distort(List<Point2D> undistortedPoints, 
            List<Point2D> distortedPoints) throws IllegalArgumentException, 
            NotSupportedException, DistortionException {
        if (undistortedPoints.size() != distortedPoints.size()) {
            throw new IllegalArgumentException();
        }
        
        Iterator<Point2D> it1 = undistortedPoints.iterator();
        Iterator<Point2D> it2 = distortedPoints.iterator();
        while (it1.hasNext() && it2.hasNext()) {
            Point2D undistortedPoint = it1.next();
            Point2D distortedPoint = it2.next();
            distort(undistortedPoint, distortedPoint);
        }
    }
        
    /**
     * Undistorts provided 2D point.
     * @param distortedPoint distorted point to be undistorted
     * @return undistorted point.
     * @throws NotSupportedException riased if distortion implementation does
     * not support undistorting points.
     * @throws DistortionException raised if undistortion computation failed.
     */
    public Point2D undistort(Point2D distortedPoint) 
            throws NotSupportedException, DistortionException {
        Point2D undistortedPoint = Point2D.create();
        undistort(distortedPoint, undistortedPoint);
        return undistortedPoint;
    }
    
    /**
     * Undistorts provided 2D points.
     * @param distortedPoints list of distorted points to be undistorted
     * @return undistorted points.
     * @throws NotSupportedException raised if distortion implementation does
     * not support undistorting points.
     * @throws DistortionException raised if undistortion computation failed.
     */
    public List<Point2D> undistort(List<Point2D> distortedPoints) 
            throws NotSupportedException, DistortionException {
        int size = distortedPoints.size();
        List<Point2D> undistortedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            undistortedPoints.add(Point2D.create());
        }
        undistort(distortedPoints, undistortedPoints);
        return undistortedPoints;        
    }
    
    /**
     * Undistorts provided 2D points and stores them into provided undistorted
     * points list.
     * @param distortedPoints list of distorted points to be undistorted.
     * @param undistortedPoints undistorted points where result is stored.
     * @throws IllegalArgumentException if both lists don't have the same size.
     * @throws NotSupportedException riased if distortion implementation does
     * not support undistorting points.
     * @throws DistortionException raised if undistortion computation failed.
     */
    public void undistort(List<Point2D> distortedPoints, 
            List<Point2D> undistortedPoints) throws IllegalArgumentException, 
            NotSupportedException, DistortionException {
        if (distortedPoints.size() != undistortedPoints.size()) {
            throw new IllegalArgumentException();
        }
        
        Iterator<Point2D> it1 = distortedPoints.iterator();
        Iterator<Point2D> it2 = undistortedPoints.iterator();
        while (it1.hasNext() && it2.hasNext()) {
            Point2D distortedPoint = it1.next();
            Point2D undistortedPoint = it2.next();
            undistort(distortedPoint, undistortedPoint);
        }
    }
    
    /**
     * Distorts provided 2D point and stores result into provided distorted
     * point.
     * @param undistortedPoint undistorted point to be undistorted.
     * @param distortedPoint distorted point where result is stored.
     * @throws NotSupportedException raised if distortion implementation does
     * not support distorting points.
     * @throws DistortionException raised if distortion computation failed.
     */
    public abstract void distort(Point2D undistortedPoint, 
            Point2D distortedPoint) throws NotSupportedException, 
            DistortionException;
    
    /**
     * Undistorts provided 2D point and stores result into provided undistorted
     * point.
     * @param distortedPoint distorted point to be undistorted.
     * @param undistortedPoint undistorted point where result is stored.
     * @throws NotSupportedException riased if distortion implementation does
     * not support undistorting points.
     * @throws DistortionException raised if undistortion computation failed.
     */
    public abstract void undistort(Point2D distortedPoint, 
            Point2D undistortedPoint) throws NotSupportedException, 
            DistortionException;
    
    /**
     * Indicates whether this instance can distort points.
     * @return true if points can be distorted, false otherwise.
     */
    public abstract boolean canDistort();
    
    /**
     * Indicates whether this instance can undistort points.
     * @return true if points can be undistorted, false otherwise.
     */
    public abstract boolean canUndistort();
    
    /**
     * Returnds kind of distortion.
     * @return kind of distortion.
     */
    public abstract DistortionKind getKind();
}
