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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.numerical.robust.WeightSelection;
import com.irurueta.sorting.SortingException;

import java.util.List;

/**
 * Triangulates matched 2D points into a single 3D one by using 2D point
 * correspondences on different views along with the corresponding cameras on
 * each of those views by finding a weighted solution to an inhomogeneous 
 * system of equations.
 * Each equation on the linear system of equations is weighted using provided 
 * weight for each point and camera correspondence, so that some equations can
 * be considered more important than others if we are more confident on some
 * measures than others.
 */
@SuppressWarnings("WeakerAccess")
public class WeightedInhomogeneousSinglePoint3DTriangulator extends 
        SinglePoint3DTriangulator {
    
    /**
     * Default number of correspondences to be weighted and taken into account.
     * If more correspondences are provided, they are ignored to avoid numerical
     * inaccuracies.
     */
    public static final int DEFAULT_MAX_CORRESPONDENCES = 50;
    
    /**
     * Indicates if weights are sorted by default so that largest weighted
     * correspondences are used first.
     */
    public static final boolean DEFAULT_SORT_WEIGHTS = true;
    
    /**
     * Maximum number of correspondences to be weighted and taken into account.
     */
    private int mMaxCorrespondences;
    
    /**
     * Indicates if weights are sorted by default so that largest weighted
     * correspondences are used first.
     */
    private boolean mSortWeights;
    
    /**
     * Array containing weights for all correspondences.
     */
    private double[] mWeights;
    
    /**
     * Constructor.
     */
    public WeightedInhomogeneousSinglePoint3DTriangulator() {
        super();
        mMaxCorrespondences = DEFAULT_MAX_CORRESPONDENCES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
    }
    
    /**
     * Constructor.
     * @param points2D list of matched 2D points on each view. Each point in the
     * list is assumed to be projected by the corresponding camera in the list.
     * @param cameras camera for each view where 2D points are represented.
     * @throws IllegalArgumentException if provided lists don't have the same
     * length or their length is less than 2 views, which is the minimum 
     * required to compute triangulation.
     */
    public WeightedInhomogeneousSinglePoint3DTriangulator(
            List<Point2D> points2D, List<PinholeCamera> cameras) {
        super(points2D, cameras);
        mMaxCorrespondences = DEFAULT_MAX_CORRESPONDENCES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
    }
    
    /**
     * Constructor.
     * @param points2D list of matched 2D points on each view. Each point in the
     * list is assumed to be projected by the corresponding camera in the list.
     * @param cameras camera for each view where 2D points are represented.
     * @param weights weights assigned to each view.
     * @throws IllegalArgumentException if provided lists or weights don't have 
     * the same length or their length is less than 2 views, which is the 
     * minimum required to compute triangulation.
     */
    public WeightedInhomogeneousSinglePoint3DTriangulator(
            List<Point2D> points2D, List<PinholeCamera> cameras, 
            double[] weights) {
        this();
        internalSetPointsCamerasAndWeights(points2D, cameras, weights);
    }
    
    
    /**
     * Constructor.
     * @param listener listener to notify events generated by instances of this
     * class.
     */
    public WeightedInhomogeneousSinglePoint3DTriangulator(
            SinglePoint3DTriangulatorListener listener) {
        super(listener);
        mMaxCorrespondences = DEFAULT_MAX_CORRESPONDENCES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
    }
    
    /**
     * Constructor.
     * @param points2D list of matched 2D points on each view. Each point in the
     * list is assumed to be projected by the corresponding camera in the list.
     * @param cameras cameras for each view where 2D points are represented.
     * @param listener listener to notify events generated by instances of this
     * class.
     * @throws IllegalArgumentException if provided lists don't have the same
     * length or their length is less than 2 views, which is the minimum
     * required to compute triangulation.
     */
    public WeightedInhomogeneousSinglePoint3DTriangulator(
            List<Point2D> points2D, List<PinholeCamera> cameras, 
            SinglePoint3DTriangulatorListener listener) {
        super(points2D, cameras, listener);
        mMaxCorrespondences = DEFAULT_MAX_CORRESPONDENCES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
    }
    
    /**
     * Constructor.
     * @param points2D list of matched 2D points on each view. Each point in the
     * list is assumed to be projected by the corresponding camera in the list.
     * @param cameras camera for each view where 2D points are represented.
     * @param weights weights assigned to each view.
     * @param listener listener to notify events generated by instances of this
     * class.
     * @throws IllegalArgumentException if provided lists or weights don't have 
     * the same length or their length is less than 2 views, which is the 
     * minimum required to compute triangulation.
     */
    public WeightedInhomogeneousSinglePoint3DTriangulator(
            List<Point2D> points2D, List<PinholeCamera> cameras, 
            double[] weights, SinglePoint3DTriangulatorListener listener) {
        this(listener);
        internalSetPointsCamerasAndWeights(points2D, cameras, weights);
    }    
    
    /**
     * Returns weights assigned to each view.
     * The larger a weight is the more reliable a view is considered and 
     * equations to triangulate a 3D point will be take precedence over other
     * views when estimating an averaged solution.
     * @return weights assigned to each view.
     */
    public double[] getWeights() {
        return mWeights;
    }
    
    /**
     * Sets list of matched 2D points for each view and their corresponding 
     * cameras used to project them along with their weights.
     * @param points2D list of matched 2D points on each view. Each point in the
     * list is assumed to be projected by the corresponding camera in the list.
     * @param cameras cameras for each view where 2D points are represented.
     * @param weights weights assigned to each view.
     * @throws LockedException if this instance is locked.
     * @throws IllegalArgumentException if provided lists don't have the same
     * length or their length is less than 2 views, which is the minimum 
     * required to compute triangulation.
     */    
    public void setPointsCamerasAndWeights(List<Point2D> points2D,
            List<PinholeCamera> cameras, double[] weights) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPointsCamerasAndWeights(points2D, cameras, weights);
    }
    
    /**
     * Indicates whether this instance is ready to start the triangulation.
     * An instance is ready when both lists of 2D points and cameras are 
     * provided, both lists have the same length, at least data for 2 views
     * is provided and the corresponding weights are also provided.
     * @return true if this instance is ready, false otherwise.
     */    
    @Override
    public boolean isReady() {
        return areValidPointsCamerasAndWeights(mPoints2D, mCameras, mWeights);
    }
    
    /**
     * Indicates whether provided points, cameras and weights are valid to start
     * the triangulation.
     * In order to triangulate points, at least two cameras and their 
     * corresponding 2 matched 2D points are required along with weights for
     * each view.
     * If more views are provided, an averaged solution can be found.
     * @param points2D list of matched points on each view.
     * @param cameras cameras for each view where 2D points are represented.
     * @param weights weights assigned to each view.
     * @return true if data is enough to start triangulation, false otherwise.
     */
    public static boolean areValidPointsCamerasAndWeights(
            List<Point2D> points2D, List<PinholeCamera> cameras, 
            double[] weights) {
        return areValidPointsAndCameras(points2D, cameras) && weights != null &&
                weights.length == points2D.size();
    }
    
    /**
     * Returns maximum number of correspondences to be weighted and taken into
     * account.
     * @return maximum number of correspondences to be weighted.
     */
    public int getMaxCorrespondences() {
        return mMaxCorrespondences;
    }
    
    /**
     * Sets maximum number of correspondences to be weighted and taken into 
     * account.
     * @param maxCorrespondences maximum number of correspondences to be 
     * weighted.
     * @throws IllegalArgumentException if provided value is less than the 
     * minimum required number of views, which is 2.
     * @throws LockedException if this instance is locked.
     */
    public void setMaxCorrespondences(int maxCorrespondences) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxCorrespondences < MIN_REQUIRED_VIEWS) {
            throw new IllegalArgumentException();
        }
        
        mMaxCorrespondences = maxCorrespondences;
    }
    
    /**
     * Indicates if weights are sorted by so that largest weighted 
     * correspondences are used first.
     * @return true if weights are sorted, false otherwise.
     */
    public boolean isSortWeightsEnabled() {
        return mSortWeights;
    }
    
    /**
     * Specifies whether weights are sorted by so that largest weighted
     * correspondences are used first.
     * @param sortWeights true if weights are sorted, false otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setSortWeightsEnabled(boolean sortWeights)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mSortWeights = sortWeights;
    }
    
    /**
     * Returns type of triangulator.
     * @return type of triangulator.
     */
    @Override
    public Point3DTriangulatorType getType() {
        return Point3DTriangulatorType.WEIGHTED_INHOMOGENEOUS_TRIANGULATOR;
    }

    /**
     * Internal method to triangulate provided matched 2D points being projected 
     * by each corresponding camera into a single 3D point.
     * At least 2 matched 2D points and their corresponding 2 cameras are 
     * required to compute triangulation. If more views are provided, an 
     * averaged solution is found.
     * This method does not check whether instance is locked or ready.
     * @param points2D matched 2D points. Each point in the list is assumed to
     * be projected by the corresponding camera in the list.
     * @param cameras list of cameras associated to the matched 2D point on the
     * same position as the camera on the list.
     * @param result instance where triangulated 3D point is stored.
     * @throws Point3DTriangulationException if triangulation fails for some
     * other reason (i.e. degenerate geometry, numerical instabilities, etc).
     */    
    @Override
    @SuppressWarnings("Duplicates")
    protected void triangulate(List<Point2D> points2D, 
            List<PinholeCamera> cameras, Point3D result) 
            throws Point3DTriangulationException {
        try {
            mLocked = true;
            
            if (mListener != null) {
                mListener.onTriangulateStart(this);
            }
            
            WeightSelection selection = WeightSelection.selectWeights(mWeights, 
                    mSortWeights, mMaxCorrespondences);
            
            boolean[] selected = selection.getSelected();
            
            int numViews = cameras.size();
            
            Matrix a = new Matrix(2 * numViews, 3);
            double[] b = new double[2 * numViews];
            
            Point2D point;
            PinholeCamera camera;
            Plane horizontalAxisPlane = new Plane();
            Plane verticalAxisPlane = new Plane();
            Plane principalPlane = new Plane();
            int row = 0;
            double rowNorm;
            for (int i = 0; i < numViews; i++) {
                if (selected[i]) {
                    point = points2D.get(i);
                    camera = cameras.get(i);
                
                    //to increase accuracy
                    point.normalize();
                    camera.normalize();
                
                    double homX = point.getHomX();
                    double homY = point.getHomY();
                    double homW = point.getHomW();
                
                    //pick rows of camera corresponding to different planes
                    //(we do not normalize planes, as it would introduce errors)
                
                    //1st camera row (p1T)
                    camera.verticalAxisPlane(verticalAxisPlane);
                    //2nd camera row (p2T)
                    camera.horizontalAxisPlane(horizontalAxisPlane);
                    //3rd camera row (p3T)
                    camera.principalPlane(principalPlane);
                
                
                    //1st equation
                    a.setElementAt(row, 0, homX * principalPlane.getA() -
                            homW * verticalAxisPlane.getA());
                    a.setElementAt(row, 1, homX * principalPlane.getB() -
                            homW * verticalAxisPlane.getB());
                    a.setElementAt(row, 2, homX * principalPlane.getC() -
                            homW * verticalAxisPlane.getC());
                
                    b[row] = homW * verticalAxisPlane.getD() -
                            homX * principalPlane.getD();
                
                    //normalize equation to increase accuracy
                    rowNorm = Math.sqrt(Math.pow(a.getElementAt(row, 0), 2.0) +
                            Math.pow(a.getElementAt(row, 1), 2.0) +
                            Math.pow(a.getElementAt(row, 2), 2.0));
                
                    a.setElementAt(row, 0, a.getElementAt(row, 0) / rowNorm);
                    a.setElementAt(row, 1, a.getElementAt(row, 1) / rowNorm);
                    a.setElementAt(row, 2, a.getElementAt(row, 2) / rowNorm);
                    b[row] /= rowNorm;
                
                    //2nd equation
                    row++;
                
                    a.setElementAt(row, 0, homY * principalPlane.getA() -
                            homW * horizontalAxisPlane.getA());
                    a.setElementAt(row, 1, homY * principalPlane.getB() -
                            homW * horizontalAxisPlane.getB());
                    a.setElementAt(row, 2, homY * principalPlane.getC() -
                            homW * horizontalAxisPlane.getC());
                
                    b[row] = homW * horizontalAxisPlane.getD() -
                            homY * principalPlane.getD();
                
                    //normalize equation to increase accuracy
                    rowNorm = Math.sqrt(Math.pow(a.getElementAt(row, 0), 2.0) +
                            Math.pow(a.getElementAt(row, 1), 2.0) +
                            Math.pow(a.getElementAt(row, 2), 2.0));
                
                    a.setElementAt(row, 0, a.getElementAt(row, 0) / rowNorm);
                    a.setElementAt(row, 1, a.getElementAt(row, 1) / rowNorm);
                    a.setElementAt(row, 2, a.getElementAt(row, 2) / rowNorm);
                    b[row] /= rowNorm;
                }
            }
            
            //make SVD to find solution of A * M = 0
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            
            decomposer.decompose();
            
            //solve linear system of equations to obtain inhomogeneous 
            //coordinates of triangulated point
            double[] solution = decomposer.solve(b);
            
            result.setInhomogeneousCoordinates(solution[0], solution[1], 
                    solution[2]);
            
            if (mListener != null) {
                mListener.onTriangulateEnd(this);
            }
        } catch (AlgebraException | SortingException e) {
            throw new Point3DTriangulationException(e);
        } finally {
            mLocked = false;
        }

    }
    
    /**
     * Internal method to set list of matched 2D points for each view and their
     * corresponding cameras used to project them along with their weights.
     * This method does not check whether instance is locked.
     * @param points2D list of matched 2D points on each view. Each point in the
     * list is assumed to be projected by the corresponding camera in the list.
     * @param cameras cameras for each view where 2D points are represented.
     * @param weights weights assigned to each view.
     * @throws IllegalArgumentException if provided lists don't have the same
     * length or their length is less than 2 views, which is the minimum 
     * required to compute triangulation.
     */
    @SuppressWarnings("Duplicates")
    private void internalSetPointsCamerasAndWeights(List<Point2D> points2D,
            List<PinholeCamera> cameras, double[] weights) {
        if (!areValidPointsCamerasAndWeights(points2D, cameras, weights)) {
            throw new IllegalArgumentException();
        }
        
        mPoints2D = points2D;
        mCameras = cameras;
        mWeights = weights;
    }
}
