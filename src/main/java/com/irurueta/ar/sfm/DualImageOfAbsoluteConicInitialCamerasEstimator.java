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

import com.irurueta.ar.calibration.estimators.KruppaDualImageOfAbsoluteConicEstimator;
import com.irurueta.ar.epipolar.Corrector;
import com.irurueta.ar.epipolar.CorrectorType;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * Estimates an initial pair of cameras in the metric stratum (up to an
 * arbitrary scale) using a given fundamental matrix to obtain the Dual Image
 * of Absolute Conic by solving Kruppa equations to obtain the Essential matrix,
 * so that once it is computed it can be used to determine best pair of camera
 * poses and translations by triangulating a set of matched points and checking
 * that their triangulation lies in front of cameras.
 */
public class DualImageOfAbsoluteConicInitialCamerasEstimator extends InitialCamerasEstimator {

    /**
     * Indicates whether matched 2D points must be triangulated by default.
     */
    public static final boolean DEFAULT_TRIANGULATE_POINTS = false;

    /**
     * Indicates whether triangulated points must be marked as valid (i.e. when
     * they lie in front of both of the estimated cameras) or not.
     */
    public static final boolean DEFAULT_MARK_VALID_TRIANGULATED_POINTS = false;

    /**
     * Aspect ratio of intrinsic parameters of cameras.
     * Typically, this value is 1.0 if vertical coordinates increase upwards,
     * or -1.0 if it is the opposite.
     */
    private double aspectRatio = KruppaDualImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO;

    /**
     * Horizontal coordinate of principal point. This value should be the
     * coordinates of the center of an image assuming that the coordinates start
     * on the top-left or bottom-left corner. Using a value close to zero
     * will produce inaccurate results.
     */
    private double principalPointX;

    /**
     * Vertical coordinate of principal point. This value should be the
     * coordinates of the center of an image assuming that the coordinates start
     * on the top-left or bottom-left corner. Using a value close to zero will
     * produce inaccurate results.
     */
    private double principalPointY;

    /**
     * Matched 2D points on left view.
     */
    private List<Point2D> leftPoints;

    /**
     * Matched 2D points on right view.
     */
    private List<Point2D> rightPoints;

    /**
     * Type of corrector to use to triangulate matched points or null if no
     * corrector needs to be used.
     */
    private CorrectorType correctorType = Corrector.DEFAULT_TYPE;

    /**
     * Indicates whether matched 2D points need to be triangulated.
     */
    private boolean triangulatePoints = DEFAULT_TRIANGULATE_POINTS;

    /**
     * Marks which of the triangulated points are marked as valid (lie in front
     * of both of the estimated cameras) and which ones aren't.
     */
    private boolean markValidTriangulatedPoints = DEFAULT_MARK_VALID_TRIANGULATED_POINTS;

    /**
     * Contains triangulated points.
     */
    private List<Point3D> triangulatedPoints;

    /**
     * Contains booleans indicating whether triangulated points are valid (i.e.
     * lie in front of both estimated cameras) or not.
     */
    private BitSet validTriangulatedPoints;

    /**
     * Constructor.
     */
    public DualImageOfAbsoluteConicInitialCamerasEstimator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     */
    public DualImageOfAbsoluteConicInitialCamerasEstimator(final FundamentalMatrix fundamentalMatrix) {
        super(fundamentalMatrix);
    }

    /**
     * Constructor.
     *
     * @param leftPoints  matched 2D points on left view.
     * @param rightPoints matched 2D points on right view.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size.
     */
    public DualImageOfAbsoluteConicInitialCamerasEstimator(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        super();
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param leftPoints        matched 2D points on left view.
     * @param rightPoints       matched 2D points on right view.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size.
     */
    public DualImageOfAbsoluteConicInitialCamerasEstimator(
            final FundamentalMatrix fundamentalMatrix, final List<Point2D> leftPoints,
            final List<Point2D> rightPoints) {
        super(fundamentalMatrix);
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this instance.
     */
    public DualImageOfAbsoluteConicInitialCamerasEstimator(final InitialCamerasEstimatorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param listener          listener to handle events raised by this instance.
     */
    public DualImageOfAbsoluteConicInitialCamerasEstimator(
            final FundamentalMatrix fundamentalMatrix, final InitialCamerasEstimatorListener listener) {
        super(fundamentalMatrix, listener);
    }

    /**
     * Constructor.
     *
     * @param leftPoints  matched 2D points on left view.
     * @param rightPoints matched 2D points on right view.
     * @param listener    listener to handle events raised by this instance.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size.
     */
    public DualImageOfAbsoluteConicInitialCamerasEstimator(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final InitialCamerasEstimatorListener listener) {
        super(listener);
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param leftPoints        matched 2D points on left view.
     * @param rightPoints       matched 2D points on right view.
     * @param listener          listener to handle events raised by this instance.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size.
     */
    public DualImageOfAbsoluteConicInitialCamerasEstimator(
            final FundamentalMatrix fundamentalMatrix,
            final List<Point2D> leftPoints,
            final List<Point2D> rightPoints,
            final InitialCamerasEstimatorListener listener) {
        super(fundamentalMatrix, listener);
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Returns method used by this estimator.
     *
     * @return method used by this estimator.
     */
    @Override
    public InitialCamerasEstimatorMethod getMethod() {
        return InitialCamerasEstimatorMethod.DUAL_IMAGE_OF_ABSOLUTE_CONIC;
    }

    /**
     * Indicates if estimator is ready.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return fundamentalMatrix != null && leftPoints != null && rightPoints != null
                && leftPoints.size() == rightPoints.size();
    }

    /**
     * Estimates cameras.
     *
     * @throws LockedException                         if estimator is locked.
     * @throws NotReadyException                       if estimator is not ready.
     * @throws InitialCamerasEstimationFailedException if estimation of cameras
     *                                                 fails for some reason, typically due to numerical
     *                                                 instabilities.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public void estimate() throws LockedException, NotReadyException, InitialCamerasEstimationFailedException {
        if (isLocked()) {
            throw new LockedException();
        }

        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            locked = true;

            if (listener != null) {
                listener.onStart(this);
            }

            if (triangulatePoints) {
                triangulatedPoints = new ArrayList<>();
            } else {
                triangulatedPoints = null;
            }

            final var nPoints = leftPoints.size();
            if (markValidTriangulatedPoints) {
                validTriangulatedPoints = new BitSet(nPoints);
            } else {
                validTriangulatedPoints = null;
            }

            if (estimatedLeftCamera == null) {
                estimatedLeftCamera = new PinholeCamera();
            }
            if (estimatedRightCamera == null) {
                estimatedRightCamera = new PinholeCamera();
            }

            generateInitialMetricCamerasUsingDIAC(fundamentalMatrix, principalPointX, principalPointY, aspectRatio,
                    leftPoints, rightPoints, correctorType, estimatedLeftCamera, estimatedRightCamera,
                    triangulatedPoints, validTriangulatedPoints);

            if (listener != null) {
                listener.onFinish(this, estimatedLeftCamera, estimatedRightCamera);
            }
        } catch (final InitialCamerasEstimationFailedException e) {
            if (listener != null) {
                listener.onFail(this, e);
            }
            throw e;
        } finally {
            locked = false;
        }
    }

    /**
     * Gets aspect ratio of intrinsic parameters of cameras.
     * Typically, this value is 1.0 if vertical coordinates increase upwards,
     * or -1.0 if it is the opposite.
     *
     * @return aspect ratio of intrinsic parameters of cameras.
     */
    public double getAspectRatio() {
        return aspectRatio;
    }

    /**
     * Sets aspect ratio of intrinsic parameters of cameras.
     * Typically, this value is 1.0 if vertical coordinates increase upwards,
     * or -1.0 if it is the opposite.
     *
     * @param aspectRatio aspect ratio of intrinsic parameters of cameras.
     * @throws LockedException if estimator is locked.
     */
    public void setAspectRatio(final double aspectRatio) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.aspectRatio = aspectRatio;
    }

    /**
     * Gets horizontal coordinate of principal point. This value should be the
     * coordinates of the center of an image assuming that the coordinates start
     * on the top-left or bottom-left corner. Using a value close to zero
     * will produce inaccurate results.
     *
     * @return horizontal coordinate of principal point.
     */
    public double getPrincipalPointX() {
        return principalPointX;
    }

    /**
     * Sets horizontal coordinate of principal point. This value should be the
     * coordinates of the center of an image assuming that the coordinates start
     * on the top-left or bottom-left corner. Using a value close to zero
     * will produce inaccurate results.
     *
     * @param principalPointX horizontal coordinate of principal point.
     * @throws LockedException if estimator is locked.
     */
    public void setPrincipalPointX(final double principalPointX) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.principalPointX = principalPointX;
    }

    /**
     * Gets vertical coordinate of principal point. This value should be the
     * coordinates of the center of an image assuming that the coordinates start
     * on the top-left or bottom-left corner. Using a value close to zero will
     * produce inaccurate results.
     *
     * @return vertical coordinate of principal point.
     */
    public double getPrincipalPointY() {
        return principalPointY;
    }

    /**
     * Sets vertical coordinate of principal point. This value should be the
     * coordinates of the center of an image assuming that the coordinates start
     * on the top-left or bottom-left corner. Using a value close to zero will
     * produce inaccurate results.
     *
     * @param principalPointY vertical coordinate of principal point.
     * @throws LockedException if estimator is locked.
     */
    public void setPrincipalPointY(final double principalPointY) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.principalPointY = principalPointY;
    }

    /**
     * Sets horizontal and vertical coordinates of principal point. This value
     * should be the coordinates of the center of an image assuming that the
     * coordinates start on the top-left or bottom-left corner. Using a value
     * close to zero will produce inaccurate results.
     *
     * @param principalPointX horizontal coordinate of principal point.
     * @param principalPointY vertical coordinate of principal point.
     * @throws LockedException if estimator is locked.
     */
    public void setPrincipalPoint(final double principalPointX, final double principalPointY) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.principalPointX = principalPointX;
        this.principalPointY = principalPointY;
    }

    /**
     * Gets matched 2D points on left view.
     *
     * @return matched 2D points on left view.
     */
    public List<Point2D> getLeftPoints() {
        return leftPoints;
    }

    /**
     * Sets matched 2D points on left view.
     *
     * @param leftPoints matched 2D points on left view.
     * @throws LockedException if estimator is locked.
     */
    public void setLeftPoints(final List<Point2D> leftPoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.leftPoints = leftPoints;
    }

    /**
     * Gets matched 2D points on right view.
     *
     * @return matched 2D points on right view.
     */
    public List<Point2D> getRightPoints() {
        return rightPoints;
    }

    /**
     * Sets matched 2D points on right view.
     *
     * @param rightPoints matched 2D points on right view.
     * @throws LockedException if estimator is locked.
     */
    public void setRightPoints(final List<Point2D> rightPoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.rightPoints = rightPoints;
    }

    /**
     * Sets matched 2D points on left and right views.
     *
     * @param leftPoints  matched 2D points on left view.
     * @param rightPoints matched 2D points on right view.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size.
     */
    public void setLeftAndRightPoints(final List<Point2D> leftPoints, final List<Point2D> rightPoints)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Gets type of corrector to use to triangulate matched points or null if
     * no corrector needs to be used.
     *
     * @return type of corrector to use.
     */
    public CorrectorType getCorrectorType() {
        return correctorType;
    }

    /**
     * Sets type of corrector to use to triangulate matched points or null if
     * no corrector needs to be used.
     *
     * @param correctorType type of corrector to use.
     * @throws LockedException if estimator is locked.
     */
    public void setCorrectorType(final CorrectorType correctorType) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.correctorType = correctorType;
    }

    /**
     * Indicates whether matched 2D points need to be triangulated or not.
     *
     * @return true if 2D points need to be triangulated, false otherwise.
     */
    public boolean arePointsTriangulated() {
        return triangulatePoints;
    }

    /**
     * Specifies whether matched 2D points need to be triangulated or not.
     *
     * @param triangulatePoints true if 2D points need to be triangulated, false
     *                          otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setPointsTriangulated(final boolean triangulatePoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.triangulatePoints = triangulatePoints;
    }

    /**
     * Indicates which triangulated points are marked as valid (lie in front
     * of both of the estimated cameras) and which ones aren't.
     *
     * @return true to mark valid and invalid triangulated points, false
     * otherwise.
     */
    public boolean areValidTriangulatedPointsMarked() {
        return markValidTriangulatedPoints;
    }

    /**
     * Specifies whether triangulated points are marked as valid (lie in front
     * of both of the estimated cameras) and which ones aren't.
     *
     * @param markValidTriangulatedPoints true to mark valid and invalid
     *                                    triangulated points, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setValidTriangulatedPointsMarked(final boolean markValidTriangulatedPoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.markValidTriangulatedPoints = markValidTriangulatedPoints;
    }

    /**
     * Gets triangulated points, if available.
     *
     * @return triangulated points or null.
     */
    public List<Point3D> getTriangulatedPoints() {
        return triangulatedPoints;
    }

    /**
     * Gets bitset indicating which of the triangulated points are valid and
     * which ones aren't.
     *
     * @return bitset indicating validity of triangulated points or null if not
     * available.
     */
    public BitSet getValidTriangulatedPoints() {
        return validTriangulatedPoints;
    }

    /**
     * Generates a pair of metric cameras (up to an arbitrary space) by
     * estimating the intrinsic parameters of the views by solving the Kruppa
     * equations to obtain the Dual Image of Absolute Conic (DIAC).
     * The estimated intrinsic parameters can later be used to find the
     * essential matrix (assuming that both views have the same intrinsic
     * parameters), and the essential matrix along with provided matched 2D
     * points can be used to determine the best pair of camera pose and
     * translation that yields the largest number of triangulated points laying
     * in front of both of the estimated cameras.
     * This method uses default corrector type, does not keep triangulated
     * points or valid triangulated points, and uses default aspect ratio (1.0).
     *
     * @param fundamentalMatrix fundamental matrix relating both left and right
     *                          views.
     * @param principalPointX   horizontal coordinate of principal point. This
     *                          value should be the coordinates of the center of an image assuming that
     *                          the coordinates start on the top-left or bottom-left corner. Using a
     *                          value close to zero will produce inaccurate results.
     * @param principalPointY   vertical coordinate of principal point. This
     *                          value should be the coordinates of the center of an image assuming that
     *                          the coordinates start on the top-left or bottom-left corner. Using a
     *                          value close to zero will produce inaccurate results.
     * @param leftPoints        points on left view matched with points on right view,
     *                          so they can be triangulated using estimated cameras. Both lists of points
     *                          must have the same size.
     * @param rightPoints       points on right view matched with points on left view,
     *                          so they can be triangulated using estimated cameras. Both lists of points
     *                          must have the same size.
     * @param leftCamera        instance where estimated left camera will be stored.
     * @param rightCamera       instance where estimated right camera will be stored.
     * @return number of valid triangulated points which lie in front of the two
     * estimated cameras.
     * @throws InitialCamerasEstimationFailedException if estimation of cameras
     *                                                 fails for some reason, typically due to numerical
     *                                                 instabilities.
     * @throws IllegalArgumentException                if provided lists of left and right
     *                                                 points don't have the same size.
     */
    public static int generateInitialMetricCamerasUsingDIAC(
            final FundamentalMatrix fundamentalMatrix, final double principalPointX,
            final double principalPointY, final List<Point2D> leftPoints,
            final List<Point2D> rightPoints, final PinholeCamera leftCamera,
            final PinholeCamera rightCamera) throws InitialCamerasEstimationFailedException {

        return generateInitialMetricCamerasUsingDIAC(fundamentalMatrix, principalPointX, principalPointY,
                leftPoints, rightPoints, Corrector.DEFAULT_TYPE, leftCamera, rightCamera);
    }

    /**
     * Generates a pair of metric cameras (up to an arbitrary space) by
     * estimating the intrinsic parameters of the views by solving the Kruppa
     * equations to obtain the Dual Image of Absolute Conic (DIAC).
     * The estimated intrinsic parameters can later be used to find the
     * essential matrix (assuming that both views have the same intrinsic
     * parameters), and the essential matrix along with provided matched 2D
     * points can be used to determine the best pair of camera pose and
     * translation that yields the largest number of triangulated points laying
     * in front of both of the estimated cameras.
     * This method does not keep triangulated points or valid triangulated
     * points and uses default aspect ratio (1.0).
     *
     * @param fundamentalMatrix fundamental matrix relating both left and right
     *                          views.
     * @param principalPointX   horizontal coordinate of principal point. This
     *                          value should be the coordinates of the center of an image assuming that
     *                          the coordinates start on the top-left or bottom-left corner. Using a
     *                          value close to zero will produce inaccurate results.
     * @param principalPointY   vertical coordinate of principal point. This
     *                          value should be the coordinates of the center of an image assuming that
     *                          the coordinates start on the top-left or bottom-left corner. Using a
     *                          value close to zero will produce inaccurate results.
     * @param leftPoints        points on left view matched with points on right view,
     *                          so they can be triangulated using estimated cameras. Both lists of points
     *                          must have the same size.
     * @param rightPoints       points on right view matched with points on left view,
     *                          so they can be triangulated using estimated cameras. Both lists of points
     *                          must have the same size.
     * @param correctorType     corrector type to be used to correct 2D points, so
     *                          they follow the epipolar geometry defined by provided fundamental matrix
     *                          so that error on triangulated points is reduced. If null, no corrector
     *                          will be used.
     * @param leftCamera        instance where estimated left camera will be stored.
     * @param rightCamera       instance where estimated right camera will be stored.
     * @return number of valid triangulated points which lie in front of the two
     * estimated cameras.
     * @throws InitialCamerasEstimationFailedException if estimation of
     *                                                 cameras fails for some reason, typically due to
     *                                                 numerical instabilities.
     * @throws IllegalArgumentException                if provided lists of left and right
     *                                                 points don't have the same size.
     */
    public static int generateInitialMetricCamerasUsingDIAC(
            final FundamentalMatrix fundamentalMatrix, final double principalPointX,
            final double principalPointY, final List<Point2D> leftPoints,
            final List<Point2D> rightPoints, final CorrectorType correctorType,
            final PinholeCamera leftCamera, final PinholeCamera rightCamera)
            throws InitialCamerasEstimationFailedException {

        return generateInitialMetricCamerasUsingDIAC(fundamentalMatrix, principalPointX, principalPointY,
                leftPoints, rightPoints, correctorType, leftCamera, rightCamera, null,
                null);
    }

    /**
     * Generates a pair of metric cameras (up to an arbitrary space) by
     * estimating the intrinsic parameters of the views by solving the Kruppa
     * equations to obtain the Dual Image of Absolute Conic (DIAC).
     * The estimated intrinsic parameters can later be used to find the
     * essential matrix (assuming that both views have the same intrinsic
     * parameters), and the essential matrix along with provided matched 2D
     * points can be used to determine the best pair of camera pose and
     * translation that yields the largest number of triangulated points laying
     * in front of both of the estimated cameras.
     * This method uses default corrector type and default aspect ratio (1.0).
     *
     * @param fundamentalMatrix       fundamental matrix relating both left and right
     *                                views.
     * @param principalPointX         horizontal coordinate of principal point. This
     *                                value should be the coordinates of the center of an image assuming
     *                                that the coordinates start on the top-left or bottom-left corner.
     *                                Using a value close to zero will produce inaccurate results.
     * @param principalPointY         vertical coordinate of principal point. This
     *                                value should be the coordinates of the center of an image assuming
     *                                that the coordinates start on the top-left or bottom-left corner.
     *                                Using a value close to zero will produce inaccurate results.
     * @param leftPoints              points on left view matched with points on right view,
     *                                so they can be triangulated using estimated cameras. Both lists of
     *                                points must have the same size.
     * @param rightPoints             points on right view matched with points on left view,
     *                                so they can be triangulated using estimated cameras. Both lists of
     *                                points must have the same size.
     * @param leftCamera              instance where estimated left camera will be stored.
     * @param rightCamera             instance where estimated right camera will be stored.
     * @param triangulatedPoints      instance where triangulated 3D points will be
     *                                stored or null if triangulated points don't need to be kept.
     * @param validTriangulatedPoints instance which indicates which
     *                                triangulated 3D points are considered valid because they lie in
     *                                front of both cameras or null if such data doesn't need to be kept.
     * @return number of valid triangulated points which lie in front of the two
     * estimated cameras.
     * @throws InitialCamerasEstimationFailedException if estimation of cameras
     *                                                 fails for some reason, typically due to numerical
     *                                                 instabilities.
     * @throws IllegalArgumentException                if provided lists of left and right
     *                                                 points don't have the same size.
     */
    public static int generateInitialMetricCamerasUsingDIAC(
            final FundamentalMatrix fundamentalMatrix, final double principalPointX,
            final double principalPointY, final List<Point2D> leftPoints,
            final List<Point2D> rightPoints, final PinholeCamera leftCamera,
            final PinholeCamera rightCamera, final List<Point3D> triangulatedPoints,
            final BitSet validTriangulatedPoints)
            throws InitialCamerasEstimationFailedException {

        return generateInitialMetricCamerasUsingDIAC(fundamentalMatrix, principalPointX, principalPointY,
                leftPoints, rightPoints, Corrector.DEFAULT_TYPE, leftCamera, rightCamera, triangulatedPoints,
                validTriangulatedPoints);
    }

    /**
     * Generates a pair of metric cameras (up to an arbitrary space) by
     * estimating the intrinsic parameters of the views by solving the Kruppa
     * equations to obtain the Dual Image of Absolute Conic (DIAC).
     * The estimated intrinsic parameters can later be used to find the
     * essential matrix (assuming that both views have the same intrinsic
     * parameters), and the essential matrix along with provided matched 2D
     * points can be used to determine the best pair of camera pose and
     * translation that yields the largest number of triangulated points laying
     * in front of both of the estimated cameras.
     * This method uses default aspect ratio (1.0).
     *
     * @param fundamentalMatrix       fundamental matrix relating both left and right
     *                                views.
     * @param principalPointX         horizontal coordinate of principal point. This
     *                                value should be the coordinates of the center of an image assuming
     *                                that the coordinates start on the top-left or bottom-left corner.
     *                                Using a value close to zero will produce inaccurate results.
     * @param principalPointY         vertical coordinate of principal point. This
     *                                value should be the coordinates of the center of an image assuming
     *                                that the coordinates start on the top-left or bottom-left corner.
     *                                Using a value close to zero will produce inaccurate results.
     * @param leftPoints              points on left view matched with points on right view,
     *                                so they can be triangulated using estimated cameras. Both lists of
     *                                points must have the same size.
     * @param rightPoints             points on right view matched with points on left view,
     *                                so they can be triangulated using estimated cameras. Both lists of
     *                                points must have the same size.
     * @param correctorType           corrector type to be used to correct 2D points, so
     *                                they follow the epipolar geometry defined by provided fundamental
     *                                matrix so that error on triangulated points is reduced. If null, no
     *                                corrector will be used.
     * @param leftCamera              instance where estimated left camera will be stored.
     * @param rightCamera             instance where estimated right camera will be stored.
     * @param triangulatedPoints      instance where triangulated 3D points will be
     *                                stored or null if triangulated points don't need to be kept.
     * @param validTriangulatedPoints instance which indicates which
     *                                triangulated 3D points are considered valid because they lie in
     *                                front of both cameras or null if such data doesn't need to be kept.
     * @return number of valid triangulated points which lie in front of the two
     * estimated cameras.
     * @throws InitialCamerasEstimationFailedException if estimation of cameras
     *                                                 fails for some reason, typically due to numerical
     *                                                 instabilities.
     * @throws IllegalArgumentException                if provided lists of left and right
     *                                                 points don't have the same size.
     */
    public static int generateInitialMetricCamerasUsingDIAC(
            final FundamentalMatrix fundamentalMatrix, final double principalPointX,
            final double principalPointY, final List<Point2D> leftPoints,
            final List<Point2D> rightPoints, final CorrectorType correctorType,
            final PinholeCamera leftCamera, final PinholeCamera rightCamera,
            final List<Point3D> triangulatedPoints, final BitSet validTriangulatedPoints)
            throws InitialCamerasEstimationFailedException {

        return generateInitialMetricCamerasUsingDIAC(fundamentalMatrix,
                principalPointX, principalPointY,
                KruppaDualImageOfAbsoluteConicEstimator.DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, leftPoints, rightPoints,
                correctorType, leftCamera, rightCamera, triangulatedPoints, validTriangulatedPoints);
    }

    /**
     * Generates a pair of metric cameras (up to an arbitrary space) by
     * estimating the intrinsic parameters of the views by solving the Kruppa
     * equations to obtain the Dual Image of Absolute Conic (DIAC).
     * The estimated intrinsic parameters can later be used to find the
     * essential matrix (assuming that both views have the same intrinsic
     * parameters), and the essential matrix along with provided matched 2D
     * points can be used to determine the best pair of camera pose and
     * translation that yields the largest number of triangulated points laying
     * in front of both of the estimated cameras.
     * This method uses default corrector type and does not keep triangulated
     * points or valid triangulated points.
     *
     * @param fundamentalMatrix fundamental matrix relating both left and right
     *                          views.
     * @param principalPointX   horizontal coordinate of principal point. This
     *                          value should be the coordinates of the center of an image assuming that
     *                          the coordinates start on the top-left or bottom-left corner. Using a
     *                          value close to zero will produce inaccurate results.
     * @param principalPointY   vertical coordinate of principal point. This
     *                          value should be the coordinates of the center of an image assuming that
     *                          the coordinates start on the top-left or bottom-left corner. Using a
     *                          value close to zero will produce inaccurate results.
     * @param aspectRatio       aspect ratio for estimated intrinsic parameters. This
     *                          is typically 1.0 if vertical coordinates increase upwards or -1.0 if it
     *                          is the opposite.
     * @param leftPoints        points on left view matched with points on right view,
     *                          so they can be triangulated using estimated cameras. Both lists of points
     *                          must have the same size.
     * @param rightPoints       points on right view matched with points on left view,
     *                          so they can be triangulated using estimated cameras. Both lists of points
     *                          must have the same size.
     * @param leftCamera        instance where estimated left camera will be stored.
     * @param rightCamera       instance where estimated right camera will be stored.
     * @return number of valid triangulated points which lie in front of the two
     * estimated cameras.
     * @throws InitialCamerasEstimationFailedException if estimation of cameras
     *                                                 fails for some reason, typically due to numerical
     *                                                 instabilities.
     * @throws IllegalArgumentException                if provided lists of left and right
     *                                                 points don't have the same size.
     */
    public static int generateInitialMetricCamerasUsingDIAC(
            final FundamentalMatrix fundamentalMatrix, final double principalPointX,
            final double principalPointY, final double aspectRatio,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final PinholeCamera leftCamera, final PinholeCamera rightCamera)
            throws InitialCamerasEstimationFailedException {

        return generateInitialMetricCamerasUsingDIAC(fundamentalMatrix, principalPointX, principalPointY, aspectRatio,
                leftPoints, rightPoints, Corrector.DEFAULT_TYPE, leftCamera, rightCamera);
    }

    /**
     * Generates a pair of metric cameras (up to an arbitrary space) by
     * estimating the intrinsic parameters of the views by solving the Kruppa
     * equations to obtain the Dual Image of Absolute Conic (DIAC).
     * The estimated intrinsic parameters can later be used to find the
     * essential matrix (assuming that both views have the same intrinsic
     * parameters), and the essential matrix along with provided matched 2D
     * points can be used to determine the best pair of camera pose and
     * translation that yields the largest number of triangulated points laying
     * in front of both of the estimated cameras.
     * This method does not keep triangulated points or valid triangulated
     * points.
     *
     * @param fundamentalMatrix fundamental matrix relating both left and right
     *                          views.
     * @param principalPointX   horizontal coordinate of principal point. This
     *                          value should be the coordinates of the center of an image assuming that
     *                          the coordinates start on the top-left or bottom-left corner. Using a
     *                          value close to zero will produce inaccurate results.
     * @param principalPointY   vertical coordinate of principal point. This
     *                          value should be the coordinates of the center of an image assuming that
     *                          the coordinates start on the top-left or bottom-left corner. Using a
     *                          value close to zero will produce inaccurate results.
     * @param aspectRatio       aspect ratio for estimated intrinsic parameters. This
     *                          is typically 1.0 if vertical coordinates increase upwards or -1.0 if it
     *                          is the opposite.
     * @param leftPoints        points on left view matched with points on right view,
     *                          so they can be triangulated using estimated cameras. Both lists of points
     *                          must have the same size.
     * @param rightPoints       points on right view matched with points on left view,
     *                          so they can be triangulated using estimated cameras. Both lists of points
     *                          must have the same size.
     * @param correctorType     corrector type to be used to correct 2D points, so
     *                          they follow the epipolar geometry defined by provided fundamental matrix
     *                          so that error on triangulated points is reduced. If null, no corrector
     *                          will be used.
     * @param leftCamera        instance where estimated left camera will be stored.
     * @param rightCamera       instance where estimated right camera will be stored.
     * @return number of valid triangulated points which lie in front of the two
     * estimated cameras.
     * @throws InitialCamerasEstimationFailedException if estimation of
     *                                                 cameras fails for some reason, typically due to
     *                                                 numerical instabilities.
     * @throws IllegalArgumentException                if provided lists of left and right
     *                                                 points don't have the same size.
     */
    public static int generateInitialMetricCamerasUsingDIAC(
            final FundamentalMatrix fundamentalMatrix, final double principalPointX,
            final double principalPointY, final double aspectRatio,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final CorrectorType correctorType, final PinholeCamera leftCamera,
            final PinholeCamera rightCamera) throws InitialCamerasEstimationFailedException {

        return generateInitialMetricCamerasUsingDIAC(fundamentalMatrix, principalPointX, principalPointY, aspectRatio,
                leftPoints, rightPoints, correctorType, leftCamera, rightCamera, null,
                null);
    }

    /**
     * Generates a pair of metric cameras (up to an arbitrary space) by
     * estimating the intrinsic parameters of the views by solving the Kruppa
     * equations to obtain the Dual Image of Absolute Conic (DIAC).
     * The estimated intrinsic parameters can later be used to find the
     * essential matrix (assuming that both views have the same intrinsic
     * parameters), and the essential matrix along with provided matched 2D
     * points can be used to determine the best pair of camera pose and
     * translation that yields the largest number of triangulated points laying
     * in front of both of the estimated cameras.
     * This method uses default corrector type.
     *
     * @param fundamentalMatrix       fundamental matrix relating both left and right
     *                                views.
     * @param principalPointX         horizontal coordinate of principal point. This
     *                                value should be the coordinates of the center of an image assuming
     *                                that the coordinates start on the top-left or bottom-left corner.
     *                                Using a value close to zero will produce inaccurate results.
     * @param principalPointY         vertical coordinate of principal point. This
     *                                value should be the coordinates of the center of an image assuming
     *                                that the coordinates start on the top-left or bottom-left corner.
     *                                Using a value close to zero will produce inaccurate results.
     * @param aspectRatio             aspect ratio for estimated intrinsic parameters. This
     *                                is typically 1.0 if vertical coordinates increase upwards or -1.0 if
     *                                it is the opposite.
     * @param leftPoints              points on left view matched with points on right view,
     *                                so they can be triangulated using estimated cameras. Both lists of
     *                                points must have the same size.
     * @param rightPoints             points on right view matched with points on left view,
     *                                so they can be triangulated using estimated cameras. Both lists of
     *                                points must have the same size.
     * @param leftCamera              instance where estimated left camera will be stored.
     * @param rightCamera             instance where estimated right camera will be stored.
     * @param triangulatedPoints      instance where triangulated 3D points will be
     *                                stored or null if triangulated points don't need to be kept.
     * @param validTriangulatedPoints instance which indicates which
     *                                triangulated 3D points are considered valid because they lie in
     *                                front of both cameras or null if such data doesn't need to be kept.
     * @return number of valid triangulated points which lie in front of the two
     * estimated cameras.
     * @throws InitialCamerasEstimationFailedException if estimation of cameras
     *                                                 fails for some reason, typically due to numerical
     *                                                 instabilities.
     * @throws IllegalArgumentException                if provided lists of left and right
     *                                                 points don't have the same size.
     */
    public static int generateInitialMetricCamerasUsingDIAC(
            final FundamentalMatrix fundamentalMatrix, final double principalPointX,
            final double principalPointY, final double aspectRatio,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final PinholeCamera leftCamera, final PinholeCamera rightCamera,
            final List<Point3D> triangulatedPoints, final BitSet validTriangulatedPoints)
            throws InitialCamerasEstimationFailedException {

        return generateInitialMetricCamerasUsingDIAC(fundamentalMatrix, principalPointX, principalPointY, aspectRatio,
                leftPoints, rightPoints, Corrector.DEFAULT_TYPE, leftCamera, rightCamera, triangulatedPoints,
                validTriangulatedPoints);
    }

    /**
     * Generates a pair of metric cameras (up to an arbitrary space) by
     * estimating the intrinsic parameters of the views by solving the Kruppa
     * equations to obtain the Dual Image of Absolute Conic (DIAC).
     * The estimated intrinsic parameters can later be used to find the
     * essential matrix (assuming that both views have the same intrinsic
     * parameters), and the essential matrix along with provided matched 2D
     * points can be used to determine the best pair of camera pose and
     * translation that yields the largest number of triangulated points laying
     * in front of both of the estimated cameras.
     *
     * @param fundamentalMatrix       fundamental matrix relating both left and right
     *                                views.
     * @param principalPointX         horizontal coordinate of principal point. This
     *                                value should be the coordinates of the center of an image assuming
     *                                that the coordinates start on the top-left or bottom-left corner.
     *                                Using a value close to zero will produce inaccurate results.
     * @param principalPointY         vertical coordinate of principal point. This
     *                                value should be the coordinates of the center of an image assuming
     *                                that the coordinates start on the top-left or bottom-left corner.
     *                                Using a value close to zero will produce inaccurate results.
     * @param aspectRatio             aspect ratio for estimated intrinsic parameters. This
     *                                is typically 1.0 if vertical coordinates increase upwards or -1.0 if
     *                                it is the opposite.
     * @param leftPoints              points on left view matched with points on right view,
     *                                so they can be triangulated using estimated cameras. Both lists of
     *                                points must have the same size.
     * @param rightPoints             points on right view matched with points on left view,
     *                                so they can be triangulated using estimated cameras. Both lists of
     *                                points must have the same size.
     * @param correctorType           corrector type to be used to correct 2D points, so
     *                                they follow the epipolar geometry defined by provided fundamental
     *                                matrix so that error on triangulated points is reduced. If null, no
     *                                corrector will be used.
     * @param leftCamera              instance where estimated left camera will be stored.
     * @param rightCamera             instance where estimated right camera will be stored.
     * @param triangulatedPoints      instance where triangulated 3D points will be
     *                                stored or null if triangulated points don't need to be kept.
     * @param validTriangulatedPoints instance which indicates which
     *                                triangulated 3D points are considered valid because they lie in
     *                                front of both cameras or null if such data doesn't need to be kept.
     * @return number of valid triangulated points which lie in front of the two
     * estimated cameras.
     * @throws InitialCamerasEstimationFailedException if estimation of cameras
     *                                                 fails for some reason, typically due to numerical
     *                                                 instabilities.
     * @throws IllegalArgumentException                if provided lists of left and right
     *                                                 points don't have the same size.
     */
    public static int generateInitialMetricCamerasUsingDIAC(
            final FundamentalMatrix fundamentalMatrix, final double principalPointX,
            final double principalPointY, final double aspectRatio,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final CorrectorType correctorType, final PinholeCamera leftCamera,
            final PinholeCamera rightCamera, final List<Point3D> triangulatedPoints,
            final BitSet validTriangulatedPoints) throws InitialCamerasEstimationFailedException {

        try {
            final KruppaDualImageOfAbsoluteConicEstimator diacEstimator = new KruppaDualImageOfAbsoluteConicEstimator(
                    fundamentalMatrix);
            diacEstimator.setPrincipalPointX(principalPointX);
            diacEstimator.setPrincipalPointY(principalPointY);
            diacEstimator.setFocalDistanceAspectRatioKnown(true);
            diacEstimator.setFocalDistanceAspectRatio(aspectRatio);

            final var diac = diacEstimator.estimate();

            final var intrinsic = diac.getIntrinsicParameters();

            return EssentialMatrixInitialCamerasEstimator.generateInitialMetricCamerasFromEssentialMatrix(
                    fundamentalMatrix, intrinsic, intrinsic, leftPoints, rightPoints, correctorType, leftCamera,
                    rightCamera, triangulatedPoints, validTriangulatedPoints);

        } catch (final InitialCamerasEstimationFailedException e) {
            throw e;
        } catch (final Exception e) {
            throw new InitialCamerasEstimationFailedException(e);
        }
    }

    /**
     * Internal method to set matched 2D points on left and right views.
     * This method does not check whether the estimator is locked or not, only
     * ensures that provided lists have the same size.
     *
     * @param leftPoints  matched 2D points on left view.
     * @param rightPoints matched 2D points on right view.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size.
     */
    private void internalSetLeftAndRightPoints(final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        if (leftPoints == null || rightPoints == null || leftPoints.size() != rightPoints.size()) {
            throw new IllegalArgumentException();
        }
        this.leftPoints = leftPoints;
        this.rightPoints = rightPoints;
    }
}
