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
import com.irurueta.ar.epipolar.Corrector;
import com.irurueta.ar.epipolar.CorrectorType;
import com.irurueta.ar.epipolar.EssentialMatrix;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.geometry.CameraException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * Estimates an initial pair of cameras in the metric stratum (up to an
 * arbitrary scale) using a given fundamental matrix and provided intrinsic
 * parameters on left and right views (which can be obtained by offline
 * calibration) to compute the essential matrix and choose the best combination
 * of rotation and translation on estimated cameras so that triangulated 3D
 * points obtained from provided matched 2D points are located in front of
 * the estimated cameras.
 */
@SuppressWarnings("DuplicatedCode")
public class EssentialMatrixInitialCamerasEstimator
        extends InitialCamerasEstimator {

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
     * Intrinsic parameters to be used for estimated left camera.
     */
    private PinholeCameraIntrinsicParameters mLeftIntrinsic;

    /**
     * Intrinsic parameters to be used for estimated right camera.
     */
    private PinholeCameraIntrinsicParameters mRightIntrinsic;

    /**
     * Matched 2D points on left view.
     */
    private List<Point2D> mLeftPoints;

    /**
     * Matched 2D points on right view.
     */
    private List<Point2D> mRightPoints;

    /**
     * Type of corrector to use to triangulate matched points or null if no
     * corrector needs to be used.
     */
    private CorrectorType mCorrectorType = Corrector.DEFAULT_TYPE;

    /**
     * Indicates whether matched 2D points need to be triangulated.
     */
    private boolean mTriangulatePoints = DEFAULT_TRIANGULATE_POINTS;

    /**
     * Marks which of the triangulated points are marked as valid (lie in front
     * of both of the estimated cameras) and which ones aren't.
     */
    private boolean mMarkValidTriangulatedPoints =
            DEFAULT_MARK_VALID_TRIANGULATED_POINTS;

    /**
     * Contains triangulated points.
     */
    private List<Point3D> mTriangulatedPoints;

    /**
     * Contains booleans indicating whether triangulated points are valid (i.e.
     * lie in front of both estimated cameras) or not.
     */
    private BitSet mValidTriangulatedPoints;

    /**
     * Constructor.
     */
    public EssentialMatrixInitialCamerasEstimator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     */
    public EssentialMatrixInitialCamerasEstimator(
            final FundamentalMatrix fundamentalMatrix) {
        super(fundamentalMatrix);
    }

    /**
     * Constructor.
     *
     * @param leftIntrinsic  intrinsic parameters to be used for estimated left
     *                       camera.
     * @param rightIntrinsic intrinsic parameters to be used for estimated right
     *                       camera.
     */
    public EssentialMatrixInitialCamerasEstimator(
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic) {
        super();
        mLeftIntrinsic = leftIntrinsic;
        mRightIntrinsic = rightIntrinsic;
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param leftIntrinsic     intrinsic parameters to be used for estimated left
     *                          camera.
     * @param rightIntrinsic    intrinsic parameters to be used for estimated right
     *                          camera.
     */
    public EssentialMatrixInitialCamerasEstimator(
            final FundamentalMatrix fundamentalMatrix,
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic) {
        super(fundamentalMatrix);
        mLeftIntrinsic = leftIntrinsic;
        mRightIntrinsic = rightIntrinsic;
    }

    /**
     * Constructor.
     *
     * @param leftPoints  matched 2D points on left view.
     * @param rightPoints matched 2D points on right view.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size.
     */
    public EssentialMatrixInitialCamerasEstimator(final List<Point2D> leftPoints,
                                                  final List<Point2D> rightPoints) {
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
    public EssentialMatrixInitialCamerasEstimator(
            final FundamentalMatrix fundamentalMatrix, final List<Point2D> leftPoints,
            final List<Point2D> rightPoints) {
        super(fundamentalMatrix);
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Constructor.
     *
     * @param leftIntrinsic  intrinsic parameters to be used for estimated left
     *                       camera.
     * @param rightIntrinsic intrinsic parameters to be used for estimated right
     *                       camera.
     * @param leftPoints     matched 2D points on left view.
     * @param rightPoints    matched 2D points on right view.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size.
     */
    public EssentialMatrixInitialCamerasEstimator(
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        this(leftIntrinsic, rightIntrinsic);
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param leftIntrinsic     intrinsic parameters to be used for estimated left
     *                          camera.
     * @param rightIntrinsic    intrinsic parameters to be used for estimated right
     *                          camera.
     * @param leftPoints        matched 2D points on left view.
     * @param rightPoints       matched 2D points on right view.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size.
     */
    public EssentialMatrixInitialCamerasEstimator(
            final FundamentalMatrix fundamentalMatrix,
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        this(fundamentalMatrix, leftIntrinsic, rightIntrinsic);
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this instance.
     */
    public EssentialMatrixInitialCamerasEstimator(
            final InitialCamerasEstimatorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param listener          listener to handle events raised by this instance.
     */
    public EssentialMatrixInitialCamerasEstimator(
            final FundamentalMatrix fundamentalMatrix,
            final InitialCamerasEstimatorListener listener) {
        super(fundamentalMatrix, listener);
    }

    /**
     * Constructor.
     *
     * @param leftIntrinsic  intrinsic parameters to be used for estimated left
     *                       camera.
     * @param rightIntrinsic intrinsic parameters to be used for estimated right
     *                       camera.
     * @param listener       listener to handle events raised by this instance.
     */
    public EssentialMatrixInitialCamerasEstimator(
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic,
            final InitialCamerasEstimatorListener listener) {
        super(listener);
        mLeftIntrinsic = leftIntrinsic;
        mRightIntrinsic = rightIntrinsic;
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param leftIntrinsic     intrinsic parameters to be used for estimated left
     *                          camera.
     * @param rightIntrinsic    intrinsic parameters to be used for estimated right
     *                          camera.
     * @param listener          listener to handle events raised by this instance.
     */
    public EssentialMatrixInitialCamerasEstimator(
            final FundamentalMatrix fundamentalMatrix,
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic,
            final InitialCamerasEstimatorListener listener) {
        super(fundamentalMatrix, listener);
        mLeftIntrinsic = leftIntrinsic;
        mRightIntrinsic = rightIntrinsic;
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
    public EssentialMatrixInitialCamerasEstimator(
            final List<Point2D> leftPoints,
            final List<Point2D> rightPoints,
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
    public EssentialMatrixInitialCamerasEstimator(
            final FundamentalMatrix fundamentalMatrix,
            final List<Point2D> leftPoints,
            final List<Point2D> rightPoints,
            final InitialCamerasEstimatorListener listener) {
        super(fundamentalMatrix, listener);
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Constructor.
     *
     * @param leftIntrinsic  intrinsic parameters to be used for estimated left
     *                       camera.
     * @param rightIntrinsic intrinsic parameters to be used for estimated right
     *                       camera.
     * @param leftPoints     matched 2D points on left view.
     * @param rightPoints    matched 2D points on right view.
     * @param listener       listener to handle events raised by this instance.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size.
     */
    public EssentialMatrixInitialCamerasEstimator(
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic,
            final List<Point2D> leftPoints,
            final List<Point2D> rightPoints,
            final InitialCamerasEstimatorListener listener) {
        this(leftIntrinsic, rightIntrinsic, listener);
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param leftIntrinsic     intrinsic parameters to be used for estimated left
     *                          camera.
     * @param rightIntrinsic    intrinsic parameters to be used for estimated right
     *                          camera.
     * @param leftPoints        matched 2D points on left view.
     * @param rightPoints       matched 2D points on right view.
     * @param listener          listener to handle events raised by this instance.
     * @throws IllegalArgumentException if provided lists don't have the same
     *                                  size.
     */
    public EssentialMatrixInitialCamerasEstimator(
            final FundamentalMatrix fundamentalMatrix,
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic,
            final List<Point2D> leftPoints,
            final List<Point2D> rightPoints,
            final InitialCamerasEstimatorListener listener) {
        this(fundamentalMatrix, leftIntrinsic, rightIntrinsic, listener);
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Returns method used by this estimator.
     *
     * @return method used by this estimator.
     */
    @Override
    public InitialCamerasEstimatorMethod getMethod() {
        return InitialCamerasEstimatorMethod.ESSENTIAL_MATRIX;
    }

    /**
     * Indicates if estimator is ready.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mFundamentalMatrix != null && mLeftIntrinsic != null &&
                mRightIntrinsic != null && mLeftPoints != null &&
                mRightPoints != null &&
                mLeftPoints.size() == mRightPoints.size();
    }

    /**
     * Estimates cameras.
     *
     * @throws LockedException                         if estimator is locked.
     * @throws NotReadyException                       if estimator is not ready.
     * @throws InitialCamerasEstimationFailedException if estimation of cameras
     *                                                 fails for some reason, typically due to numerical unstabilities.
     */
    @Override
    public void estimate() throws LockedException, NotReadyException,
            InitialCamerasEstimationFailedException {
        if (isLocked()) {
            throw new LockedException();
        }

        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            mLocked = true;

            if (mListener != null) {
                mListener.onStart(this);
            }

            if (mTriangulatePoints) {
                mTriangulatedPoints = new ArrayList<>();
            } else {
                mTriangulatedPoints = null;
            }

            final int nPoints = mLeftPoints.size();
            if (mMarkValidTriangulatedPoints) {
                mValidTriangulatedPoints = new BitSet(nPoints);
            } else {
                mValidTriangulatedPoints = null;
            }

            if (mEstimatedLeftCamera == null) {
                mEstimatedLeftCamera = new PinholeCamera();
            }
            if (mEstimatedRightCamera == null) {
                mEstimatedRightCamera = new PinholeCamera();
            }

            generateInitialMetricCamerasFromEssentialMatrix(mFundamentalMatrix,
                    mLeftIntrinsic, mRightIntrinsic, mLeftPoints, mRightPoints,
                    mCorrectorType, mEstimatedLeftCamera, mEstimatedRightCamera,
                    mTriangulatedPoints, mValidTriangulatedPoints);

            if (mListener != null) {
                mListener.onFinish(this, mEstimatedLeftCamera,
                        mEstimatedRightCamera);
            }
        } catch (final InitialCamerasEstimationFailedException e) {
            if (mListener != null) {
                mListener.onFail(this, e);
            }
            throw e;
        } finally {
            mLocked = false;
        }
    }

    /**
     * Gets intrinsic parameters to be used for estimated left camera.
     *
     * @return intrinsic parameters to be used for estimated left camera.
     */
    public PinholeCameraIntrinsicParameters getLeftIntrinsic() {
        return mLeftIntrinsic;
    }

    /**
     * Sets intrinsic parameters to be used for estimated left camera.
     *
     * @param leftIntrinsic intrinsic parameters to be used for estimated left
     *                      camera.
     * @throws LockedException if estimator is locked.
     */
    public void setLeftIntrinsic(
            final PinholeCameraIntrinsicParameters leftIntrinsic)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mLeftIntrinsic = leftIntrinsic;
    }

    /**
     * Gets intrinsic parameters to be used for estimated right camera.
     *
     * @return intrinsic parameters to be used for estimated right camera.
     */
    public PinholeCameraIntrinsicParameters getRightIntrinsic() {
        return mRightIntrinsic;
    }

    /**
     * Sets intrinsic parameters to be used for estimated right camera.
     *
     * @param rightIntrinsic intrinsic parameters to be used for estimated right
     *                       camera.
     * @throws LockedException if estimator is locked.
     */
    public void setRightIntrinsic(
            final PinholeCameraIntrinsicParameters rightIntrinsic)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRightIntrinsic = rightIntrinsic;
    }

    /**
     * Sets intrinsic parameters to be used for estimated left and right
     * cameras.
     *
     * @param leftIntrinsic  intrinsic parameters to be used for estimated left
     *                       camera.
     * @param rightIntrinsic intrinsic parameters to be used for estimated right
     *                       camera.
     * @throws LockedException if estimator is locked.
     */
    public void setLeftAndRightIntrinsics(
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mLeftIntrinsic = leftIntrinsic;
        mRightIntrinsic = rightIntrinsic;
    }

    /**
     * Sets the same intrinsic parameters to be used for both estimated left
     * and right cameras.
     *
     * @param intrinsic intrinsic parameters to be used for both cameras.
     * @throws LockedException if estimator is locked.
     */
    public void setIntrinsicsForBoth(final PinholeCameraIntrinsicParameters intrinsic)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mLeftIntrinsic = mRightIntrinsic = intrinsic;
    }

    /**
     * Gets matched 2D points on left view.
     *
     * @return matched 2D points on left view.
     */
    public List<Point2D> getLeftPoints() {
        return mLeftPoints;
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
        mLeftPoints = leftPoints;
    }

    /**
     * Gets matched 2D points on right view.
     *
     * @return matched 2D points on right view.
     */
    public List<Point2D> getRightPoints() {
        return mRightPoints;
    }

    /**
     * Sets matched 2D points on right view.
     *
     * @param rightPoints matched 2D points on right view.
     * @throws LockedException if estimator is locked.
     */
    public void setRightPoints(final List<Point2D> rightPoints)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mRightPoints = rightPoints;
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
    public void setLeftAndRightPoints(
            final List<Point2D> leftPoints,
            final List<Point2D> rightPoints) throws LockedException {
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
        return mCorrectorType;
    }

    /**
     * Sets type of corrector to use to triangulate matched points or null if
     * no corrector needs to be used.
     *
     * @param correctorType type of corrector to use.
     * @throws LockedException if estimator is locked.
     */
    public void setCorrectorType(final CorrectorType correctorType)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mCorrectorType = correctorType;
    }

    /**
     * Indicates whether matched 2D points need to be triangulated or not.
     *
     * @return true if 2D points need to be triangulated, false otherwise.
     */
    public boolean arePointsTriangulated() {
        return mTriangulatePoints;
    }

    /**
     * Specifies whether matched 2D points need to be triangulated or not.
     *
     * @param triangulatePoints true if 2D points need to be triangulated, false
     *                          otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setPointsTriangulated(final boolean triangulatePoints)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mTriangulatePoints = triangulatePoints;
    }

    /**
     * Indicates which triangulated points are marked as valid (lie in front
     * of both of the estimated cameras) and which ones aren't.
     *
     * @return true to mark valid and invalid triangulated points, false
     * otherwise.
     */
    public boolean areValidTriangulatedPointsMarked() {
        return mMarkValidTriangulatedPoints;
    }

    /**
     * Specifies whether triangulated points are marked as valid (lie in front
     * of both of the estimated cameras) and which ones aren't.
     *
     * @param markValidTriangulatedPoints true to mark valid and invalid
     *                                    triangulated points, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setValidTriangulatedPointsMarked(
            final boolean markValidTriangulatedPoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mMarkValidTriangulatedPoints = markValidTriangulatedPoints;
    }

    /**
     * Gets triangulated points, if available.
     *
     * @return triangulated points or null.
     */
    public List<Point3D> getTriangulatedPoints() {
        return mTriangulatedPoints;
    }

    /**
     * Gets bitset indicating which of the triangulated points are valid and
     * which ones aren't.
     *
     * @return bitset indicating validity of triangulated points or null if not
     * available.
     */
    public BitSet getValidTriangulatedPoints() {
        return mValidTriangulatedPoints;
    }

    /**
     * Generates a pair of metric cameras (up to an arbitrary space) by
     * computing the essential matrix from provided fundamental matrix and
     * intrinsic parameters of left and right cameras, and choosing the best
     * pair of camera pose and translation that yields the largest number of
     * triangulated points laying in front of both of the estimated cameras.
     * This method uses default corrector type and does not keep triangulated
     * points or valid triangulated points.
     *
     * @param fundamentalMatrix fundamental matrix relating both left and right
     *                          views.
     * @param leftIntrinsic     intrinsic parameters to be set on left view.
     *                          This can be used when cameras have been previously calibrated.
     * @param rightIntrinsic    intrinsic parameters to be set on right view.
     *                          This can be used when cameras have been previously calibrated.
     * @param leftPoints        points on left view matched with points on right view
     *                          so they can be triangulated using estimated cameras. Both lists of points
     *                          must have the same size.
     * @param rightPoints       points on right view matched with points on left view
     *                          so they can be triangulated using estimated cameras. Both lists of points
     *                          must have the same size.
     * @param leftCamera        instance where estimated left camera will be stored.
     * @param rightCamera       instance where estimated right camera will be stored.
     * @return number of valid triangulated points which lie in front of the two
     * estimated cameras.
     * @throws InitialCamerasEstimationFailedException if estimation of cameras
     *                                                 fails for some reason, typically due to numerical unstabilities.
     * @throws IllegalArgumentException                if provided lists of left and right
     *                                                 points don't have the same size.
     */
    public static int generateInitialMetricCamerasFromEssentialMatrix(
            final FundamentalMatrix fundamentalMatrix,
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final PinholeCamera leftCamera, final PinholeCamera rightCamera)
            throws InitialCamerasEstimationFailedException {
        return generateInitialMetricCamerasFromEssentialMatrix(
                fundamentalMatrix, leftIntrinsic, rightIntrinsic, leftPoints,
                rightPoints, Corrector.DEFAULT_TYPE, leftCamera, rightCamera);
    }

    /**
     * Generates a pair of metric cameras (up to an arbitrary space) by
     * computing the essential matrix from provided fundamental matrix and
     * intrinsic parameters of left and right cameras, and choosing the best
     * pair of camera pose and translation that yields the largest number of
     * triangulated points laying in front of both of the estimated cameras.
     * This method does not keep triangulated points or valid triangulated
     * points.
     *
     * @param fundamentalMatrix fundamental matrix relating both left and right
     *                          views.
     * @param leftIntrinsic     intrinsic parameters to be set on left view.
     *                          This can be used when cameras have been previously calibrated.
     * @param rightIntrinsic    intrinsic parameters to be set on right view.
     *                          This can be used when cameras have been previously calibrated.
     * @param leftPoints        points on left view matched with points on right view
     *                          so they can be triangulated using estimated cameras. Both lists of points
     *                          must have the same size.
     * @param rightPoints       points on right view matched with points on left view
     *                          so they can be triangulated using estimated cameras. Both lists of points
     *                          must have the same size.
     * @param correctorType     corrector type to be used to correct 2D points so
     *                          they follow the epipolar geometry defined by provided fundamental matrix
     *                          so that error on triangulated points is reduced. If null, no corrector
     *                          will be used.
     * @param leftCamera        instance where estimated left camera will be stored.
     * @param rightCamera       instance where estimated right camera will be stored.
     * @return number of valid triangulated points which lie in front of the two
     * estimated cameras.
     * @throws InitialCamerasEstimationFailedException if estimation of
     *                                                 cameras fails for some reason, typically due to numerical unstabilities.
     * @throws IllegalArgumentException                if provided lists of left and right
     *                                                 points don't have the same size.
     */
    public static int generateInitialMetricCamerasFromEssentialMatrix(
            final FundamentalMatrix fundamentalMatrix,
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final CorrectorType correctorType, final PinholeCamera leftCamera,
            final PinholeCamera rightCamera) throws InitialCamerasEstimationFailedException {
        return generateInitialMetricCamerasFromEssentialMatrix(
                fundamentalMatrix, leftIntrinsic, rightIntrinsic, leftPoints,
                rightPoints, correctorType, leftCamera, rightCamera, null,
                null);
    }

    /**
     * Generates a pair of metric cameras (up to an arbitrary space) by
     * computing the essential matrix from provided fundamental matrix and
     * intrinsic parameters of left and right cameras, and choosing the best
     * pair of camera pose and translation that yields the largest number of
     * triangulated points laying in front of both of the estimated cameras.
     * This method uses default corrector type.
     *
     * @param fundamentalMatrix       fundamental matrix relating both left and right
     *                                views.
     * @param leftIntrinsic           intrinsic parameters to be set on left view.
     *                                This can be used when cameras have been previously calibrated.
     * @param rightIntrinsic          intrinsic parameters to be set on right view.
     *                                This can be used when cameras have been previously calibrated.
     * @param leftPoints              points on left view matched with points on right view
     *                                so they can be triangulated using estimated cameras. Both lists of points
     *                                must have the same size.
     * @param rightPoints             points on right view matched with points on left view
     *                                so they can be triangulated using estimated cameras. Both lists of points
     *                                must have the same size.
     * @param leftCamera              instance where estimated left camera will be stored.
     * @param rightCamera             instance where estimated right camera will be stored.
     * @param triangulatedPoints      instance where triangulated 3D points will be
     *                                stored or null if triangulated points don't need to be kept.
     * @param validTriangulatedPoints instance which indicates which
     *                                triangulated 3D points are considered valid because they lie in front of
     *                                both cameras or null if such data doesn't need to be kept.
     * @return number of valid triangulated points which lie in front of the two
     * estimated cameras.
     * @throws InitialCamerasEstimationFailedException if estimation of cameras
     *                                                 fails for some reason, typically due to numerical unstabilities.
     * @throws IllegalArgumentException                if provided lists of left and right
     *                                                 points don't have the same size.
     */
    public static int generateInitialMetricCamerasFromEssentialMatrix(
            final FundamentalMatrix fundamentalMatrix,
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final PinholeCamera leftCamera, final PinholeCamera rightCamera,
            final List<Point3D> triangulatedPoints, final BitSet validTriangulatedPoints)
            throws InitialCamerasEstimationFailedException {

        return generateInitialMetricCamerasFromEssentialMatrix(
                fundamentalMatrix, leftIntrinsic, rightIntrinsic, leftPoints,
                rightPoints, Corrector.DEFAULT_TYPE, leftCamera, rightCamera,
                triangulatedPoints, validTriangulatedPoints);
    }

    /**
     * Generates a pair of metric cameras (up to an arbitrary space) by
     * computing the essential matrix from provided fundamental matrix and
     * intrinsic parameters of left and right cameras, and choosing the best
     * pair of camera pose and translation that yields the largest number of
     * triangulated points laying in front of both of the estimated cameras.
     *
     * @param fundamentalMatrix       fundamental matrix relating both left and right
     *                                views.
     * @param leftIntrinsic           intrinsic parameters to be set on left view.
     *                                This can be used when cameras have been previously calibrated.
     * @param rightIntrinsic          intrinsic parameters to be set on right view.
     *                                This can be used when cameras have been previously calibrated.
     * @param leftPoints              points on left view matched with points on right view
     *                                so they can be triangulated using estimated cameras. Both lists of points
     *                                must have the same size.
     * @param rightPoints             points on right view matched with points on left view
     *                                so they can be triangulated using estimated cameras. Both lists of points
     *                                must have the same size.
     * @param correctorType           corrector type to be used to correct 2D points so
     *                                they follow the epipolar geometry defined by provided fundamental matrix
     *                                so that error on triangulated points is reduced. If null, no corrector
     *                                will be used.
     * @param leftCamera              instance where estimated left camera will be stored.
     * @param rightCamera             instance where estimated right camera will be stored.
     * @param triangulatedPoints      instance where triangulated 3D points will be
     *                                stored or null if triangulated points don't need to be kept.
     * @param validTriangulatedPoints instance which indicates which
     *                                triangulated 3D points are considered valid because they lie in front of
     *                                both cameras or null if such data doesn't need to be kept.
     * @return number of valid triangulated points which lie in front of the two
     * estimated cameras.
     * @throws InitialCamerasEstimationFailedException if estimation of
     *                                                 cameras fails for some reason, typically due to numerical unstabilities.
     * @throws IllegalArgumentException                if provided lists of left and right
     *                                                 points don't have the same size.
     */
    public static int generateInitialMetricCamerasFromEssentialMatrix(
            final FundamentalMatrix fundamentalMatrix,
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic,
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final CorrectorType correctorType, final PinholeCamera leftCamera,
            final PinholeCamera rightCamera, final List<Point3D> triangulatedPoints,
            final BitSet validTriangulatedPoints)
            throws InitialCamerasEstimationFailedException {

        if (leftPoints.size() != rightPoints.size()) {
            throw new IllegalArgumentException(
                    "left and right points must have the same size");
        }

        final List<Point2D> correctedLeftPoints;
        final List<Point2D> correctedRightPoints;
        final Rotation3D rotation1;
        final Rotation3D rotation2;
        final Point2D translation1;
        final Point2D translation2;
        try {
            final EssentialMatrix essential = new EssentialMatrix(fundamentalMatrix,
                    leftIntrinsic, rightIntrinsic);

            essential.computePossibleRotationAndTranslations();

            rotation1 = essential.getFirstPossibleRotation();
            translation1 = essential.getFirstPossibleTranslation();

            rotation2 = essential.getSecondPossibleRotation();
            translation2 = essential.getSecondPossibleTranslation();

            if (correctorType != null) {
                // use corrector
                final Corrector corrector = Corrector.create(leftPoints, rightPoints,
                        fundamentalMatrix, correctorType);
                corrector.correct();

                correctedLeftPoints = corrector.getLeftCorrectedPoints();
                correctedRightPoints = corrector.getRightCorrectedPoints();
            } else {
                // don't use corrector
                correctedLeftPoints = leftPoints;
                correctedRightPoints = rightPoints;
            }
        } catch (final Exception e) {
            throw new InitialCamerasEstimationFailedException(e);
        }

        if (triangulatedPoints != null) {
            triangulatedPoints.clear();
        }
        if (validTriangulatedPoints != null) {
            validTriangulatedPoints.clear();
        }
        int numValidTriangulatedPoints;

        final int numPoints = correctedLeftPoints.size();
        boolean skip = false;

        // obtain 1st pair of possible cameras and their corresponding
        // triangulated points
        try {
            numValidTriangulatedPoints =
                    computeCamerasAndTriangulation(rotation1, translation1,
                            leftIntrinsic, rightIntrinsic, correctedLeftPoints,
                            correctedRightPoints, leftCamera, rightCamera,
                            triangulatedPoints, validTriangulatedPoints);
        } catch (final Exception e) {
            numValidTriangulatedPoints = 0;
        }

        if (numValidTriangulatedPoints >= numPoints) {
            // all points are valid, hence, we can set current pair of
            // cameras as the best result
            skip = true;
        }

        final PinholeCamera attemptLeftCamera = new PinholeCamera();
        final PinholeCamera attemptRightCamera = new PinholeCamera();
        List<Point3D> attemptTriangulatedPoints = null;
        if (triangulatedPoints != null) {
            attemptTriangulatedPoints = new ArrayList<>();
        }
        BitSet attemptValidTriangulatedPoints = null;
        if (validTriangulatedPoints != null) {
            attemptValidTriangulatedPoints = new BitSet(numPoints);
        }
        int attemptNumValidTriangulatedPoints;

        if (!skip) {
            // obtain 2nd pair of possible cameras and their corresponding
            // triangulated points
            try {
                attemptNumValidTriangulatedPoints =
                        computeCamerasAndTriangulation(rotation1, translation2,
                                leftIntrinsic, rightIntrinsic, correctedLeftPoints,
                                correctedRightPoints, attemptLeftCamera,
                                attemptRightCamera, attemptTriangulatedPoints,
                                attemptValidTriangulatedPoints);
            } catch (final Exception e) {
                attemptNumValidTriangulatedPoints = 0;
            }

            if (attemptNumValidTriangulatedPoints >= numPoints) {
                // all points are valid, hence, we can set current pair of
                // cameras as the best result
                skip = true;
            }

            if (attemptNumValidTriangulatedPoints > numValidTriangulatedPoints) {
                // a better solution containing more valid points has been foound

                // keep better solution
                updateBestSolutionData(leftCamera, rightCamera,
                        triangulatedPoints, validTriangulatedPoints,
                        attemptLeftCamera, attemptRightCamera,
                        attemptTriangulatedPoints,
                        attemptValidTriangulatedPoints);
                numValidTriangulatedPoints = attemptNumValidTriangulatedPoints;
            }
        }

        if (!skip) {
            // obtain 3rd pair of possible cameras and their corresponding
            // triangulated points
            try {
                attemptNumValidTriangulatedPoints =
                        computeCamerasAndTriangulation(rotation2, translation1,
                                leftIntrinsic, rightIntrinsic, correctedLeftPoints,
                                correctedRightPoints, attemptLeftCamera,
                                attemptRightCamera, attemptTriangulatedPoints,
                                attemptValidTriangulatedPoints);
            } catch (final Exception e) {
                attemptNumValidTriangulatedPoints = 0;
            }

            if (attemptNumValidTriangulatedPoints >= numPoints) {
                // all points are valid, hence, we can set current pair of
                // cameras as the best result
                skip = true;
            }

            if (attemptNumValidTriangulatedPoints > numValidTriangulatedPoints) {
                // a better solution containing more valid points has been foound

                // keep better solution
                updateBestSolutionData(leftCamera, rightCamera,
                        triangulatedPoints, validTriangulatedPoints,
                        attemptLeftCamera, attemptRightCamera,
                        attemptTriangulatedPoints,
                        attemptValidTriangulatedPoints);
                numValidTriangulatedPoints = attemptNumValidTriangulatedPoints;
            }
        }

        if (!skip) {
            // obtain 4th pair of possible cameras and their corresponding
            // triangulated points
            try {
                attemptNumValidTriangulatedPoints =
                        computeCamerasAndTriangulation(rotation2, translation2,
                                leftIntrinsic, rightIntrinsic, correctedLeftPoints,
                                correctedRightPoints, attemptLeftCamera,
                                attemptRightCamera, attemptTriangulatedPoints,
                                attemptValidTriangulatedPoints);
            } catch (final Exception e) {
                attemptNumValidTriangulatedPoints = 0;
            }

            if (attemptNumValidTriangulatedPoints > numValidTriangulatedPoints) {
                // a better solution containing more valid points has been foound

                // keep better solution
                updateBestSolutionData(leftCamera, rightCamera,
                        triangulatedPoints, validTriangulatedPoints,
                        attemptLeftCamera, attemptRightCamera,
                        attemptTriangulatedPoints,
                        attemptValidTriangulatedPoints);
                numValidTriangulatedPoints = attemptNumValidTriangulatedPoints;
            }
        }

        if (numValidTriangulatedPoints == 0) {
            throw new InitialCamerasEstimationFailedException(
                    "no valid points found");
        }

        return numValidTriangulatedPoints;
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
    private void internalSetLeftAndRightPoints(final List<Point2D> leftPoints,
                                               final List<Point2D> rightPoints) {
        if (leftPoints == null || rightPoints == null ||
                leftPoints.size() != rightPoints.size()) {
            throw new IllegalArgumentException();
        }
        mLeftPoints = leftPoints;
        mRightPoints = rightPoints;
    }

    /**
     * Updates data for best solution found so far.
     *
     * @param leftCamera                     instance where best found left camera will be stored.
     * @param rightCamera                    instance where best found right camera will be stored.
     * @param triangulatedPoints             instance where triangulated points for best
     *                                       found solution will be stored or null if points don't need to be kept.
     * @param validTriangulatedPoints        instance where valid triangulated points
     *                                       for best found solution will be stored or null if such data doesn't need
     *                                       to be kept.
     * @param attemptLeftCamera              estimated left camera to be copied into best
     *                                       solution.
     * @param attemptRightCamera             estimated right camera to be copied into best
     *                                       solution.
     * @param attemptTriangulatedPoints      triangulated points to be copied into
     *                                       best solution, or null if nothing needs to be copied.
     * @param attemptValidTriangulatedPoints valid triangulated points to be
     *                                       copied into best solution, or null if nothing needs to be copied.
     * @throws InitialCamerasEstimationFailedException if something fails.
     */
    private static void updateBestSolutionData(
            final PinholeCamera leftCamera,
            final PinholeCamera rightCamera,
            final List<Point3D> triangulatedPoints,
            final BitSet validTriangulatedPoints,
            final PinholeCamera attemptLeftCamera,
            final PinholeCamera attemptRightCamera,
            final List<Point3D> attemptTriangulatedPoints,
            final BitSet attemptValidTriangulatedPoints)
            throws InitialCamerasEstimationFailedException {

        try {
            leftCamera.setInternalMatrix(attemptLeftCamera.getInternalMatrix());
            rightCamera.setInternalMatrix(
                    attemptRightCamera.getInternalMatrix());
            if (triangulatedPoints != null &&
                    attemptTriangulatedPoints != null) {
                triangulatedPoints.clear();
                triangulatedPoints.addAll(attemptTriangulatedPoints);
            }
            if (validTriangulatedPoints != null &&
                    attemptValidTriangulatedPoints != null) {
                validTriangulatedPoints.clear();
                validTriangulatedPoints.or(attemptValidTriangulatedPoints);
            }
        } catch (final WrongSizeException e) {
            throw new InitialCamerasEstimationFailedException(e);
        }
    }

    /**
     * Computes a pair of cameras for provided rotation and translation
     * using provided intrinsic parameters.
     * This method also triangulates proved matched 2D points and determines
     * how many of them lie in front of both estimated cameras.
     *
     * @param rotation                rotation between estimated left and right cameras.
     * @param translation             translation between estimated left and right cameras.
     * @param leftIntrinsic           intrinsic parameters to set on estimated left
     *                                camera.
     * @param rightIntrinsic          intrinsic parameters to set on estimated right
     *                                camera.
     * @param leftPoints              points on left view matched with points on right view
     *                                so they can be triangulated using estimated cameras. Both lists of points
     *                                must have the same size.
     * @param rightPoints             points on right view matched with points on left view
     *                                so they can be triangulated using estimated cameras. Both lists of points
     *                                must have the same size.
     * @param estimatedLeftCamera     instance where estimated left camera will be
     *                                stored.
     * @param estimatedRightCamera    instance where estimated right camera will be
     *                                stored.
     * @param triangulatedPoints      instance where triangulated 3D points will be
     *                                stored or null if triangulated points don't need to be kept.
     * @param validTriangulatedPoints instance which indicates which
     *                                triangulated 3D points are considered valid because they lie in front of
     *                                both cameras or null if such data doesn't need to be kept.
     * @return number of valid triangulated points which lie in front of the two
     * estimated cameras.
     * @throws WrongSizeException            never occurs.
     * @throws CameraException               if any of the estimated cameras become
     *                                       numerically unstable.
     * @throws LockedException               never occurs.
     * @throws NotReadyException             never occurs.
     * @throws Point3DTriangulationException if points cannot be triangulated
     *                                       because of numerical unstabilities.
     */
    private static int computeCamerasAndTriangulation(
            final Rotation3D rotation,
            final Point2D translation,
            final PinholeCameraIntrinsicParameters leftIntrinsic,
            final PinholeCameraIntrinsicParameters rightIntrinsic,
            final List<Point2D> leftPoints,
            final List<Point2D> rightPoints,
            final PinholeCamera estimatedLeftCamera,
            final PinholeCamera estimatedRightCamera,
            final List<Point3D> triangulatedPoints,
            final BitSet validTriangulatedPoints) throws WrongSizeException,
            CameraException, LockedException, NotReadyException,
            Point3DTriangulationException {

        if (triangulatedPoints != null) {
            triangulatedPoints.clear();
        }
        if (validTriangulatedPoints != null) {
            validTriangulatedPoints.clear();
        }
        int numValidTriangulatedPoints = 0;

        final Matrix leftIntrinsicMatrix = leftIntrinsic.getInternalMatrix();
        final Matrix rightIntrinsicMatrix = rightIntrinsic.getInternalMatrix();

        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();

        // 1st camera

        // set camera as a canonical matrix
        final Matrix tmp = Matrix.identity(PinholeCamera.PINHOLE_CAMERA_MATRIX_ROWS,
                PinholeCamera.PINHOLE_CAMERA_MATRIX_COLS);

        // add intrinsic parameters
        leftIntrinsicMatrix.multiply(tmp);

        // set internal matrix, normalize and fix camera sign
        estimatedLeftCamera.setInternalMatrix(leftIntrinsicMatrix);
        estimatedLeftCamera.normalize();
        estimatedLeftCamera.fixCameraSign();

        // 2nd camera

        // set left 3x3 minor containing rotation
        tmp.setSubmatrix(0, 0, 2, 2, rotationMatrix);

        // set last column containing translation
        translation.normalize();
        tmp.setElementAt(0, 3, translation.getHomX());
        tmp.setElementAt(1, 3, translation.getHomY());
        tmp.setElementAt(2, 3, translation.getHomW());

        // add intrinsic parameters
        rightIntrinsicMatrix.multiply(tmp);

        // set internal matrix, normalize and fix camera sign
        estimatedRightCamera.setInternalMatrix(rightIntrinsicMatrix);
        estimatedRightCamera.normalize();
        estimatedRightCamera.fixCameraSign();

        // set cameras on triangulator
        final SinglePoint3DTriangulator triangulator = SinglePoint3DTriangulator.
                create();

        final int numPoints = leftPoints.size();
        Point2D leftPoint;
        Point2D rightPoint;
        final List<Point2D> points = new ArrayList<>();
        final List<PinholeCamera> cameras = new ArrayList<>();
        Point3D triangulatedPoint;
        boolean frontLeft;
        boolean frontRight;
        for (int i = 0; i < numPoints; i++) {
            leftPoint = leftPoints.get(i);
            rightPoint = rightPoints.get(i);

            points.clear();
            points.add(leftPoint);
            points.add(rightPoint);

            cameras.clear();
            cameras.add(estimatedLeftCamera);
            cameras.add(estimatedRightCamera);

            triangulator.setPointsAndCameras(points, cameras);
            triangulatedPoint = triangulator.triangulate();
            if (triangulatedPoints != null) {
                triangulatedPoints.add(triangulatedPoint);
            }

            // check that triangulated point is in front of both cameras
            frontLeft = estimatedLeftCamera.isPointInFrontOfCamera(
                    triangulatedPoint);
            frontRight = estimatedRightCamera.isPointInFrontOfCamera(
                    triangulatedPoint);

            if (frontLeft && frontRight) {
                // point is valid because it is in front of both cameras
                if (validTriangulatedPoints != null) {
                    validTriangulatedPoints.set(i);
                }
                numValidTriangulatedPoints++;
            } else {
                // point is not valid
                if (validTriangulatedPoints != null) {
                    validTriangulatedPoints.clear(i);
                }
            }
        }

        return numValidTriangulatedPoints;
    }
}
