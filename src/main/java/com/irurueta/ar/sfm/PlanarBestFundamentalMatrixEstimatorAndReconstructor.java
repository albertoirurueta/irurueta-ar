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
import com.irurueta.ar.epipolar.Corrector;
import com.irurueta.ar.epipolar.CorrectorType;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.estimators.FundamentalMatrixEstimatorException;
import com.irurueta.ar.epipolar.estimators.PlanarFundamentalMatrixEstimator;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.PointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.geometry.estimators.RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.List;

/**
 * This class takes matched pairs of 2D points corresponding to a planar scene,
 * estimates an homography relating both sets of points, decomposes such
 * homography induced by the 3D plane on the scene, and uses such decomposition
 * to determine the best epipolar geometry (e.g. fundamental matrix) by using
 * the essential matrix and provided intrinsic camera parameters on both views
 * corresponding to both sets of points to reconstruct points and choose the
 * solution that produces the largest amount of points located in front of both
 * cameras.
 * This class requires 2 sets of matched 2D points and the intrinsic parameters
 * of the cameras in both views, hence cameras must be calibrated in some way
 * before using this class.
 * This class is similar to PlanarFundamentalMatrixEstimator but picks the best
 * solution by reconstructing the 3D points in the scene and choosing the
 * solution that produces the largest amount of points located in front of both
 * cameras.
 */
public class PlanarBestFundamentalMatrixEstimatorAndReconstructor {

    /**
     * Minimum number of matched points required to find a solution.
     */
    public static final int MINIMUM_SIZE = 4;

    /**
     * List of matched 2D points in the left view.
     */
    private List<Point2D> leftPoints;

    /**
     * List of matched 2D points in the right view.
     */
    private List<Point2D> rightPoints;

    /**
     * Intrinsic parameters for the camera on the left view.
     */
    private PinholeCameraIntrinsicParameters leftIntrinsics;

    /**
     * Intrinsic parameters for the camera on the right view.
     */
    private PinholeCameraIntrinsicParameters rightIntrinsics;

    /**
     * Listener to attend events generated by this instance.
     */
    private PlanarBestFundamentalMatrixEstimatorAndReconstructorListener listener;

    /**
     * Indicates whether this instance is locked while computing a solution.
     */
    private boolean locked;

    /**
     * Homography estimator relating both views.
     */
    private PointCorrespondenceProjectiveTransformation2DRobustEstimator homographyEstimator;

    /**
     * Type of corrector to use to triangulate matched points using the
     * corresponding essential matrix or null if no corrector needs to be used.
     */
    private CorrectorType essentialCameraEstimatorCorrectorType = Corrector.DEFAULT_TYPE;

    /**
     * Estimated homography.
     */
    private ProjectiveTransformation2D homography;

    /**
     * Best estimated fundamental matrix.
     */
    private FundamentalMatrix fundamentalMatrix;

    /**
     * Best estimated triangulated points.
     */
    private List<Point3D> triangulatedPoints;

    /**
     * Contains booleans indicating which of the best triangulated points are
     * valid (i.e. lie in front of both estimated cameras) or not.
     */
    private BitSet validTriangulatedPoints;

    /**
     * Best estimated camera for left view.
     */
    private PinholeCamera estimatedLeftCamera;

    /**
     * Best estimated camera for right view.
     */
    private PinholeCamera estimatedRightCamera;

    /**
     * Constructor.
     */
    public PlanarBestFundamentalMatrixEstimatorAndReconstructor() {
        homographyEstimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create();
    }

    /**
     * Constructor.
     *
     * @param leftPoints      list of matched 2D points in the left view.
     * @param rightPoints     list of matched 2D points in the right view.
     * @param leftIntrinsics  intrinsic parameters for the camera on the left view.
     * @param rightIntrinsics intrinsic parameters for the camera on the right view.
     * @throws IllegalArgumentException if provided list of matched points do
     *                                  not contain enough points or if they have different sizes.
     */
    public PlanarBestFundamentalMatrixEstimatorAndReconstructor(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final PinholeCameraIntrinsicParameters leftIntrinsics,
            final PinholeCameraIntrinsicParameters rightIntrinsics) {
        this();
        internalSetLeftAndRightPoints(leftPoints, rightPoints);
        this.leftIntrinsics = leftIntrinsics;
        this.rightIntrinsics = rightIntrinsics;
    }

    /**
     * Constructor.
     *
     * @param leftPoints      list of matched 2D points in the left view.
     * @param rightPoints     list of matched 2D points in the right view.
     * @param leftIntrinsics  intrinsic parameters for the camera on the left view.
     * @param rightIntrinsics intrinsic parameters for the camera on the right view.
     * @param listener        listener to be notified of events generated by this instance.
     * @throws IllegalArgumentException if provided list of matched points do
     *                                  not contain enough points or if they have different sizes.
     */
    public PlanarBestFundamentalMatrixEstimatorAndReconstructor(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints,
            final PinholeCameraIntrinsicParameters leftIntrinsics,
            final PinholeCameraIntrinsicParameters rightIntrinsics,
            final PlanarBestFundamentalMatrixEstimatorAndReconstructorListener listener) {
        this(leftPoints, rightPoints, leftIntrinsics, rightIntrinsics);
        this.listener = listener;
    }

    /**
     * Gets list of matched 2D points in the left view.
     *
     * @return list of matched 2D points in the left view.
     */
    public List<Point2D> getLeftPoints() {
        return leftPoints;
    }

    /**
     * Sets list of matched 2D points in the left view.
     *
     * @param leftPoints list of matched 2D points in the left view.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided points do not have enough
     *                                  points.
     */
    public void setLeftPoints(final List<Point2D> leftPoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (leftPoints.size() < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }

        this.leftPoints = leftPoints;
    }

    /**
     * Gets list of matched 2D points in the right view.
     *
     * @return list of matched 2D points in the right view.
     */
    public List<Point2D> getRightPoints() {
        return rightPoints;
    }

    /**
     * Sets list of matched 2D points in the right view.
     *
     * @param rightPoints list of matched 2D points in the right view.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided points do not have enough
     *                                  points.
     */
    public void setRightPoints(final List<Point2D> rightPoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (rightPoints.size() < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }

        this.rightPoints = rightPoints;
    }

    /**
     * Sets lists of matched 2D points in the left and right views.
     *
     * @param leftPoints  list of matched 2D points in the left view.
     * @param rightPoints list of matched 2D points in the right view.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided points do not have enough
     *                                  points or lists have different sizes.
     */
    public void setLeftAndRightPoints(final List<Point2D> leftPoints, final List<Point2D> rightPoints)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetLeftAndRightPoints(leftPoints, rightPoints);
    }

    /**
     * Gets intrinsic parameters for the camera on the left view.
     *
     * @return intrinsic parameters for the camera on the left view.
     */
    public PinholeCameraIntrinsicParameters getLeftIntrinsics() {
        return leftIntrinsics;
    }

    /**
     * Sets intrinsic parameters for the camera on the left view.
     *
     * @param leftIntrinsics intrinsic parameters for the camera on the left
     *                       view.
     * @throws LockedException if estimator is locked.
     */
    public void setLeftIntrinsics(final PinholeCameraIntrinsicParameters leftIntrinsics) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.leftIntrinsics = leftIntrinsics;
    }

    /**
     * Gets intrinsic parameters for the camera on the right view.
     *
     * @return intrinsic parameters for the camera on the right view.
     */
    public PinholeCameraIntrinsicParameters getRightIntrinsics() {
        return rightIntrinsics;
    }

    /**
     * Sets intrinsic parameters for the camera on the right view.
     *
     * @param rightIntrinsics intrinsic parameters for the camera on the right
     *                        view.
     * @throws LockedException if estimator is locked.
     */
    public void setRightIntrinsics(final PinholeCameraIntrinsicParameters rightIntrinsics) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.rightIntrinsics = rightIntrinsics;
    }

    /**
     * Gets internal homography estimator.
     *
     * @return internal homography estimator.
     */
    public PointCorrespondenceProjectiveTransformation2DRobustEstimator getHomographyEstimator() {
        return homographyEstimator;
    }

    /**
     * Sets internal homography estimator.
     *
     * @param homographyEstimator internal homography estimator.
     * @throws LockedException      if estimator is locked.
     * @throws NullPointerException if provided estimator is null.
     */
    public void setHomographyEstimator(
            final PointCorrespondenceProjectiveTransformation2DRobustEstimator homographyEstimator)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (homographyEstimator == null) {
            throw new NullPointerException();
        }

        this.homographyEstimator = homographyEstimator;
    }

    /**
     * Gets type of corrector to use to triangulate matched points using the
     * corresponding essential matrix or null if no corrector needs to be used.
     *
     * @return corrector to use for triangulation or null.
     */
    public CorrectorType getEssentialCameraEstimatorCorrectorType() {
        return essentialCameraEstimatorCorrectorType;
    }

    /**
     * Sets type of corrector to use to triangulate matched points using the
     * corresponding essential matrix or null if no corrector needs to be used.
     *
     * @param correctorType corrector to use for triangulation or null.
     * @throws LockedException if estimator is locked.
     */
    public void setEssentialCameraEstimatorCorrectorType(final CorrectorType correctorType) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        essentialCameraEstimatorCorrectorType = correctorType;
    }

    /**
     * Gets amount of confidence on homography estimation expressed as a value
     * between 0.0 and 1.0 (which is equivalent to 100%). The amount of
     * confidence indicates the probability that the estimated result is
     * correct. Usually this value will be close to 1.0, but not exactly 1.0.
     *
     * @return amount of confidence as a value between 0.0 and 1.0.
     */
    public double getHomographyConfidence() {
        return homographyEstimator.getConfidence();
    }

    /**
     * Sets amount of confidence on homography estimation expressed as a value
     * between 0.0 and 1.0 (which is equivalent to 100%). The amount of
     * confidence indicates the probability that the estimated result is
     * correct. Usually this value will be close to 1.0, but not exactly 1.0.
     *
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and
     *                                  1.0.
     * @throws LockedException          if estimator is locked.
     */
    public void setHomographyConfidence(final double confidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        homographyEstimator.setConfidence(confidence);
    }

    /**
     * Returns maximum allowed number of iterations for homography estimation.
     *
     * @return maximum allowed number of iterations.
     */
    public int getHomographyMaxIterations() {
        return homographyEstimator.getMaxIterations();
    }

    /**
     * Sets maximum allowed number of iterations for homography estimation.
     *
     * @param maxIterations maximum allowed number of iterations.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if estimator is locked.
     */
    public void setHomographyMaxIterations(final int maxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        homographyEstimator.setMaxIterations(maxIterations);
    }

    /**
     * Indicates whether result of homography estimation is refined.
     *
     * @return true to refine homography result, false to simply use result
     * found by robust estimation without further refining.
     */
    public boolean isHomographyRefined() {
        return homographyEstimator.isResultRefined();
    }

    /**
     * Specifies whether homography estimation must be refined or not.
     *
     * @param refineResult true to refine homography result, false to simply use
     *                     result found by robust estimator without further refining.
     * @throws LockedException if estimator is locked.
     */
    public void setHomographyRefined(final boolean refineResult) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        homographyEstimator.setResultRefined(refineResult);
    }

    /**
     * Indicates whether homography covariance must be kept after estimation.
     * This setting is only taken into account if homography is refined.
     *
     * @return true if homography covariance must be kept after estimation,
     * false otherwise.
     */
    public boolean isHomographyCovarianceKept() {
        return homographyEstimator.isCovarianceKept();
    }

    /**
     * Specifies whether homography covariance must be kept after estimation.
     * This setting is only taken into account if homography is refined.
     *
     * @param keepCovariance true if homography covariance must be kept after
     *                       estimation, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setHomographyCovarianceKept(final boolean keepCovariance) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        homographyEstimator.setCovarianceKept(keepCovariance);
    }

    /**
     * Gets covariance for estimated homography if available.
     * This is only available when homography has been refined and covariance is
     * kept.
     *
     * @return estimated homography covariance or null.
     */
    public Matrix getHomographyCovariance() {
        return homographyEstimator.getCovariance();
    }

    /**
     * Returns method being used for homography robust estimation.
     *
     * @return method being used for homography robust estimation.
     */
    public RobustEstimatorMethod getHomographyMethod() {
        return homographyEstimator.getMethod();
    }

    /**
     * Returns quality scores corresponding to each pair of matched points.
     * This is used for homography estimation.
     * The larger the score value the better the quality of the matching.
     *
     * @return quality scores for each pair of matched points.
     */
    public double[] getQualityScores() {
        return homographyEstimator.getQualityScores();
    }

    /**
     * Sets quality scores corresponding to each pair of matched points.
     * This is used for homography estimation.
     * The larger the score value the better the quality of the matching.
     *
     * @param qualityScore quality scores corresponding to eac pair of matched
     *                     points.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than minimum required size (4 points).
     */
    public void setQualityScores(final double[] qualityScore) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        homographyEstimator.setQualityScores(qualityScore);
    }

    /**
     * Gets listener to attend events generated by this instance.
     *
     * @return listener to attend events generated by this instance.
     */
    public PlanarBestFundamentalMatrixEstimatorAndReconstructorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to attend events generated by this instance.
     *
     * @param listener listener to attend events generated by this instance.
     */
    public void setListener(final PlanarBestFundamentalMatrixEstimatorAndReconstructorListener listener) {
        this.listener = listener;
    }

    /**
     * Indicates whether this instance is locked while computing a solution.
     *
     * @return true if this instance is locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Indicates whether this instance is ready to start the estimation when all
     * required data has been provided.
     *
     * @return true if this instance is ready, false otherwise.
     */
    public boolean isReady() {
        return leftPoints != null && leftPoints.size() >= MINIMUM_SIZE && rightPoints != null
                && rightPoints.size() >= MINIMUM_SIZE && leftPoints.size() == rightPoints.size()
                && leftIntrinsics != null && rightIntrinsics != null && homographyEstimator.isReady();
    }

    /**
     * Gets estimated homography.
     *
     * @return estimated homography.
     */
    public ProjectiveTransformation2D getHomography() {
        return homography;
    }

    /**
     * Gets best estimated fundamental matrix.
     *
     * @return best estimated fundamental matrix.
     */
    public FundamentalMatrix getFundamentalMatrix() {
        return fundamentalMatrix;
    }

    /**
     * Gets best estimated triangulated points.
     *
     * @return best estimated triangulated points.
     */
    public List<Point3D> getTriangulatedPoints() {
        return triangulatedPoints;
    }

    /**
     * Gets set containing booleans indicating which of the best triangulated
     * points are valid (i.e. lie in front of both estimated cameras) or not.
     *
     * @return set indicating which of the best triangulated points are valid.
     */
    public BitSet getValidTriangulatedPoints() {
        return validTriangulatedPoints;
    }

    /**
     * Gets best estimated camera for left view.
     *
     * @return best estimated camera for left view.
     */
    public PinholeCamera getEstimatedLeftCamera() {
        return estimatedLeftCamera;
    }

    /**
     * Gets best estimated camera for right view.
     *
     * @return best estimated camera for right view.
     */
    public PinholeCamera getEstimatedRightCamera() {
        return estimatedRightCamera;
    }

    /**
     * Estimates homography, the best fundamental matrix, their cameras and
     * reconstructs matched points.
     *
     * @throws LockedException                     if estimator is locked.
     * @throws NotReadyException                   if estimator is not ready because required data
     *                                             is missing.
     * @throws FundamentalMatrixEstimatorException if something fails, typically
     *                                             due to numerical instabilities.
     */
    public void estimateAndReconstruct() throws LockedException, NotReadyException,
            FundamentalMatrixEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        // always enable inlier estimation for homography
        enableHomographyInliersEstimation();

        try {
            locked = true;

            if (listener != null) {
                listener.onEstimateStart(this);
            }

            // estimate homography
            homography = homographyEstimator.estimate();

            final var homographyInliers = homographyEstimator.getInliersData();

            // estimate all fundamental matrices for homography
            final var fundamentalMatrixEstimator = new PlanarFundamentalMatrixEstimator(homography, leftIntrinsics,
                    rightIntrinsics);

            final var fundamentalMatrices = fundamentalMatrixEstimator.estimate();

            if (fundamentalMatrices == null) {
                throw new FundamentalMatrixEstimatorException();
            }

            // select homography inlier points
            final var lPoints = new ArrayList<Point2D>();
            final var rPoints = new ArrayList<Point2D>();
            final var bitset = homographyInliers.getInliers();
            final var bitsetLength = bitset.length();
            for (var i = 0; i < bitsetLength; i++) {
                if (bitset.get(i)) {
                    // is inlier
                    lPoints.add(this.leftPoints.get(i));
                    rPoints.add(this.rightPoints.get(i));
                }
            }

            // pick best fundamental matrix
            final var essentialCamerasEstimator = new EssentialMatrixInitialCamerasEstimator(leftIntrinsics,
                    rightIntrinsics, lPoints, rPoints);
            essentialCamerasEstimator.setCorrectorType(essentialCameraEstimatorCorrectorType);
            essentialCamerasEstimator.setPointsTriangulated(true);
            essentialCamerasEstimator.setValidTriangulatedPointsMarked(true);

            var numBest = 0;
            for (final var fMatrix : fundamentalMatrices) {
                essentialCamerasEstimator.setFundamentalMatrix(fMatrix);

                essentialCamerasEstimator.estimate();

                final var validPoints = essentialCamerasEstimator.getValidTriangulatedPoints();
                final var num = validPoints.cardinality();
                if (num > numBest) {
                    numBest = num;
                    this.fundamentalMatrix = fMatrix;
                    triangulatedPoints = essentialCamerasEstimator.getTriangulatedPoints();
                    validTriangulatedPoints = validPoints;
                    estimatedLeftCamera = essentialCamerasEstimator.getEstimatedLeftCamera();
                    estimatedRightCamera = essentialCamerasEstimator.getEstimatedRightCamera();
                }
            }

            if (listener != null) {
                listener.onEstimateEnd(this);
            }

        } catch (final RobustEstimatorException | InitialCamerasEstimationFailedException e) {
            throw new FundamentalMatrixEstimatorException(e);
        } finally {
            locked = false;
        }
    }

    /**
     * Internal method that sets list of matched 2D points in the left and right
     * views.
     *
     * @param leftPoints  list of matched 2D points in the left view.
     * @param rightPoints list of matched 2D points in the right view.
     * @throws IllegalArgumentException if provided points do not have enough
     *                                  points or lists have different sizes.
     */
    private void internalSetLeftAndRightPoints(final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        if (leftPoints.size() < MINIMUM_SIZE || rightPoints.size() < MINIMUM_SIZE
                || leftPoints.size() != rightPoints.size()) {
            throw new IllegalArgumentException();
        }

        this.leftPoints = leftPoints;
        this.rightPoints = rightPoints;
        try {
            homographyEstimator.setPoints(leftPoints, rightPoints);

            if (homographyEstimator.getMethod() == RobustEstimatorMethod.PROMEDS) {
                final var promedsEstimator =
                        (PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator;
                if (promedsEstimator.getQualityScores() == null) {
                    final var qualityScores = new double[leftPoints.size()];
                    Arrays.fill(qualityScores, 1.0);
                    promedsEstimator.setQualityScores(qualityScores);
                }
            } else if (homographyEstimator.getMethod() == RobustEstimatorMethod.PROSAC) {
                final var prosacEstimator =
                        (PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator;
                if (prosacEstimator.getQualityScores() == null) {
                    final var qualityScores = new double[leftPoints.size()];
                    Arrays.fill(qualityScores, 1.0);
                    prosacEstimator.setQualityScores(qualityScores);
                }
            }
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Ensures that inlier estimation is enabled on homography estimator.
     */
    private void enableHomographyInliersEstimation() {
        try {
            if (homographyEstimator.getMethod() == RobustEstimatorMethod.RANSAC) {
                final var ransacHomographyEstimator =
                        (RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator;
                ransacHomographyEstimator.setComputeAndKeepInliersEnabled(true);
                ransacHomographyEstimator.setComputeAndKeepResidualsEnabled(true);
            } else if (homographyEstimator.getMethod() == RobustEstimatorMethod.PROSAC) {
                final var prosacHomographyEstimator =
                        (PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator;
                prosacHomographyEstimator.setComputeAndKeepInliersEnabled(true);
                prosacHomographyEstimator.setComputeAndKeepResidualsEnabled(true);
            }
        } catch (final LockedException ignore) {
            // never happens
        }
    }
}
