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
package com.irurueta.ar.calibration.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.ar.calibration.ImageOfAbsoluteConic;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best
 * ImageOfAbsoluteConic (IAC) for provided collection of 2D homographies.
 * Implementions of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class ImageOfAbsoluteConicRobustEstimator {
    /**
     * Default robust estimator method when none is provided.
     * In general for IAC estimation is best to use PROSAC or RANSAC than
     * any other method, as it provides more robust results.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD = RobustEstimatorMethod.PROSAC;

    /**
     * Default amount of progress variation before notifying a change in
     * estimation progress. By default, this is set to 5%.
     */
    public static final float DEFAULT_PROGRESS_DELTA = 0.05f;

    /**
     * Minimum allowed value for progress delta.
     */
    public static final float MIN_PROGRESS_DELTA = 0.0f;

    /**
     * Maximum allowed value for progress delta.
     */
    public static final float MAX_PROGRESS_DELTA = 1.0f;

    /**
     * Constant defining default confidence of the estimated result, which is
     * 99%. This means that with a probability of 99% estimation will be
     * accurate because chosen sub-samples will be inliers.
     */
    public static final double DEFAULT_CONFIDENCE = 0.99;

    /**
     * Default maximum allowed number of iterations.
     */
    public static final int DEFAULT_MAX_ITERATIONS = 5000;

    /**
     * Minimum allowed confidence value.
     */
    public static final double MIN_CONFIDENCE = 0.0;

    /**
     * Maximum allowed confidence value.
     */
    public static final double MAX_CONFIDENCE = 1.0;

    /**
     * Minimum allowed number of iterations.
     */
    public static final int MIN_ITERATIONS = 1;

    /**
     * Homographies to estimate image of absolute conic (IAC).
     */
    protected List<Transformation2D> homographies;

    /**
     * Internal non-robust estimator of IAC.
     */
    protected final LMSEImageOfAbsoluteConicEstimator iacEstimator;

    /**
     * Listener to be notified of events such as when estimation starts, ends or
     * its progress significantly changes.
     */
    protected ImageOfAbsoluteConicRobustEstimatorListener listener;

    /**
     * Indicates if this estimator is locked because an estimation is being
     * computed.
     */
    protected boolean locked;

    /**
     * Amount of progress variation before notifying a progress change during
     * estimation.
     */
    protected float progressDelta;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability
     * that the estimated result is correct. Usually this value will be close
     * to 1.0, but not exactly 1.0.
     */
    protected double confidence;

    /**
     * Maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an
     * approximate result will be available for retrieval.
     */
    protected int maxIterations;

    // internal members to compute residuals

    /**
     * IAC matrix for one iteration of the robust estimator.
     * This is used during residuals estimation.
     * This instance is reused for performance reasons.
     */
    private Matrix iacMatrix;

    /**
     * Matrix representation of an homography.
     * This is used during residuals estimation.
     * This instance is reused for performance reasons.
     */
    private Matrix homMatrix;

    /**
     * Sub-matrix of homography used as the left term on a matrix multiplication.
     * This is used during residuals estimation.
     * This instance is reused for performance reasons.
     */
    private double[] subMatrixLeft;

    /**
     * Sub-matrix of homography used as the right term on a matrix
     * multiplication.
     * This is used during residuals estimation.
     * This instance is reused for performance reasons.
     */
    private Matrix subMatrixRight;

    /**
     * Product multiplication of IAC by the right term.
     * This is used during residuals estimation.
     * This instance is reused for performance reasons.
     */
    private Matrix mult1;

    /**
     * Constructor.
     */
    protected ImageOfAbsoluteConicRobustEstimator() {
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
        iacEstimator = new LMSEImageOfAbsoluteConicEstimator();
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when
     *                 estimation starts, ends or its progress significantly changes.
     */
    protected ImageOfAbsoluteConicRobustEstimator(final ImageOfAbsoluteConicRobustEstimatorListener listener) {
        this();
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param homographies list of homographies (2D transformations) used to
     *                     estimate the image of absolute conic (IAC), which can be used to obtain
     *                     pinhole camera intrinsic parameters.
     * @throws IllegalArgumentException if not enough homographies are provided
     *                                  for default settings. Hence, at least 1 homography must be provided.
     */
    protected ImageOfAbsoluteConicRobustEstimator(final List<Transformation2D> homographies) {
        this();
        internalSetHomographies(homographies);
    }

    /**
     * Constructor.
     *
     * @param homographies list of homographies (2D transformations) used to
     *                     estimate the image of absolute conic (IAC), which can be used to obtain
     *                     pinhole camera intrinsic parameters.
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if not enough homographies are provided
     *                                  for default settings. Hence, at least 1 homography must be provided.
     */
    protected ImageOfAbsoluteConicRobustEstimator(
            final List<Transformation2D> homographies, final ImageOfAbsoluteConicRobustEstimatorListener listener) {
        this(listener);
        internalSetHomographies(homographies);
    }

    /**
     * Returns boolean indicating whether camera skewness is assumed to be zero
     * or not.
     * Skewness determines whether LCD sensor cells are properly aligned or not,
     * where zero indicates perfect alignment.
     * Typically, skewness is a value equal or very close to zero.
     *
     * @return true if camera skewness is assumed to be zero, otherwise camera
     * skewness is estimated.
     */
    public boolean isZeroSkewness() {
        return iacEstimator.isZeroSkewness();
    }

    /**
     * Sets boolean indicating whether camera skewness is assumed to be zero or
     * not.
     * Skewness determines whether LCD sensor cells are properly aligned or not,
     * where zero indicates perfect alignment.
     * Typically, skewness is a value equal or very close to zero.
     *
     * @param zeroSkewness true if camera skewness is assumed to be zero,
     *                     otherwise camera skewness is estimated.
     * @throws LockedException if estimator is locked.
     */
    public void setZeroSkewness(final boolean zeroSkewness) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        iacEstimator.setZeroSkewness(zeroSkewness);
    }

    /**
     * Returns boolean indicating whether principal point is assumed to be at
     * origin of coordinates or not.
     * Typically principal point is located at image center (origin of
     * coordinates), and usually matches the center of radial distortion if
     * it is taken into account.
     *
     * @return true if principal point is assumed to be at origin of
     * coordinates, false if principal point must be estimated.
     */
    public boolean isPrincipalPointAtOrigin() {
        return iacEstimator.isPrincipalPointAtOrigin();
    }

    /**
     * Sets boolean indicating whether principal point is assumed to be at
     * origin of coordinates or not.
     * Typically principal point is located at image center (origin of
     * coordinates), and usually matches the center of radial distortion if it
     * is taken into account.
     *
     * @param principalPointAtOrigin true if principal point is assumed to bet
     *                               at origin of coordinates, false if principal point must be estimated
     * @throws LockedException if estimator is locked.
     */
    public void setPrincipalPointAtOrigin(final boolean principalPointAtOrigin) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        iacEstimator.setPrincipalPointAtOrigin(principalPointAtOrigin);
    }

    /**
     * Returns boolean indicating whether aspect ratio of focal distances (i.e.
     * vertical focal distance divided by horizontal focal distance) is known or
     * not.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio. Typically, LCD sensor cells are square and hence aspect
     * ratio of focal distances is known and equal to 1.
     * This value is only taken into account if skewness is assumed to be zero,
     * otherwise it is ignored.
     *
     * @return true if focal distance aspect ratio is known, false otherwise.
     */
    public boolean isFocalDistanceAspectRatioKnown() {
        return iacEstimator.isFocalDistanceAspectRatioKnown();
    }

    /**
     * Sets boolean indicating whether aspect ratio of focal distances (i.e.
     * vertical focal distance divided by horizontal focal distance) is known or
     * not.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio. Typically, LCD sensor cells are square and hence aspect
     * ratio of focal distances is known and equal to 1.
     * This value is only taken into account if skewness is assumed to be zero,
     * otherwise it is ignored.
     *
     * @param focalDistanceAspectRatioKnown true if focal distance aspect ratio
     *                                      is known, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setFocalDistanceAspectRatioKnown(final boolean focalDistanceAspectRatioKnown) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        iacEstimator.setFocalDistanceAspectRatioKnown(focalDistanceAspectRatioKnown);
    }

    /**
     * Returns aspect ratio of focal distances (i.e. vertical focal distance
     * divided by horizontal focal distance).
     * This value is only taken into account if skewness is assumed to be zero
     * and focal distance aspect ratio is marked as known, otherwise it is
     * ignored.
     * By default, this is 1.0, since it is taken into account that typically
     * LCD sensor cells are square and hence aspect ratio focal distances is
     * known and equal to 1.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio.
     * Notice that a negative aspect ratio indicates that vertical axis is
     * reversed. This can be useful in some situations where image vertical
     * coordinates are reversed respect to the physical world (i.e. in computer
     * graphics typically image vertical coordinates go downwards, while in
     * physical world they go upwards).
     *
     * @return aspect ratio of focal distances.
     */
    public double getFocalDistanceAspectRatio() {
        return iacEstimator.getFocalDistanceAspectRatio();
    }

    /**
     * Sets aspect ratio of focal distances (i.e. vertical focal distance
     * divided by horizontal focal distance).
     * This value is only taken into account if skewness is assumed to be zero
     * and focal distance aspect ratio is marked as known, otherwise it is
     * ignored.
     * By default, this is 1.0, since it is taken into account that typically
     * LCD sensor cells are square and hence aspect ratio focal distances is
     * known and equal to 1.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio.
     * Notice that a negative aspect ratio indicates that vertical axis is
     * reversed. This can be useful in some situations where image vertical
     * coordinates are reversed respect to the physical world (i.e. in computer
     * graphics typically image vertical coordinates go downwards, while in
     * physical world they go upwards).
     *
     * @param focalDistanceAspectRatio aspect ratio of focal distances to be set
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if focal distance aspect ratio is too
     *                                  close to zero, as it might produce numerical instabilities.
     */
    public void setFocalDistanceAspectRatio(final double focalDistanceAspectRatio) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        iacEstimator.setFocalDistanceAspectRatio(focalDistanceAspectRatio);
    }

    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     *
     * @return listener to be notified of events.
     */
    public ImageOfAbsoluteConicRobustEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(final ImageOfAbsoluteConicRobustEstimatorListener listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.listener = listener;
    }

    /**
     * Indicates whether listener has been provided and is available for
     * retrieval.
     *
     * @return true if available, false otherwise.
     */
    public boolean isListenerAvailable() {
        return listener != null;
    }

    /**
     * Indicates if this instance is locked because estimation is being
     * computed.
     *
     * @return true if locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Returns amount of progress variation before notifying a progress change
     * during estimation.
     *
     * @return amount of progress variation before notifying a progress change
     * during estimation.
     */
    public float getProgressDelta() {
        return progressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change
     * during estimation.
     *
     * @param progressDelta amount of progress variation before notifying a
     *                      progress change during estimation.
     * @throws IllegalArgumentException if progress delta is less than zero or
     *                                  greater than 1.
     * @throws LockedException          if this estimator is locked because an estimation
     *                                  is being computed.
     */
    public void setProgressDelta(final float progressDelta) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA || progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        this.progressDelta = progressDelta;
    }

    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%). The amount of confidence indicates the
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0.
     *
     * @return amount of confidence as a value between 0.0 and 1.0.
     */
    public double getConfidence() {
        return confidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which
     * is equivalent to 100%). The amount of confidence indicates the
     * probability that the estimated result is correct. Usually this value will
     * be close to 1.0, but not exactly 1.0.
     *
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and
     *                                  1.0.
     * @throws LockedException          if this estimator is locked because an estimator
     *                                  is being computed.
     */
    public void setConfidence(final double confidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (confidence < MIN_CONFIDENCE || confidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        this.confidence = confidence;
    }

    /**
     * Returns maximum allowed number of iterations. If maximum allowed number
     * of iterations is achieved without converging to a result when calling
     * estimate(), a RobustEstimatorException will be raised.
     *
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return maxIterations;
    }

    /**
     * Sets maximum allowed number of iterations. When the maximum number of
     * iterations is exceeded, result will not be available, however an
     * approximate result will be available for retrieval.
     *
     * @param maxIterations maximum allowed number of iterations to be set.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if this estimator is locked because an estimation
     *                                  is being computed.
     */
    public void setMaxIterations(final int maxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        this.maxIterations = maxIterations;
    }

    /**
     * Gets list of homographies to estimate IAC.
     *
     * @return list of homographies to estimate IAC.
     */
    public List<Transformation2D> getHomographies() {
        return homographies;
    }

    /**
     * Sets list of homographies to estimate IAC.
     *
     * @param homographies list of homographies to estimate IAC.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if provided list of homographies does
     *                                  not contain enough elements to estimate the DIAC using current
     *                                  settings.
     */
    public void setHomographies(final List<Transformation2D> homographies) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetHomographies(homographies);
    }

    /**
     * Returns minimum number of required homographies needed to estimate the
     * Imag eof Absolute Conic (IAC).
     * If no constraints are imposed, then at least 3 homographies are required.
     * For each constraint imposed, one less equation will be required, hence
     * if skewness is assumed to be known (by using its typical value of zero),
     * then only 4 equations will be needed.
     * If also the horizontal and vertical coordinates of the principal point
     * are assumed to be known (by being placed at the 2D origin of coordinates,
     * which is a typical value), then only 2 equations will be needed
     * (1 homography).
     *
     * @return minimum number of required homographies required to estimate the
     * IAC.
     */
    public int getMinNumberOfRequiredHomographies() {
        return iacEstimator.getMinNumberOfRequiredHomographies();
    }

    /**
     * Returns value indicating whether required data has been provided so that
     * IAC estimation can start.
     * If true, estimator is ready to compute the IAC, otherwise more data needs
     * to be provided.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return homographies != null && homographies.size() >= getMinNumberOfRequiredHomographies();
    }

    /**
     * Returns quality scores corresponding to each homography.
     * The larger the score value the better the quality of the homography.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @return quality scores corresponding to each homography.
     */
    public double[] getQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each homography.
     * The larger the score value the better the quality of the homography.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each homography.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than minimum required number of homographies.
     */
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Estimates Image of Absolute Conic (IAC).
     *
     * @return estimated IAC.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws NotReadyException        if provided input data is not enough to start
     *                                  the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    public abstract ImageOfAbsoluteConic estimate() throws LockedException, NotReadyException,
            RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Creates an image of absolute conic robust estimator using provided
     * method.
     *
     * @param method method of a robust estimator algorithm to estimate best
     *               IAC.
     * @return an instance of an image of absolute conic robust estimator.
     */
    public static ImageOfAbsoluteConicRobustEstimator create(final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSImageOfAbsoluteConicRobustEstimator();
            case MSAC -> new MSACImageOfAbsoluteConicRobustEstimator();
            case PROSAC -> new PROSACImageOfAbsoluteConicRobustEstimator();
            case PROMEDS -> new PROMedSImageOfAbsoluteConicRobustEstimator();
            default -> new RANSACImageOfAbsoluteConicRobustEstimator();
        };
    }

    /**
     * Creates an image of absolute conic robust estimator using provided
     * homographies.
     *
     * @param homographies  list of homographies.
     * @param qualityScores quality scores corresponding to each homography.
     * @param method        method of a robust estimator algorithm to estimate
     *                      best IAC.
     * @return an instance of an image of absolute conic robust estimator.
     * @throws IllegalArgumentException if provided list of homographies and
     *                                  quality scores don't have the same size or size is too short.
     */
    public static ImageOfAbsoluteConicRobustEstimator create(
            final List<Transformation2D> homographies, final double[] qualityScores,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSImageOfAbsoluteConicRobustEstimator(homographies);
            case MSAC -> new MSACImageOfAbsoluteConicRobustEstimator(homographies);
            case PROSAC -> new PROSACImageOfAbsoluteConicRobustEstimator(homographies, qualityScores);
            case PROMEDS -> new PROMedSImageOfAbsoluteConicRobustEstimator(homographies, qualityScores);
            default -> new RANSACImageOfAbsoluteConicRobustEstimator(homographies);
        };
    }

    /**
     * Creates an image of absolute conic robust estimator using provided
     * homographies.
     *
     * @param homographies list of homographies.
     * @param method       method of a robust estimator algorithm to estimate
     *                     best IAC.
     * @return an instance of an image of absolute conic robust estimator.
     * @throws IllegalArgumentException if provided list of homographies is too
     *                                  short.
     */
    public static ImageOfAbsoluteConicRobustEstimator create(
            final List<Transformation2D> homographies, final RobustEstimatorMethod method) {
        return switch (method) {
            case LMEDS -> new LMedSImageOfAbsoluteConicRobustEstimator(homographies);
            case MSAC -> new MSACImageOfAbsoluteConicRobustEstimator(homographies);
            case PROSAC -> new PROSACImageOfAbsoluteConicRobustEstimator(homographies);
            case PROMEDS -> new PROMedSImageOfAbsoluteConicRobustEstimator(homographies);
            default -> new RANSACImageOfAbsoluteConicRobustEstimator(homographies);
        };
    }

    /**
     * Creates an image of absolute conic robust estimator using default
     * method.
     *
     * @return an instance of an image of absolute conic robust estimator.
     */
    public static ImageOfAbsoluteConicRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an image of absolute conic robust estimator using provided
     * homographies.
     *
     * @param homographies  list of homographies.
     * @param qualityScores quality scores corresponding to each point.
     * @return an instance of an image of absolute conic robust estimator.
     * @throws IllegalArgumentException if provided list of homographies and
     *                                  quality scores don't have the same size or size is too short.
     */
    public static ImageOfAbsoluteConicRobustEstimator create(
            final List<Transformation2D> homographies, final double[] qualityScores) {
        return create(homographies, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates an image of absolute conic robust estimator using provided
     * homographies.
     *
     * @param homographies list of homographies.
     * @return an instance of an image of absolute conic robust estimator.
     * @throws IllegalArgumentException if provided list of homographies and
     *                                  is too short.
     */
    public static ImageOfAbsoluteConicRobustEstimator create(final List<Transformation2D> homographies) {
        return create(homographies, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Computes the residual between an image of absolute conic (IAC) and
     * a 2D transformation (homography).
     * Residual is derived from the fact that rotation matrices are
     * orthonormal (i.e. h1'*IAC*h2 = 0 and
     * h1'*IAC*h1 = h2'*IAC*h2 --&lt; h1'*IAC*h1 - h2'*IAC*h2 = 0).
     * The residual will be the average error of those two equations.
     *
     * @param iac        the matrix of an image of absolute conic (IAC)
     * @param homography 2D transformation (homography)
     * @return residual.
     */
    protected double residual(final ImageOfAbsoluteConic iac, final Transformation2D homography) {

        iac.normalize();
        if (homography instanceof ProjectiveTransformation2D projectiveTransformation2D) {
            projectiveTransformation2D.normalize();
        }

        try {
            if (iacMatrix == null) {
                iacMatrix = iac.asMatrix();
            } else {
                iac.asMatrix(iacMatrix);
            }

            if (homMatrix == null) {
                homMatrix = homography.asMatrix();
            } else {
                homography.asMatrix(homMatrix);
            }

            if (subMatrixLeft == null) {
                subMatrixLeft = new double[ProjectiveTransformation2D.HOM_COORDS];
            }
            if (subMatrixRight == null) {
                subMatrixRight = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 1);
            }
            if (mult1 == null) {
                mult1 = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 1);
            }

            // 1st equation h1'*IAC*h2 = 0

            // 1st column of homography
            homMatrix.getSubmatrixAsArray(0, 0,
                    ProjectiveTransformation2D.HOM_COORDS - 1, 0, subMatrixLeft);
            // 2nd column of homography
            homMatrix.getSubmatrix(0, 1,
                    ProjectiveTransformation2D.HOM_COORDS - 1, 1, subMatrixRight);

            // IAC * h2
            iacMatrix.multiply(subMatrixRight, mult1);

            // h1' * (IAC * h2)
            final var error1 = Math.abs(ArrayUtils.dotProduct(subMatrixLeft, mult1.getBuffer()));

            // 2nd equation h1'*IAC*h1 - h2'*IAC*h2 = 0

            // 1st column of homography
            homMatrix.getSubmatrixAsArray(0, 0,
                    ProjectiveTransformation2D.HOM_COORDS - 1, 0, subMatrixLeft);
            homMatrix.getSubmatrix(0, 0,
                    ProjectiveTransformation2D.HOM_COORDS - 1, 0, subMatrixRight);

            // IAC * h1
            iacMatrix.multiply(subMatrixRight, mult1);

            // h1' * (IAC * h1)
            final var error2a = ArrayUtils.dotProduct(subMatrixLeft, mult1.getBuffer());

            // 2nd column of homography
            homMatrix.getSubmatrixAsArray(0, 1,
                    ProjectiveTransformation2D.HOM_COORDS - 1, 1, subMatrixLeft);
            homMatrix.getSubmatrix(0, 1,
                    ProjectiveTransformation2D.HOM_COORDS - 1, 1, subMatrixRight);

            // IAC * h2
            iacMatrix.multiply(subMatrixRight, mult1);

            // h2' * (IAC * h1)
            final var error2b = ArrayUtils.dotProduct(subMatrixLeft, mult1.getBuffer());

            final var error2 = Math.abs(error2a - error2b);

            return 0.5 * (error1 + error2);
        } catch (final AlgebraException e) {
            return Double.MAX_VALUE;
        }
    }

    /**
     * Sets list of homographies.
     * This method does not check whether estimator is locked.
     *
     * @param homographies list of homographies to estimate IAC.
     * @throws IllegalArgumentException if provided list of homographies is null
     *                                  or too small.
     */
    private void internalSetHomographies(final List<Transformation2D> homographies) {
        if (homographies == null || homographies.size() < getMinNumberOfRequiredHomographies()) {
            throw new IllegalArgumentException();
        }

        this.homographies = homographies;
    }
}
