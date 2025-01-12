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
package com.irurueta.ar.calibration.estimators;

import com.irurueta.ar.calibration.DualAbsoluteQuadric;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best
 * DualAbsoluteQuadric (DAQ) for provided collection of cameras.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
@SuppressWarnings("DuplicatedCode")
public abstract class DualAbsoluteQuadricRobustEstimator {

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD = RobustEstimatorMethod.LMEDS;

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
     * Cameras to estimate dual absolute quadric (DAQ).
     */
    protected List<PinholeCamera> cameras;

    /**
     * Internal non-robust estimator of DAQ.
     */
    protected final LMSEDualAbsoluteQuadricEstimator daqEstimator;

    /**
     * Listener to be notified of events such as when estimation starts, ends or
     * its progress significantly changes.
     */
    protected DualAbsoluteQuadricRobustEstimatorListener listener;

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

    /**
     * Constructor.
     */
    protected DualAbsoluteQuadricRobustEstimator() {
        progressDelta = DEFAULT_PROGRESS_DELTA;
        confidence = DEFAULT_CONFIDENCE;
        maxIterations = DEFAULT_MAX_ITERATIONS;
        daqEstimator = new LMSEDualAbsoluteQuadricEstimator();
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when
     *                 estimation starts, ends or its progress significantly changes.
     */
    protected DualAbsoluteQuadricRobustEstimator(final DualAbsoluteQuadricRobustEstimatorListener listener) {
        this();
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param cameras list of cameras used to estimate the dual absolute
     *                quadric (DAQ), which can be used to obtain pinhole camera intrinsic
     *                parameters.
     * @throws IllegalArgumentException if not enough cameras are provided
     *                                  for default settings. Hence, at least 2 cameras must be provided.
     */
    protected DualAbsoluteQuadricRobustEstimator(final List<PinholeCamera> cameras) {
        this();
        internalSetCameras(cameras);
    }

    /**
     * Constructor.
     *
     * @param cameras  list of cameras used to estimate the dual absolute
     *                 quadric (DAQ), which can be used to obtain pinhole camera intrinsic
     *                 parameters.
     * @param listener listener to be notified of events such as when
     *                 estimation starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if not enough cameras are provided
     *                                  for default settings. Hence, at least 2 cameras must be provided.
     */
    protected DualAbsoluteQuadricRobustEstimator(
            final List<PinholeCamera> cameras, final DualAbsoluteQuadricRobustEstimatorListener listener) {
        this(listener);
        internalSetCameras(cameras);
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
        return daqEstimator.isZeroSkewness();
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

        daqEstimator.setZeroSkewness(zeroSkewness);
    }

    /**
     * Returns boolean indicating whether principal point is assumed to be at
     * origin of coordinates or not.
     * Typically principal point is located at image center (origin of
     * coordinates), and usually matches the center of radial distortion if
     * it is taken into account.
     *
     * @return true if principal point is assumed to be at origin of
     * coordinates, false if principal point must be estimated
     */
    public boolean isPrincipalPointAtOrigin() {
        return daqEstimator.isPrincipalPointAtOrigin();
    }

    /**
     * Sets boolean indicating whether principal point is assumed to be at
     * origin of coordinates or not.
     * Typically principal point is located at image center (origin of
     * coordinates), and usually matches the center of radial distortion if it
     * is taken into account.
     *
     * @param principalPointAtOrigin true if principal point is assumed to be at
     *                               origin of coordinates, false if principal point must be estimated.
     * @throws LockedException if estimator is locked.
     */
    public void setPrincipalPointAtOrigin(final boolean principalPointAtOrigin) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        daqEstimator.setPrincipalPointAtOrigin(principalPointAtOrigin);
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
        return daqEstimator.isFocalDistanceAspectRatioKnown();
    }

    /**
     * Sets value indicating whether aspect ratio of focal distances (i.e.
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

        daqEstimator.setFocalDistanceAspectRatioKnown(focalDistanceAspectRatioKnown);
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
        return daqEstimator.getFocalDistanceAspectRatio();
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
     * aspect ratio
     * Notice that a negative aspect ratio indicates that vertical axis is
     * reversed. This can be useful in some situations where image vertical
     * coordinates are reversed respect to the physical world (i.e. in computer
     * graphics typically image vertical coordinates go downwards, while in
     * physical world they go upwards).
     *
     * @param focalDistanceAspectRatio aspect ratio of focal distances to be set.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if focal distance aspect ratio is too
     *                                  close to zero, as it might produce numerical instabilities.
     */
    public void setFocalDistanceAspectRatio(final double focalDistanceAspectRatio) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        daqEstimator.setFocalDistanceAspectRatio(focalDistanceAspectRatio);
    }

    /**
     * Indicates whether a singular DAQ is enforced or not.
     * Dual Absolute Quadric is singular (has rank 3) in any projective space,
     * however, due to noise in samples, estimated DAQ might not be fully
     * singular.
     *
     * @return true when singular DAQ is enforced, false otherwise.
     */
    public boolean isSingularityEnforced() {
        return daqEstimator.isSingularityEnforced();
    }

    /**
     * Specifies whether a singular DAQ is enforced or not.
     * Dual Absolute Quadric is singular (has rank 3) in any projective space,
     * however, due to noise in samples, estimated DAQ might not be fully
     * singular.
     *
     * @param singularityEnforced true when singular DAQ is enforced, false
     *                            otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setSingularityEnforced(final boolean singularityEnforced) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        daqEstimator.setSingularityEnforced(singularityEnforced);
    }

    /**
     * Indicates whether enforced singularity will be validated by checking that
     * determinant of estimated Dual Absolute Quadric (DAQ) is below a certain
     * threshold.
     *
     * @return true if enforced singularity is validated, false otherwise.
     */
    public boolean isEnforcedSingularityValidated() {
        return daqEstimator.isEnforcedSingularityValidated();
    }

    /**
     * Specifies whether enforced singularity will be validated by checking that
     * determinant of estimated Dual Absolute Quadric (DAQ) is below a certain
     * threshold.
     *
     * @param validateEnforcedSingularity true if enforced singularity is
     *                                    validated, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setEnforcedSingularityValidated(final boolean validateEnforcedSingularity) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        daqEstimator.setEnforcedSingularityValidated(validateEnforcedSingularity);
    }

    /**
     * Returns threshold to determine whether estimated Dual Absolute Quadric
     * (DAQ) has rank 3 or not when validation is enabled.
     *
     * @return threshold to determine whether estimated DAQ has rank 3 or not.
     */
    public double getDeterminantThreshold() {
        return daqEstimator.getDeterminantThreshold();
    }

    /**
     * Sets threshold to determine whether estimated Dual Absolute Quadric (DAQ)
     * has rank 3 or not when validation is enabled.
     *
     * @param determinantThreshold threshold to determine whether estimated DAQ
     *                             has rank 3 or not.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException          if estimator is locked.
     */
    public void setDeterminantThreshold(final double determinantThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        daqEstimator.setDeterminantThreshold(determinantThreshold);
    }

    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts, ends or its progress significantly changes.
     *
     * @return listener to be notified of events.
     */
    public DualAbsoluteQuadricRobustEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or its progress significantly changes.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if robust estimator is locked.
     */
    public void setListener(final DualAbsoluteQuadricRobustEstimatorListener listener) throws LockedException {
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
     * Indicates whether this instance is locked.
     *
     * @return true if this estimator is busy estimating the Dual Absolute
     * Quadric, false otherwise.
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
     * Obtains the list of cameras used to estimate the Dual Absolute Quadric
     * (DAQ).
     *
     * @return list of cameras to estimate the DAQ.
     */
    public List<PinholeCamera> getCameras() {
        return cameras;
    }

    /**
     * Sets the list of cameras used to estimate the Dual Absolute Quadric
     * (DAQ).
     *
     * @param cameras list of cameras used to estimate the DAQ.
     * @throws IllegalArgumentException if list is null.
     * @throws LockedException          if estimator is locked.
     */
    public final void setCameras(final List<PinholeCamera> cameras) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetCameras(cameras);
    }

    /**
     * Returns minimum number of required cameras needed to estimate the
     * Dual Absolute Quadric (DAQ).
     * At least 8 equations are needed to solve the DAQ
     * For each imposed constraint, one less equation is required.
     * Depending on the number of constraints more or less cameras will be
     * required.
     * If zero skewness is enforced, a solution is available with 8 cameras.
     * If zero skewness and focal distance aspect ratio is known, then a
     * solution is available with 4 cameras.
     * If principal point is located at origin, then a solution is available
     * with 4 cameras.
     * If zero skewness and principal point at origin are enforced, then a
     * solution is available with 3 cameras
     * If zero skewness is enforced, focal distance aspect ratio is known and
     * principal point is at origin, then a solution is available with 2
     * cameras.
     * NOTE: minimum number of cameras considers only the cameras providing
     * additional information. If a camera is equivalent to another one or does
     * not provide additional information (such as a camera at the origin with
     * no rotation), then more cameras will be needed.
     *
     * @return minimum number of required cameras needed to estimate the Dual
     * Absolute Quadric (DAQ) or -1 if constraints configurations is not valid.
     */
    public int getMinNumberOfRequiredCameras() {
        return daqEstimator.getMinNumberOfRequiredCameras();
    }

    /**
     * Indicates whether current constraints are enough to start the estimation.
     * In order to obtain a linear solution for the DAQ estimation, we need at
     * least the principal point at origin constraint.
     *
     * @return true if constraints are valid, false otherwise.
     */
    public boolean areValidConstraints() {
        return daqEstimator.areValidConstraints();
    }

    /**
     * Returns value indicating whether required data has been provided so that
     * DAQ estimation can start.
     * If true, estimator is ready to compute the DAQ, otherwise more data needs
     * to be provided.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return cameras != null && cameras.size() >= getMinNumberOfRequiredCameras() && areValidConstraints();
    }

    /**
     * Returns quality scores corresponding to each camera.
     * The larger the score value the better the quality of the camera.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @return quality scores corresponding to each camera.
     */
    public double[] getQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each camera.
     * The larger the score value the better the quality of the camera.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each camera.
     * @throws LockedException          if robust estimator is locked because an
     *                                  estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than minimum required number of cameras.
     */
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Estimates the Dual Absolute Quadric using provided cameras.
     *
     * @return estimated Dual Absolute Quadric (DAQ).
     * @throws LockedException          if robust estimator is locked.
     * @throws NotReadyException        if no valid input data has already been
     *                                  provided.
     * @throws RobustEstimatorException if estimation fails for any reason
     *                                  (i.e. numerical instability, no solution available, etc).
     */
    public abstract DualAbsoluteQuadric estimate() throws LockedException, NotReadyException, RobustEstimatorException;

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Creates a dual absolute quadric robust estimator using provided method.
     *
     * @param method method of a robust estimator algorithm to estimate best
     *               DAQ.
     * @return an instance of a dual absolute quadric robust estimator.
     */
    public static DualAbsoluteQuadricRobustEstimator create(final RobustEstimatorMethod method) {
        return switch (method) {
            case MSAC -> new MSACDualAbsoluteQuadricRobustEstimator();
            case RANSAC -> new RANSACDualAbsoluteQuadricRobustEstimator();
            case PROSAC -> new PROSACDualAbsoluteQuadricRobustEstimator();
            case PROMEDS -> new PROMedSDualAbsoluteQuadricRobustEstimator();
            default -> new LMedSDualAbsoluteQuadricRobustEstimator();
        };
    }

    /**
     * Creates a dual absolute quadric robust estimator using provided
     * cameras.
     *
     * @param cameras       list of cameras.
     * @param qualityScores quality scores corresponding to each camera.
     * @param method        method of a robust estimator algorithm to estimate best
     *                      DAQ.
     * @return an instance of a dual absolute quadric robust estimator.
     * @throws IllegalArgumentException if provided list of cameras and quality
     *                                  scores don't have the same size or size is too short.
     */
    public static DualAbsoluteQuadricRobustEstimator create(
            final List<PinholeCamera> cameras, final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case MSAC -> new MSACDualAbsoluteQuadricRobustEstimator(cameras);
            case RANSAC -> new RANSACDualAbsoluteQuadricRobustEstimator(cameras);
            case PROSAC -> new PROSACDualAbsoluteQuadricRobustEstimator(cameras, qualityScores);
            case PROMEDS -> new PROMedSDualAbsoluteQuadricRobustEstimator(cameras, qualityScores);
            default -> new LMedSDualAbsoluteQuadricRobustEstimator(cameras);
        };
    }

    /**
     * Creates a dual absolute quadric robust estimator using provided
     * cameras.
     *
     * @param cameras list of cameras.
     * @param method  method of a robust estimator algorithm to estimate
     *                best DAQ.
     * @return an instance of a dual absolute quadric robust estimator.
     * @throws IllegalArgumentException if provided list of cameras is too
     *                                  short.
     */
    public static DualAbsoluteQuadricRobustEstimator create(
            final List<PinholeCamera> cameras, final RobustEstimatorMethod method) {
        return switch (method) {
            case MSAC -> new MSACDualAbsoluteQuadricRobustEstimator(cameras);
            case RANSAC -> new RANSACDualAbsoluteQuadricRobustEstimator(cameras);
            case PROSAC -> new PROSACDualAbsoluteQuadricRobustEstimator(cameras);
            case PROMEDS -> new PROMedSDualAbsoluteQuadricRobustEstimator(cameras);
            default -> new LMedSDualAbsoluteQuadricRobustEstimator(cameras);
        };
    }

    /**
     * Creates a dual absolute quadric robust estimator using default method.
     *
     * @return an instance of a dual absolute quadric robust estimator.
     */
    public static DualAbsoluteQuadricRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual absolute quadric robust estimator using provided
     * cameras.
     *
     * @param cameras       list of cameras.
     * @param qualityScores quality scores corresponding to each camera.
     * @return an instance of a dual absolute quadric robust estimator.
     * @throws IllegalArgumentException if provided list of cameras and quality
     *                                  scores don't have the same size or size is too short.
     */
    public static DualAbsoluteQuadricRobustEstimator create(
            final List<PinholeCamera> cameras, final double[] qualityScores) {
        return create(cameras, qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a dual absolute quadric robust estimator using provided
     * cameras.
     *
     * @param cameras list of cameras.
     * @return an instance of a dual absolute quadric robust estimator.
     * @throws IllegalArgumentException if provided list of cameras is too
     *                                  short.
     */
    public static DualAbsoluteQuadricRobustEstimator create(final List<PinholeCamera> cameras) {
        return create(cameras, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Computes the residual between a dual absolute quadric (DAQ) and a pinhole
     * camera.
     *
     * @param daq    a dual absolute quadric (DAQ).
     * @param camera a camera.
     * @return residual.
     */
    protected double residual(final DualAbsoluteQuadric daq, final PinholeCamera camera) {

        daq.normalize();
        camera.normalize();
        final var cameraMatrix = camera.getInternalMatrix();

        final var p11 = cameraMatrix.getElementAt(0, 0);
        final var p21 = cameraMatrix.getElementAt(1, 0);
        final var p31 = cameraMatrix.getElementAt(2, 0);

        final var p12 = cameraMatrix.getElementAt(0, 1);
        final var p22 = cameraMatrix.getElementAt(1, 1);
        final var p32 = cameraMatrix.getElementAt(2, 1);

        final var p13 = cameraMatrix.getElementAt(0, 2);
        final var p23 = cameraMatrix.getElementAt(1, 2);
        final var p33 = cameraMatrix.getElementAt(2, 2);

        final var p14 = cameraMatrix.getElementAt(0, 3);
        final var p24 = cameraMatrix.getElementAt(1, 3);
        final var p34 = cameraMatrix.getElementAt(2, 3);

        var residual = 0.0;
        if (isPrincipalPointAtOrigin()) {
            if (isZeroSkewness()) {
                if (isFocalDistanceAspectRatioKnown()) {
                    // p2T*daq*p1 = 0
                    residual += residual2ndRowAnd1stRow(daq, p11, p21, p12, p22, p13, p23, p14, p24);

                    // p3T*daq*p1 = 0
                    residual += residual3rdRowAnd1stRow(daq, p11, p31, p12, p32, p13, p33, p14, p34);

                    // p3T*daw*p2 = 0
                    residual += residual3rdRowAnd2ndRow(daq, p21, p31, p22, p32, p23, p33, p24, p34);

                    // p1T*daq*p1*aspectRatio^2 = p2T*daq*p1
                    residual += residual1stRowEqualTo2ndRow(daq, p11, p21, p12, p22, p13, p23, p14, p24);
                } else {
                    // p2T*daq*p1 = 0
                    residual += residual2ndRowAnd1stRow(daq, p11, p21, p12, p22, p13, p23, p14, p24);

                    // p3T*daq*p1 = 0
                    residual += residual3rdRowAnd1stRow(daq, p11, p31, p12, p32, p13, p33, p14, p34);

                    //p3T*daq*p2 = 0
                    residual += residual3rdRowAnd2ndRow(daq, p21, p31, p22, p32, p23, p33, p24, p34);
                }
            } else {
                // p3T*daq*p1 = 0
                residual += residual3rdRowAnd1stRow(daq, p11, p31, p12, p32, p13, p33, p14, p34);

                // p3T*daq*p2 = 0
                residual += residual3rdRowAnd2ndRow(daq, p21, p31, p22, p32, p23, p33, p24, p34);
            }
        } else {
            return Double.MAX_VALUE;
        }

        return residual;
    }

    /**
     * Computes residual for the equation p2T*daq*p1.
     *
     * @param daq estimated DAQ.
     * @param p11 element (1,1) of camera matrix.
     * @param p21 element (2,1) of camera matrix.
     * @param p12 element (1,2) of camera matrix.
     * @param p22 element (2,2) of camera matrix.
     * @param p13 element (1,3) of camera matrix.
     * @param p23 element (2,3) of camera matrix.
     * @param p14 element (1,4) of camera matrix.
     * @param p24 element (2,4) of camera matrix.
     * @return obtained residual (ideally should be zero).
     */
    private double residual2ndRowAnd1stRow(
            final DualAbsoluteQuadric daq, final double p11, final double p21, final double p12, final double p22,
            final double p13, final double p23, final double p14, final double p24) {

        final var a = daq.getA();
        final var b = daq.getB();
        final var c = daq.getC();
        final var d = daq.getD();
        final var e = daq.getE();
        final var f = daq.getF();
        final var g = daq.getG();
        final var h = daq.getH();
        final var i = daq.getI();
        final var j = daq.getJ();

        return a * p21 * p11 + b * p22 * p12 + c * p23 * p13 + d * (p22 * p11 + p21 * p12)
                + e * (p23 * p12 + p22 * p13) + f * (p23 * p11 + p21 * p13)
                + g * (p24 * p11 + p21 * p14) + h * (p24 * p12 + p22 * p14)
                + i * (p24 * p13 + p23 * p14) + j * p24 * p14;
    }

    /**
     * Computes residual for the equation p3T*daq*p1.
     *
     * @param daq estimated DAQ.
     * @param p11 element (1,1) of camera matrix.
     * @param p31 element (3,1) of camera matrix.
     * @param p12 element (1,2) of camera matrix.
     * @param p32 element (3,2) of camera matrix.
     * @param p13 element (1,3) of camera matrix.
     * @param p33 element (3,3) of camera matrix.
     * @param p14 element (1,4) of camera matrix.
     * @param p34 element (3,4) of camera matrix.
     * @return obtained residual (ideally should be zero).
     */
    private double residual3rdRowAnd1stRow(
            final DualAbsoluteQuadric daq, final double p11, final double p31, final double p12, final double p32,
            final double p13, final double p33, final double p14, final double p34) {

        final var a = daq.getA();
        final var b = daq.getB();
        final var c = daq.getC();
        final var d = daq.getD();
        final var e = daq.getE();
        final var f = daq.getF();
        final var g = daq.getG();
        final var h = daq.getH();
        final var i = daq.getI();
        final var j = daq.getJ();

        return a * p31 * p11 + b * p32 * p12 + c * p33 * p13 + d * (p32 * p11 + p32 * p12)
                + e * (p33 * p12 + p32 * p13) + f * (p33 * p11 + p31 * p13)
                + g * (p34 * p11 + p31 * p14) + h * (p34 * p12 + p32 * p14)
                + i * (p34 * p13 + p13 * p14) + j * p34 * p14;
    }

    /**
     * Computes residual for the equation p3T*daq*p2.
     *
     * @param daq estimated DAQ.
     * @param p21 element (2,1) of camera matrix.
     * @param p31 element (3,1) of camera matrix.
     * @param p22 element (2,2) of camera matrix.
     * @param p32 element (3,2) of camera matrix.
     * @param p23 element (2,3) of camera matrix.
     * @param p33 element (3,3) of camera matrix.
     * @param p24 element (2,4) of camera matrix.
     * @param p34 element (3,4) of camera matrix.
     * @return obtained residual (ideally should be zero).
     */
    private double residual3rdRowAnd2ndRow(
            final DualAbsoluteQuadric daq, final double p21, final double p31, final double p22, final double p32,
            final double p23, final double p33, final double p24, final double p34) {

        final var a = daq.getA();
        final var b = daq.getB();
        final var c = daq.getC();
        final var d = daq.getD();
        final var e = daq.getE();
        final var f = daq.getF();
        final var g = daq.getG();
        final var h = daq.getH();
        final var i = daq.getI();
        final var j = daq.getJ();

        return a * p31 * p21 + b * p32 * p22 + c * p33 * p23 + d * (p32 * p21 + p31 * p22)
                + e * (p33 * p22 + p32 * p23) + f * (p33 * p21 + p31 * p23)
                + g * (p34 * p21 + p31 * p24) + h * (p34 * p22 + p32 * p24)
                + i * (p34 * p23 + p33 * p24) + j * p34 * p24;
    }

    /**
     * Computes residual for the equation p1T*daq*p1 = r^2*p2T*daq*p2.
     *
     * @param daq estimated DAQ.
     * @param p11 element (1,1) of camera matrix.
     * @param p21 element (2,1) of camera matrix.
     * @param p12 element (1,2) of camera matrix.
     * @param p22 element (2,2) of camera matrix.
     * @param p13 element (1,3) of camera matrix.
     * @param p23 element (2,3) of camera matrix.
     * @param p14 element (1,4) of camera matrix.
     * @param p24 element (2,4) of camera matrix.
     * @return obtained residual (ideally should be zero).
     */
    private double residual1stRowEqualTo2ndRow(
            final DualAbsoluteQuadric daq, final double p11, final double p21, final double p12, final double p22,
            final double p13, final double p23, final double p14, final double p24) {

        final var r = getFocalDistanceAspectRatio();
        final var r2 = r * r;

        final var a = daq.getA();
        final var b = daq.getB();
        final var c = daq.getC();
        final var d = daq.getD();
        final var e = daq.getE();
        final var f = daq.getF();
        final var g = daq.getG();
        final var h = daq.getH();
        final var i = daq.getI();
        final var j = daq.getJ();

        return a * (p11 * p11 * r2 - p21 * p21) + b * (p12 * p12 * r2 - p22 * p22)
                + c * (p13 * p13 * r2 - p23 * p23) + d * 2.0 * (p12 * p11 * r2 - p22 * p21)
                + e * 2.0 * (p13 * p12 * r2 - p23 * p22) + f * 2.0 * (p13 * p11 * r2 - p23 * p21)
                + g * 2.0 * (p14 * p11 * r2 - p24 * p21) + h * 2.0 * (p14 * p12 * r2 - p24 * p22)
                + i * 2.0 * (p14 * p13 * r2 - p24 * p23) + j * (p14 * p14 * r2 - p24 * p24);
    }

    /**
     * Sets list of cameras.
     * This method does not check whether estimator is locked.
     *
     * @param cameras list of cameras to estimate DAQ.
     * @throws IllegalArgumentException if provided list of cameras is null
     *                                  or too small.
     */
    private void internalSetCameras(final List<PinholeCamera> cameras) {
        if (cameras == null || cameras.size() < getMinNumberOfRequiredCameras()) {
            throw new IllegalArgumentException();
        }
        this.cameras = cameras;
    }
}
