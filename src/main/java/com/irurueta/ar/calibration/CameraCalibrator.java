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

import com.irurueta.ar.calibration.estimators.ImageOfAbsoluteConicRobustEstimator;
import com.irurueta.ar.calibration.estimators.ImageOfAbsoluteConicRobustEstimatorListener;
import com.irurueta.ar.calibration.estimators.LMedSImageOfAbsoluteConicRobustEstimator;
import com.irurueta.ar.calibration.estimators.MSACImageOfAbsoluteConicRobustEstimator;
import com.irurueta.ar.calibration.estimators.PROMedSImageOfAbsoluteConicRobustEstimator;
import com.irurueta.ar.calibration.estimators.PROSACImageOfAbsoluteConicRobustEstimator;
import com.irurueta.ar.calibration.estimators.RANSACImageOfAbsoluteConicRobustEstimator;
import com.irurueta.geometry.GeometryException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.estimators.*;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Calibrates a camera in order to find its intrinsic parameters and other
 * parameters such as radial distortion.
 */
public abstract class CameraCalibrator {

    /**
     * Default robust estimator method to be used for homography estimations.
     */
    public static final RobustEstimatorMethod DEFAULT_HOMOGRAPHY_METHOD = RobustEstimatorMethod.PROSAC;

    /**
     * Default robust estimator method to be used for IAC estimation.
     */
    public static final RobustEstimatorMethod DEFAULT_IAC_METHOD = RobustEstimatorMethod.PROSAC;

    /**
     * Indicates whether radial distortion must be estimated or not by default.
     */
    public static final boolean DEFAULT_ESTIMATE_RADIAL_DISTORTION = true;

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
     * Default method used for camera calibration.
     * The default method uses an alternating technique where first intrinsic
     * parameters and camera pose are estimated without accounting for
     * distortion and then the results are used to obtain an initial guess
     * for distortion, which is then used to correct initially sampled points
     * and repeat the whole process until convergence is achieved.
     */
    public static final CameraCalibratorMethod DEFAULT_METHOD = CameraCalibratorMethod.ERROR_OPTIMIZATION;

    /**
     * Pattern used for camera calibration. Each pattern contains a unique
     * combination of 2D points that must be sampled using the camera to be
     * calibrated.
     */
    protected Pattern2D pattern;

    /**
     * List of samples obtained from different pictures using the same camera
     * device (or same camera model). Several samples can be used to calibrate
     * the camera. The more samples are used, typically the better the results.
     */
    protected List<CameraCalibratorSample> samples;

    /**
     * Quality scores for samples. This can be used on certain
     * robust estimation methods of the IAC such as PROSAC and PROMedS.
     * If not provided, homography quality scores will be estimated based on
     * re-projection error and this value will be ignored.
     * Typically, this will not be provided, but it can be used in case that it
     * can be assured by some means that one sample is better than another.
     */
    protected double[] samplesQualityScores;

    /**
     * Estimated homographies from provided list of samples respect to provided
     * pattern.
     */
    protected List<Transformation2D> homographies;

    /**
     * Quality scores for estimated homographies to be used during IAC
     * estimation when PROSAC or PROMedS robust method is used. This value
     * is only computed when no samples quality scores are provided.
     * Homography quality scores are obtained based on re-projection error of
     * marker coordinates.
     */
    protected double[] homographyQualityScores;

    /**
     * Indicates whether homography quality scores need to be estimated if
     * samples quality scores are not provided.
     */
    protected boolean homographyQualityScoresRequired;

    /**
     * Estimated image of absolute conic. This can be used to obtain intrinsic
     * pinhole camera intrinsic parameters.
     */
    protected ImageOfAbsoluteConic iac;

    /**
     * Estimated intrinsic pinhole camera parameters. Intrinsic parameters
     * contain data related to the camera sensor such as focal length,
     * skewness or principal point. Except focal length, typically intrinsic
     * parameters are fixed, and even in some situations such as when camera
     * lens is fixed, focal length also remains constant. Because the latter is
     * true in most phone cameras, it can be considered that intrinsic
     * parameters remain constant for all phones of the same maker and model,
     * and for that reason a calibrator can be used with pictures taken from
     * different phones as long as they are the same phone model.
     */
    protected PinholeCameraIntrinsicParameters intrinsic;

    /**
     * Estimated radial distortion. Radial distortion is inherent to the camera
     * lens, and remains constant as long as the lens doesn't change.
     * Because in most phone cameras the lens remains constant, lens distortion
     * can be modeled once for each phone model, and for that reason a single
     * calibrator can be used with pictures taken from different phones as long
     * as they are the same phone model.
     */
    protected RadialDistortion distortion;

    /**
     * Indicates whether radial distortion must be estimated or not.
     */
    protected boolean estimateRadialDistortion;

    /**
     * Robust estimator method to be used during homography estimation.
     * This will only be taken into account if more than 4 markers are detected
     * on a single sample, otherwise no robust method is used and a single LMSE
     * solution for the homography is found.
     */
    protected RobustEstimatorMethod homographyMethod;

    /**
     * Robust estimator method to be used during IAC estimation.
     * This will only be taken into account if more than 1 sample is provided,
     * otherwise no robust method is used and a single LMSE solution for the
     * IAC is found.
     */
    protected RobustEstimatorMethod imageOfAbsoluteConicMethod;

    /**
     * Robust estimator of homographies between sampled markers and ideal
     * pattern markers.
     */
    protected PointCorrespondenceProjectiveTransformation2DRobustEstimator homographyEstimator;

    /**
     * Robust estimator of the Image of Absolute Conic (IAC).
     */
    protected ImageOfAbsoluteConicRobustEstimator iacEstimator;

    /**
     * Listener for homography estimator.
     */
    protected ProjectiveTransformation2DRobustEstimatorListener homographyEstimatorListener;

    /**
     * Listener for image of absolute conic estimator.
     */
    protected ImageOfAbsoluteConicRobustEstimatorListener iacEstimatorListener;

    /**
     * Indicates whether this instance is locked because calibration is in
     * progress.
     */
    protected volatile boolean locked;

    /**
     * Amount of progress variation before notifying a progress change during
     * estimation.
     */
    protected float progressDelta;

    /**
     * Listener to notify when calibration starts, finishes or its progress
     * significantly changes.
     */
    protected CameraCalibratorListener listener;

    /**
     * Indicates progress of homography estimation.
     */
    protected float homographyProgress;

    /**
     * Indicates progress of homography estimation for all samples.
     */
    protected float sampleProgress;

    /**
     * Indicates progress of IAC estimation.
     */
    protected float iacProgress;

    /**
     * Indicates progress of estimation of intrinsic parameters.
     */
    protected float intrinsicProgress;

    /**
     * Constructor.
     */
    protected CameraCalibrator() {
        pattern = null;
        homographies = null;
        homographyQualityScores = null;
        iac = null;
        intrinsic = null;
        distortion = null;
        estimateRadialDistortion = DEFAULT_ESTIMATE_RADIAL_DISTORTION;

        internalSetHomographyMethod(DEFAULT_HOMOGRAPHY_METHOD);
        internalSetImageOfAbsoluteConicMethod(DEFAULT_IAC_METHOD);

        samples = null;
        samplesQualityScores = null;

        progressDelta = DEFAULT_PROGRESS_DELTA;
    }

    /**
     * Constructor.
     *
     * @param pattern 2D pattern to use for calibration.
     * @param samples samples of the pattern taken with the camera to calibrate.
     * @throws IllegalArgumentException if not enough samples are provided.
     */
    protected CameraCalibrator(final Pattern2D pattern, final List<CameraCalibratorSample> samples) {
        this.pattern = pattern;
        homographies = null;
        homographyQualityScores = null;
        iac = null;
        intrinsic = null;
        distortion = null;
        estimateRadialDistortion = DEFAULT_ESTIMATE_RADIAL_DISTORTION;

        internalSetHomographyMethod(DEFAULT_HOMOGRAPHY_METHOD);
        internalSetImageOfAbsoluteConicMethod(DEFAULT_IAC_METHOD);

        internalSetSamples(samples);
        samplesQualityScores = null;

        progressDelta = DEFAULT_PROGRESS_DELTA;
    }

    /**
     * Constructor.
     *
     * @param pattern              2D pattern to use for calibration.
     * @param samples              samples of the pattern taken with the camera to calibrate.
     * @param samplesQualityScores quality scores for each sample.
     * @throws IllegalArgumentException if not enough samples are provided or if
     *                                  both samples and quality scores do not have the same size.
     */
    protected CameraCalibrator(final Pattern2D pattern, final List<CameraCalibratorSample> samples,
                               final double[] samplesQualityScores) {
        if (samples.size() != samplesQualityScores.length) {
            throw new IllegalArgumentException();
        }

        this.pattern = pattern;
        homographies = null;
        homographyQualityScores = null;
        iac = null;
        intrinsic = null;
        distortion = null;
        estimateRadialDistortion = DEFAULT_ESTIMATE_RADIAL_DISTORTION;

        internalSetHomographyMethod(DEFAULT_HOMOGRAPHY_METHOD);
        internalSetImageOfAbsoluteConicMethod(DEFAULT_IAC_METHOD);

        internalSetSamples(samples);
        internalSetSamplesQualityScores(samplesQualityScores);

        progressDelta = DEFAULT_PROGRESS_DELTA;
    }

    /**
     * Returns pattern used for camera calibration. Each pattern contain a
     * unique combination of 2D points that must be sampled using the camera to
     * be calibrated.
     *
     * @return pattern used for camera calibration.
     */
    public Pattern2D getPattern() {
        return pattern;
    }

    /**
     * Sets pattern used for camera calibration. Each pattern contains a unique
     * combination of 2D points that must be sampled using the camera to be
     * calibrated.
     *
     * @param pattern pattern used for camera calibration.
     * @throws LockedException if this instance is locked.
     */
    public void setPattern(final Pattern2D pattern) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.pattern = pattern;
    }

    /**
     * Returns list of samples obtained from different pictures using the same
     * camera device (or same camera model). Several samples can be used to
     * calibrate the camera (a pinhole camera can be estimated for each sample).
     * The more samples are used, typically the better the results.
     *
     * @return list of samples.
     */
    public List<CameraCalibratorSample> getSamples() {
        return samples;
    }

    /**
     * Sets list of samples obtained from different pictures using the same
     * camera device (or same camera model). Several samples can be used to
     * calibrate the camera (a pinhole camera can be estimated for each sample).
     * The more samples are used, typically the better the results.
     *
     * @param samples list of samples.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if not enough samples are provided to
     *                                  estimate the intrinsic parameters. By default, the minimum is 1,
     *                                  but depending on the settings at least 3 samples might be required.
     */
    public void setSamples(final List<CameraCalibratorSample> samples) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSamples(samples);
    }

    /**
     * Returns quality scores assigned to each provided sample. This can be used
     * on certain robust estimation methods of the IAC such as PROSAC and
     * PROMedS. If not provided, homography quality scores will be estimated
     * based on re-projection error and this value will be ignored.
     * Typically, this will not be provided, but it can be used in case that it
     * can be assured by some means that one sample is better than another
     *
     * @return quality scores assigned to each provided sample.
     */
    public double[] getSamplesQualityScores() {
        return samplesQualityScores;
    }

    /**
     * Sets quality scores assigned to each provided sample. This can be used on
     * certain robust estimation methods of the IAC such as PROSAC and PROMedS.
     * If not provided, homography quality scores will be estimated based on
     * re-projection error and this value will be ignored.
     *
     * @param samplesQualityScores quality scores assigned to each provided
     *                             sample.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if not enough quality scores are
     *                                  provided for the corresponding samples to estimate the intrinsic
     *                                  parameters. By default, the minimum is 1, but depending on the
     *                                  settings at least 3 samples might be required.
     */
    public void setSamplesQualityScores(final double[] samplesQualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetSamplesQualityScores(samplesQualityScores);
    }

    /**
     * Returns quality scores for estimated homographies.
     * If samples quality scores were provided, these will be equal
     * to those provided. If no samples quality scores were provided,
     * these scores will be related to the estimated homography
     * estimation error based on sampled data and ideal data.
     * This should rarely be used. It can be used for debugging purposes
     * teo determine whether homographies used for calibration are
     * reliable or not.
     *
     * @return estimated quality scores for homographies.
     */
    public double[] getHomographyQualityScores() {
        return homographyQualityScores;
    }

    /**
     * Returns estimated image of absolute conic. This can be used to obtain
     * intrinsic pinhole camera intrinsic parameters.
     *
     * @return estimated image of absolute conic or null if estimation has not
     * been completed.
     */
    public ImageOfAbsoluteConic getEstimatedImageOfAbsoluteConic() {
        return iac;
    }

    /**
     * Returns estimated pinhole camera intrinsic parameters.
     *
     * @return estimated pinhole camera intrinsic parameters or null if
     * estimation has not been completed.
     */
    public PinholeCameraIntrinsicParameters getEstimatedIntrinsicParameters() {
        return intrinsic;
    }

    /**
     * Returns estimated radial distortion due to camera lens.
     *
     * @return estimated radial distortion or null if estimation has not
     * completed or radial distortion was not requested.
     */
    public RadialDistortion getDistortion() {
        return distortion;
    }

    /**
     * Returns boolean indicating whether radial distortion must be estimated or
     * not during calibration.
     *
     * @return true if radial distortion must be estimated, false otherwise.
     */
    public boolean getEstimateRadialDistortion() {
        return estimateRadialDistortion;
    }

    /**
     * Sets boolean indicating whether radial distortion must be estimated or
     * not during calibration.
     *
     * @param estimateRadialDistortion true if radial distortion must be
     *                                 estimated, false otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setEstimateRadialDistortion(final boolean estimateRadialDistortion) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.estimateRadialDistortion = estimateRadialDistortion;
    }

    /**
     * Returns robust estimator method to be used during homography estimation.
     * This will only be taken into account if more than 4 markers are detected
     * on a single sample, otherwise no robust method is used and a single LMSE
     * solution for the homography is found.
     *
     * @return robust estimator method to be used during homography estimation.
     */
    public RobustEstimatorMethod getHomographyMethod() {
        return homographyMethod;
    }

    /**
     * Sets robust estimator method to be used during homography estimation.
     * This will only be taken into account if more than 4 markers are detected
     * on a single sample, otherwise no robust method is used and a single LMSE
     * solution for the homography is found.
     *
     * @param homographyMethod robust estimator method to be used during
     *                         homography estimation.
     * @throws LockedException if this instance is locked.
     */
    public void setHomographyMethod(final RobustEstimatorMethod homographyMethod) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetHomographyMethod(homographyMethod);
    }

    /**
     * Returns robust estimator method to be used during IAC estimation.
     * This will only be taken into account if more than 1 sample is provided,
     * otherwise no robust method is used and a single LMSE solution for the
     * IAC is found
     *
     * @return robust estimator method to be used during IAC estimation
     */
    public RobustEstimatorMethod getImageOfAbsoluteConicMethod() {
        return imageOfAbsoluteConicMethod;
    }

    /**
     * Sets robust estimator method to be used during IAC estimation.
     * This will only be taken into account if more than 1 sample is provided,
     * otherwise no robust method is used and a single LMSE solution for the
     * IAC is found.
     *
     * @param imageOfAbsoluteConicMethod robust estimator method to be used
     *                                   during IAC estimation.
     * @throws LockedException if this instance is locked.
     */
    public void setImageOfAbsoluteConicMethod(final RobustEstimatorMethod imageOfAbsoluteConicMethod)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetImageOfAbsoluteConicMethod(imageOfAbsoluteConicMethod);
    }

    /**
     * Returns homography estimator, which can be retrieved in case that some
     * additional parameter needed to be adjusted.
     * It is discouraged to directly access the homography estimator during
     * camera calibration, as it might interfere with the results.
     *
     * @return homography estimator.
     */
    public PointCorrespondenceProjectiveTransformation2DRobustEstimator getHomographyEstimator() {
        return homographyEstimator;
    }

    /**
     * Returns IAC estimator, which can be retrieved in case that some
     * additional parameter needed to be adjusted.
     * It is discouraged to directly access the homography estimator during
     * camera calibration, as it might interfere with the results.
     *
     * @return IAC estimator.
     */
    public ImageOfAbsoluteConicRobustEstimator getIACEstimator() {
        return iacEstimator;
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
     * @throws LockedException if this instance is locked.
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
     * coordinates), and usually matches the center of radial distortion if it
     * is taken into account.
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
     *                               at origin of coordinates, false if principal point must be estimated.
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
     * @param focalDistanceAspectRatio aspect ratio of focal distances to be set.
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
     * Indicates whether this instance is locked because calibration is in
     * progress.
     *
     * @return true if this instance, false otherwise.
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
     * Indicates whether this instance is ready to start camera calibration
     *
     * @return true if this instance has enough data to start camera
     * calibration, false otherwise.
     */
    public boolean isReady() {
        return pattern != null && samples != null
                && samples.size() >= iacEstimator.getMinNumberOfRequiredHomographies();
    }

    /**
     * Returns threshold to robustly estimate homographies between ideal pattern
     * markers and sampled pattern markers.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments
     *
     * @return threshold to robustly estimate homographies between ideal pattern
     * markers and sampled pattern markers.
     */
    public double getHomographyEstimatorThreshold() {
        return switch (homographyEstimator.getMethod()) {
            case LMEDS -> ((LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator)
                    .getStopThreshold();
            case MSAC -> ((MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator)
                    .getThreshold();
            case PROSAC -> ((PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator)
                    .getThreshold();
            case PROMEDS -> ((PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator)
                    .getStopThreshold();
            default -> ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator)
                    .getThreshold();
        };
    }

    /**
     * Sets threshold to robustly estimate homographies between ideal pattern
     * markers and sampled pattern markers.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @param homographyEstimatorThreshold threshold to robustly estimate
     *                                     homographies between ideal pattern markers and sampled pattern markers.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setHomographyEstimatorThreshold(final double homographyEstimatorThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        switch (homographyEstimator.getMethod()) {
            case LMEDS:
                ((LMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator).
                        setStopThreshold(homographyEstimatorThreshold);
                break;
            case MSAC:
                ((MSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator).
                        setThreshold(homographyEstimatorThreshold);
                break;
            case PROSAC:
                ((PROSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator).
                        setThreshold(homographyEstimatorThreshold);
                break;
            case PROMEDS:
                ((PROMedSPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator).
                        setStopThreshold(homographyEstimatorThreshold);
                break;
            case RANSAC:
            default:
                ((RANSACPointCorrespondenceProjectiveTransformation2DRobustEstimator) homographyEstimator).
                        setThreshold(homographyEstimatorThreshold);
                break;
        }
    }

    /**
     * Returns confidence to robustly estimate homographies between ideal
     * pattern markers and sampled pattern markers.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     * Confidence is expressed as a value between 0.0 (0%) and 1.0 (100%). The
     * amount of confidence indicates the probability that the estimated
     * homography is correct (i.e. no outliers were used for the estimation,
     * because they were successfully discarded).
     * Typically, this value will be close to 1.0, but not exactly 1.0, because
     * a 100% confidence would require an infinite number of iterations.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @return confidence to robustly estimate homographies.
     */
    public double getHomographyEstimatorConfidence() {
        return homographyEstimator.getConfidence();
    }

    /**
     * Sets confidence to robustly estimate homographies between ideal pattern
     * markers and sampled pattern markers.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     * Confidence is expressed as a value between 0.0 (0%) and 1.0 (100%). The
     * amount of confidence indicates the probability that the estimated
     * homography is correct (i.e. no outliers were used for the estimation,
     * because they were successfully discarded).
     * Typically, this value will be close to 1.0, but not exactly 1.0, because
     * a 100% confidence would require an infinite number of iterations.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @param homographyEstimatorConfidence confidence to robustly estimate
     *                                      homographies.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is not between 0.0 and
     *                                  1.0.
     */
    public void setHomographyEstimatorConfidence(final double homographyEstimatorConfidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        homographyEstimator.setConfidence(homographyEstimatorConfidence);
    }

    /**
     * Returns the maximum number of iterations to be done when estimating
     * the homographies between ideal pattern markers and sampled pattern
     * markers.
     * If the maximum allowed number of iterations is reached, resulting
     * estimation might not have desired confidence.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @return maximum number of iterations to be done when estimating the
     * homographies.
     */
    public int getHomographyEstimatorMaxIterations() {
        return homographyEstimator.getMaxIterations();
    }

    /**
     * Sets the maximum number of iterations to be done when estimating the
     * homographies between ideal pattern markers and sampled pattern markers.
     * If the maximum allowed number of iterations is reached, resulting
     * estimation might not have desired confidence.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @param homographyEstimatorMaxIterations maximum number of iterations to
     *                                         be done when estimating the homographies between ideal
     *                                         pattern markers and sampled pattern markers.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is negative or zero.
     */
    public void setHomographyEstimatorMaxIterations(final int homographyEstimatorMaxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        homographyEstimator.setMaxIterations(homographyEstimatorMaxIterations);
    }

    /**
     * Returns threshold to robustly estimate the image of absolute conic.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @return threshold to robustly estimate the image of absolute conic.
     */
    public double getIACEstimatorThreshold() {
        return switch (iacEstimator.getMethod()) {
            case LMEDS -> ((LMedSImageOfAbsoluteConicRobustEstimator) iacEstimator).getStopThreshold();
            case MSAC -> ((MSACImageOfAbsoluteConicRobustEstimator) iacEstimator).getThreshold();
            case PROSAC -> ((PROSACImageOfAbsoluteConicRobustEstimator) iacEstimator).getThreshold();
            case PROMEDS -> ((PROMedSImageOfAbsoluteConicRobustEstimator) iacEstimator).getStopThreshold();
            default -> ((RANSACImageOfAbsoluteConicRobustEstimator) iacEstimator).getThreshold();
        };
    }

    /**
     * Sets threshold to robustly estimate the image of absolute conic.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @param iacEstimatorThreshold threshold to robustly estimate the image of
     *                              absolute conic.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setIACEstimatorThreshold(final double iacEstimatorThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        switch (iacEstimator.getMethod()) {
            case LMEDS:
                ((LMedSImageOfAbsoluteConicRobustEstimator) iacEstimator).setStopThreshold(iacEstimatorThreshold);
                break;
            case MSAC:
                ((MSACImageOfAbsoluteConicRobustEstimator) iacEstimator).setThreshold(iacEstimatorThreshold);
                break;
            case PROSAC:
                ((PROSACImageOfAbsoluteConicRobustEstimator) iacEstimator).setThreshold(iacEstimatorThreshold);
                break;
            case PROMEDS:
                ((PROMedSImageOfAbsoluteConicRobustEstimator) iacEstimator).setStopThreshold(iacEstimatorThreshold);
                break;
            case RANSAC:
            default:
                ((RANSACImageOfAbsoluteConicRobustEstimator) iacEstimator).setThreshold(iacEstimatorThreshold);
                break;
        }
    }

    /**
     * Returns confidence to robustly estimate image of absolute conic
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     * Confidence is expressed as a value between 0.0 (0%) and 1.0 (100%). The
     * amount of confidence indicates the probability that the estimated
     * homography is correct (i.e. no outliers were used for the estimation,
     * because they were successfully discarded).
     * Typically, this value will be close to 1.0, but not exactly 1.0, because
     * a 100% confidence would require an infinite number of iterations.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @return confidence to robustly estimate the IAC.
     */
    public double getIACEstimatorConfidence() {
        return iacEstimator.getConfidence();
    }

    /**
     * Sets confidence to robustly estimate image of absolute conic.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     * Confidence is expressed as a value between 0.0 (0%) and 1.0 (100%). The
     * amount of confidence indicates the probability that the estimated
     * homography is correct (i.e. no outliers were used for the estimation,
     * because they were successfully discarded).
     * Typically, this value will be close to 1.0, but not exactly 1.0, because
     * a 100% confidence would require an infinite number of iterations.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @param iacEstimatorConfidence confidence to robustly estimate the IAC.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is not between 0.0 and
     *                                  1.0.
     */
    public void setIACEstimatorConfidence(final double iacEstimatorConfidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        iacEstimator.setConfidence(iacEstimatorConfidence);
    }

    /**
     * Returns the maximum number of iterations to be done when estimating
     * the image of absolute conic.
     * If the maximum allowed number of iterations is reached, resulting
     * estimation might not have desired confidence.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @return maximum number of iterations to be done when estimating the
     * image of absolute conic.
     */
    public int getIACEstimatorMaxIterations() {
        return iacEstimator.getMaxIterations();
    }

    /**
     * Sets the maximum number of iterations to be done when estimating the
     * image of absolute conic.
     * If the maximum allowed number of iterations is reached, resulting
     * estimation might not have desired confidence.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @param iacEstimatorMaxIterations maximum number of iterations to be done
     *                                  when estimating the image of absolute conic.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is negative or zero.
     */
    public void setIACEstimatorMaxIterations(final int iacEstimatorMaxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        iacEstimator.setMaxIterations(iacEstimatorMaxIterations);
    }

    /**
     * Returns listener to notify when calibration starts, finishes or its
     * progress significantly changes.
     *
     * @return listener to notify when calibration starts, finishes or its
     * progress significantly changes.
     */
    public CameraCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to notify when calibration starts, finishes or its progress
     * significantly changes.
     *
     * @param listener listener to notify when calibration starts, finishes or
     *                 its progress significantly changes.
     * @throws LockedException if this instance is locked.
     */
    public void setListener(final CameraCalibratorListener listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Creates a camera calibrator using provided method.
     *
     * @param method a camera calibrator method.
     * @return a camera calibrator.
     */
    public static CameraCalibrator create(final CameraCalibratorMethod method) {
        return method == CameraCalibratorMethod.ERROR_OPTIMIZATION
                ? new ErrorOptimizationCameraCalibrator() : new AlternatingCameraCalibrator();
    }

    /**
     * Creates a camera calibrator using provided pattern, samples and
     * method.
     *
     * @param pattern a 2D pattern to use for calibration.
     * @param samples samples of the 2D pattern taken with the camera to be
     *                calibrated.
     * @param method  a camera calibrator method.
     * @return a camera calibrator.
     * @throws IllegalArgumentException if not enough samples are provided.
     */
    public static CameraCalibrator create(final Pattern2D pattern, final List<CameraCalibratorSample> samples,
                                          final CameraCalibratorMethod method) {
        return method == CameraCalibratorMethod.ERROR_OPTIMIZATION
                ? new ErrorOptimizationCameraCalibrator(pattern, samples)
                : new AlternatingCameraCalibrator(pattern, samples);
    }

    /**
     * Creates a camera calibrator using provided pattern, samples and method.
     *
     * @param pattern              a 2D pattern to use for calibration.
     * @param samples              samples of the 2D pattern taken with the camera to be
     *                             calibrated.
     * @param samplesQualityScores quality scores for each sample.
     * @param method               a camera calibrator method.
     * @return a camera calibrator.
     * @throws IllegalArgumentException if not enough samples are provided.
     */
    public static CameraCalibrator create(
            final Pattern2D pattern, final List<CameraCalibratorSample> samples, final double[] samplesQualityScores,
            final CameraCalibratorMethod method) {
        return method == CameraCalibratorMethod.ERROR_OPTIMIZATION
                ? new ErrorOptimizationCameraCalibrator(pattern, samples, samplesQualityScores)
                : new AlternatingCameraCalibrator(pattern, samples, samplesQualityScores);
    }

    /**
     * Creates a camera calibrator using default method.
     *
     * @return a camera calibrator.
     */
    public static CameraCalibrator create() {
        return create(DEFAULT_METHOD);
    }

    /**
     * Creates a camera calibrator using provided pattern, samples and
     * default method.
     *
     * @param pattern a 2D pattern to use for calibration.
     * @param samples samples of the 2D pattern taken with the camera to be
     *                calibrated.
     * @return a camera calibrator.
     * @throws IllegalArgumentException if not enough samples are provided.
     */
    public static CameraCalibrator create(final Pattern2D pattern, final List<CameraCalibratorSample> samples) {
        return create(pattern, samples, DEFAULT_METHOD);
    }

    /**
     * Creates a camera calibrator using provided pattern, samples and default
     * method.
     *
     * @param pattern              a 2D pattern to use for calibration.
     * @param samples              samples of the 2D pattern taken with the camera to be
     *                             calibrated.
     * @param samplesQualityScores quality scores for each sample.
     * @return a camera calibrator.
     * @throws IllegalArgumentException if not enough samples are provided.
     */
    public static CameraCalibrator create(
            final Pattern2D pattern, final List<CameraCalibratorSample> samples, final double[] samplesQualityScores) {
        return create(pattern, samples, samplesQualityScores, DEFAULT_METHOD);
    }

    /**
     * Starts the calibration process.
     * Depending on the settings the following will be estimated:
     * intrinsic pinhole camera parameters, radial distortion of lens,
     * camera pose (rotation and translation) for each sample, and the
     * associated homobraphy of sampled points respect to the ideal pattern
     * samples.
     *
     * @throws CalibrationException if calibration fails for some reason.
     * @throws LockedException      if this instance is locked because calibration is
     *                              already in progress.
     * @throws NotReadyException    if this instance does not have enough data to
     *                              start camera calibration.
     */
    public abstract void calibrate() throws CalibrationException, LockedException, NotReadyException;

    /**
     * Returns the camera calibrator method used by this instance.
     *
     * @return the camera calibrator method.
     */
    public abstract CameraCalibratorMethod getMethod();

    /**
     * Notifies progress to current listener, if needed.
     */
    protected abstract void notifyProgress();

    /**
     * Computes intrinsic estimation progress.
     */
    protected void computeIntrinsicProgress() {
        final var lambda = 1.0f / samples.size();
        intrinsicProgress = 0.5f * (sampleProgress + lambda * homographyProgress) + 0.5f * iacProgress;
    }

    /**
     * Resets estimated value to their initial values.
     */
    protected void reset() {
        homographies = null;
        for (final var sample : samples) {
            sample.setUndistortedMarkers(null);
            sample.setHomography(null);
            sample.setRotation(null);
            sample.setCameraCenter(null);
            sample.setCamera(null);
        }
        homographyQualityScores = null;
        iac = null;
        intrinsic = null;
        distortion = null;
    }

    /**
     * Estimates pinhole camera intrinsic parameters without accounting for
     * lens radial distortion.
     *
     * @param idealFallbackPatternMarkers ideal pattern markers coordinates used
     *                                    as fallback if pattern is not provided for a given sample.
     * @throws CalibrationException if calibration fails for some reason.
     */
    protected void estimateIntrinsicParameters(final List<Point2D> idealFallbackPatternMarkers)
            throws CalibrationException {

        homographyProgress = sampleProgress = iacProgress = intrinsicProgress = 0.0f;

        if (listener != null) {
            listener.onIntrinsicParametersEstimationStarts(this);
        }

        // for each sample estimate the homography between the ideal pattern
        // markers and the sampled ones
        List<Point2D> sampledPatternMarkers;
        final var sampleSize = samples.size();
        homographies = new ArrayList<>(sampleSize);
        final var tmpHomographyQualityScores = new double[sampleSize];
        var index = 0;
        var counter = 0;
        double error;
        for (final var sample : samples) {
            try {
                // reset homography before estimation in case that estimation
                // fails on current iteration
                sample.setHomography(null);
                final List<Point2D> idealPatternMarkers;
                if (sample.getPattern() != null) {
                    // use sample pattern for homography estimation
                    idealPatternMarkers = sample.getPattern().getIdealPoints();
                } else {
                    // use fallback pattern common to all samples
                    idealPatternMarkers = idealFallbackPatternMarkers;
                }
                final var homography = sample.estimateHomography(homographyEstimator, idealPatternMarkers);
                sampledPatternMarkers = sample.getUndistortedMarkers() != null
                        ? sample.getUndistortedMarkers() : sample.getSampledMarkers();
                sample.setHomography(homography);
                homographies.add(homography);

                if (iacEstimator.getMethod() == RobustEstimatorMethod.PROSAC
                        || iacEstimator.getMethod() == RobustEstimatorMethod.PROMEDS
                        || homographyQualityScoresRequired) {
                    if (samplesQualityScores != null) {
                        // pick corresponding quality score
                        tmpHomographyQualityScores[counter] = samplesQualityScores[index];
                    } else {
                        // compute re-projection error
                        error = homographyTransformationError(homography, idealPatternMarkers, sampledPatternMarkers);
                        tmpHomographyQualityScores[counter] = 1.0 / (1.0 + error);
                    }
                }
                // counter of homography estimation successes
                counter++;
            } catch (final NumericalException | GeometryException ignore) {
                // homographies are attempted to be estimated. It's ok if some fail
            }

            // position index (regardless of homography estimation success)
            index++;

            homographyProgress = 0.0f;
            sampleProgress = (float) index / (float) sampleSize;
            computeIntrinsicProgress();
            notifyProgress();
        }

        if (iacEstimator.getMethod() == RobustEstimatorMethod.PROSAC
                || iacEstimator.getMethod() == RobustEstimatorMethod.PROMEDS || homographyQualityScoresRequired) {

            // truncate tmpHomographyQualityScores to contain only actual number of
            // successfully estimated homographies
            homographyQualityScores = Arrays.copyOf(tmpHomographyQualityScores, counter);
        }

        // estimate IAC using estimated homographies
        try {
            iacEstimator.setHomographies(homographies);
            iacEstimator.setQualityScores(homographyQualityScores);
            iac = iacEstimator.estimate();

            intrinsic = iac.getIntrinsicParameters();
        } catch (final GeometryException | NumericalException e) {
            throw new CalibrationException(e);
        }

        if (listener != null) {
            listener.onIntrinsicParametersEstimationEnds(this, intrinsic);
        }
    }

    /**
     * Computes average transformation error as a result of comparing sampled
     * pattern markers against the transformation of ideal pattern markers using
     * the estimated homography.
     *
     * @param homography            estimated homography.
     * @param idealPatternMarkers   ideal pattern markers.
     * @param sampledPatternMarkers sampled pattern markers.
     * @return average re-projection error.
     */
    protected static double homographyTransformationError(
            final Transformation2D homography, final List<Point2D> idealPatternMarkers,
            final List<Point2D> sampledPatternMarkers) {

        final var transformedPoint = new HomogeneousPoint2D();
        Point2D idealPatternMarker;
        Point2D sampledPatternMarker;
        var avgError = 0.0;
        double distance;
        final var size = sampledPatternMarkers.size();
        for (var i = 0; i < size; i++) {
            idealPatternMarker = idealPatternMarkers.get(i);
            sampledPatternMarker = sampledPatternMarkers.get(i);

            homography.transform(idealPatternMarker, transformedPoint);
            distance = transformedPoint.distanceTo(sampledPatternMarker);
            avgError += distance;
        }

        avgError /= size;

        return avgError;
    }

    /**
     * Refreshes listener of homography estimator.
     */
    protected void refreshHomographyEstimatorListener() {
        if (homographyEstimatorListener == null) {
            homographyEstimatorListener = new ProjectiveTransformation2DRobustEstimatorListener() {

                @Override
                public void onEstimateStart(final ProjectiveTransformation2DRobustEstimator estimator) {
                    homographyProgress = 0.0f;
                    computeIntrinsicProgress();
                    notifyProgress();
                }

                @Override
                public void onEstimateEnd(final ProjectiveTransformation2DRobustEstimator estimator) {
                    homographyProgress = 1.0f;
                    computeIntrinsicProgress();
                    notifyProgress();
                }

                @Override
                public void onEstimateNextIteration(
                        final ProjectiveTransformation2DRobustEstimator estimator, int iteration) {
                    // not used
                }

                @Override
                public void onEstimateProgressChange(
                        final ProjectiveTransformation2DRobustEstimator estimator, final float progress) {
                    homographyProgress = progress;
                    computeIntrinsicProgress();
                    notifyProgress();
                }
            };
        }

        try {
            homographyEstimator.setListener(homographyEstimatorListener);
        } catch (final LockedException e) {
            Logger.getLogger(CameraCalibrator.class.getName()).log(Level.WARNING,
                    "Could not set homography estimator listener", e);
        }
    }

    /**
     * Refreshes listener of IAC estimator.
     */
    protected void refreshIACEstimatorListener() {
        if (iacEstimatorListener == null) {
            iacEstimatorListener = new ImageOfAbsoluteConicRobustEstimatorListener() {

                @Override
                public void onEstimateStart(final ImageOfAbsoluteConicRobustEstimator estimator) {
                    iacProgress = 0.0f;
                    computeIntrinsicProgress();
                    notifyProgress();
                }

                @Override
                public void onEstimateEnd(final ImageOfAbsoluteConicRobustEstimator estimator) {
                    iacProgress = 1.0f;
                    computeIntrinsicProgress();
                    notifyProgress();
                }

                @Override
                public void onEstimateNextIteration(
                        final ImageOfAbsoluteConicRobustEstimator estimator, final int iteration) {
                    // not used
                }

                @Override
                public void onEstimateProgressChange(
                        final ImageOfAbsoluteConicRobustEstimator estimator, final float progress) {
                    iacProgress = progress;
                    computeIntrinsicProgress();
                    notifyProgress();
                }
            };
        }

        try {
            iacEstimator.setListener(iacEstimatorListener);
        } catch (final LockedException e) {
            Logger.getLogger(CameraCalibrator.class.getName()).log(Level.WARNING,
                    "Could not set IAC estimator listener", e);
        }
    }

    /**
     * Internal method to set list of samples obtained from different pictures
     * using the same camera device (or same camera model). Several samples can
     * be used to calibrate the camera (a pinhole camera can be estimated for
     * each sample). The more samples are used, typically the better the
     * results.
     * This method is for internal use only and does not check whether this
     * instance is locked or not.
     *
     * @param samples list of samples.
     * @throws IllegalArgumentException if not enough samples are provided to
     *                                  estimate the intrinsic parameters. By default, the minimum is 1,
     *                                  but depending on the settings at least 3 samples might be required.
     */
    private void internalSetSamples(final List<CameraCalibratorSample> samples) {
        if (samples.size() < iacEstimator.getMinNumberOfRequiredHomographies()) {
            throw new IllegalArgumentException();
        }

        this.samples = samples;
    }

    /**
     * Sets quality scores assigned to each provided sample. This can be used on
     * certain robust estimation methods of the IAC such as PROSAC and PROMedS.
     * If not provided, homography quality scores will be estimated based on
     * re-projection error and this value will be ignored.
     * This method is for internal use only and does not check whether this
     * instance is locked or not.
     *
     * @param samplesQualityScores quality scores assigned to each provided
     *                             sample.
     * @throws IllegalArgumentException if not enough quality scores are
     *                                  provided for the corresponding samples to estimate the intrinsic
     *                                  parameters. By default, the minimum is 1, but depending on the
     *                                  settings at least 3 samples might be required.
     */
    private void internalSetSamplesQualityScores(final double[] samplesQualityScores) {
        if (samplesQualityScores.length < iacEstimator.getMinNumberOfRequiredHomographies()) {
            throw new IllegalArgumentException();
        }

        this.samplesQualityScores = samplesQualityScores;
    }

    /**
     * Sets robust homography estimation method.
     * If method changes, then a new homography robust estimator is created and
     * configured.
     *
     * @param homographyMethod robust homography estimation method to be set.
     */
    private void internalSetHomographyMethod(final RobustEstimatorMethod homographyMethod) {
        // if method changes, homography estimator must be recreated
        if (homographyMethod != this.homographyMethod) {
            final var previousAvailable = homographyEstimator != null;
            var threshold = 0.0;
            var confidence = 0.0;
            var maxIterations = 0;
            if (previousAvailable) {
                threshold = getHomographyEstimatorThreshold();
                confidence = getHomographyEstimatorConfidence();
                maxIterations = getHomographyEstimatorMaxIterations();
            }

            homographyEstimator = PointCorrespondenceProjectiveTransformation2DRobustEstimator.create(homographyMethod);

            // configure new estimator
            refreshHomographyEstimatorListener();
            if (previousAvailable) {
                try {
                    setHomographyEstimatorThreshold(threshold);
                    setHomographyEstimatorConfidence(confidence);
                    setHomographyEstimatorMaxIterations(maxIterations);
                } catch (final LockedException e) {
                    Logger.getLogger(CameraCalibrator.class.getName()).log(Level.WARNING,
                            "Could not reconfigure homography estimator", e);
                }
            }
        }
        this.homographyMethod = homographyMethod;
    }

    /**
     * Sets robust IAC estimation method.
     * IF method changes, then a new IAC robust estimator is created and
     * configured
     *
     * @param imageOfAbsoluteConicMethod robust IAC estimation method to be set
     */
    private void internalSetImageOfAbsoluteConicMethod(final RobustEstimatorMethod imageOfAbsoluteConicMethod) {
        // if method changes, iac estimator must be recreated
        if (imageOfAbsoluteConicMethod != this.imageOfAbsoluteConicMethod) {
            final var previousAvailable = iacEstimator != null;
            var threshold = 0.0;
            var confidence = 0.0;
            var maxIterations = 0;
            var zeroSkewness = false;
            var principalPointAtOrigin = false;
            var focalDistanceAspectRatioKnown = false;
            var focalDistanceAspectRatio = 0.0;
            if (previousAvailable) {
                threshold = getIACEstimatorThreshold();
                confidence = getIACEstimatorConfidence();
                maxIterations = getIACEstimatorMaxIterations();
                zeroSkewness = isZeroSkewness();
                principalPointAtOrigin = isPrincipalPointAtOrigin();
                focalDistanceAspectRatioKnown = isFocalDistanceAspectRatioKnown();
                focalDistanceAspectRatio = getFocalDistanceAspectRatio();
            }

            iacEstimator = ImageOfAbsoluteConicRobustEstimator.create(imageOfAbsoluteConicMethod);

            // configure new estimator
            refreshIACEstimatorListener();
            if (previousAvailable) {
                try {
                    setIACEstimatorThreshold(threshold);
                    setIACEstimatorConfidence(confidence);
                    setIACEstimatorMaxIterations(maxIterations);
                    setZeroSkewness(zeroSkewness);
                    setPrincipalPointAtOrigin(principalPointAtOrigin);
                    setFocalDistanceAspectRatioKnown(focalDistanceAspectRatioKnown);
                    setFocalDistanceAspectRatio(focalDistanceAspectRatio);
                } catch (final LockedException e) {
                    Logger.getLogger(CameraCalibrator.class.getName()).log(Level.WARNING,
                            "Could not reconfigure IAC estimator", e);
                }
            }
        }
        this.imageOfAbsoluteConicMethod = imageOfAbsoluteConicMethod;
    }
}
