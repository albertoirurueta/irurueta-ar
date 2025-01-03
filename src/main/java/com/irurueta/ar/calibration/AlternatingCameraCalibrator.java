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

import com.irurueta.ar.calibration.estimators.LMedSRadialDistortionRobustEstimator;
import com.irurueta.ar.calibration.estimators.MSACRadialDistortionRobustEstimator;
import com.irurueta.ar.calibration.estimators.PROMedSRadialDistortionRobustEstimator;
import com.irurueta.ar.calibration.estimators.PROSACRadialDistortionRobustEstimator;
import com.irurueta.ar.calibration.estimators.RANSACRadialDistortionRobustEstimator;
import com.irurueta.ar.calibration.estimators.RadialDistortionRobustEstimator;
import com.irurueta.ar.calibration.estimators.RadialDistortionRobustEstimatorListener;
import com.irurueta.geometry.GeometryException;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Calibrates a camera in order to find its intrinsic parameters and radial
 * distortion by using an alternating technique where first an initial guess
 * of the intrinsic parameters, rotation and translation is obtained to model
 * the camera used to sample the calibration pattern, and then the result is
 * used to find the best possible radial distortion to account for all remaining
 * errors. The result is then used to undo the distortion effect and calibrate
 * again to estimate the intrinsic parameters and camera pose. This alternating
 * process is repeated until convergence is reached.
 * <p>
 * This class is based on technique described at:
 * Zhengyou Zhang. A Flexible New Technique for Camera Calibration. Technical
 * Report. MSR-TR-98-71. December 2, 1998
 */
@SuppressWarnings("DuplicatedCode")
public class AlternatingCameraCalibrator extends CameraCalibrator {

    /**
     * Default maximum number of times to do an alternating iteration to refine
     * the results.
     */
    public static final int DEFAULT_MAX_ITERATIONS = 20;

    /**
     * Minimum allowed value to be set as max iterations.
     */
    public static final int MIN_MAX_ITERATIONS = 1;

    /**
     * Default threshold to determine that convergence of the result has been
     * reached.
     */
    public static final double DEFAULT_CONVERGENCE_THRESHOLD = 1e-8;

    /**
     * Minimum allowed value to be set as convergence threshold.
     */
    public static final double MIN_CONVERGENCE_THRESHOLD = 0.0;

    /**
     * Default robust estimator method to be used for radial distortion
     * estimation.
     */
    public static final RobustEstimatorMethod DEFAULT_RADIAL_DISTORTION_METHOD = RobustEstimatorMethod.PROSAC;

    /**
     * Maximum number of times to do an alternating iteration to refine the
     * results.
     */
    private int maxIterations;

    /**
     * Default threshold to determine that convergence of the result has been
     * reached.
     */
    private double convergenceThreshold;

    /**
     * Robust estimator method to be used for radial distortion estimation.
     */
    private RobustEstimatorMethod distortionMethod;

    /**
     * Robust estimator of radial distortion.
     */
    private RadialDistortionRobustEstimator distortionEstimator;

    /**
     * Listener for robust estimator of radial distortion.
     */
    private RadialDistortionRobustEstimatorListener distortionEstimatorListener;

    /**
     * Indicates progress of radial distortion estimation.
     */
    private float radialDistortionProgress;

    /**
     * Overall progress taking into account current number of iteration.
     */
    private float iterProgress;

    /**
     * Previously notified progress.
     */
    private float previousNotifiedProgress;

    /**
     * Constructor.
     */
    public AlternatingCameraCalibrator() {
        super();
        maxIterations = DEFAULT_MAX_ITERATIONS;
        convergenceThreshold = DEFAULT_CONVERGENCE_THRESHOLD;

        internalSetDistortionMethod(DEFAULT_RADIAL_DISTORTION_METHOD);
    }

    /**
     * Constructor.
     *
     * @param pattern 2D pattern to use for calibration.
     * @param samples samples of the pattern taken with the camera to calibrate.
     * @throws IllegalArgumentException if not enough samples are provided.
     */
    public AlternatingCameraCalibrator(final Pattern2D pattern, final List<CameraCalibratorSample> samples) {
        super(pattern, samples);
        maxIterations = DEFAULT_MAX_ITERATIONS;
        convergenceThreshold = DEFAULT_CONVERGENCE_THRESHOLD;

        internalSetDistortionMethod(DEFAULT_RADIAL_DISTORTION_METHOD);
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
    public AlternatingCameraCalibrator(
            final Pattern2D pattern, final List<CameraCalibratorSample> samples, final double[] samplesQualityScores) {
        super(pattern, samples, samplesQualityScores);
        maxIterations = DEFAULT_MAX_ITERATIONS;
        convergenceThreshold = DEFAULT_CONVERGENCE_THRESHOLD;

        internalSetDistortionMethod(DEFAULT_RADIAL_DISTORTION_METHOD);
    }

    /**
     * Returns maximum number of times to do an alternating iteration to refine
     * the results.
     *
     * @return maximum number of times to do an alternating iteration.
     */
    public int getMaxIterations() {
        return maxIterations;
    }

    /**
     * Sets maximum number of times to do an alternating iteration to refine the
     * results.
     *
     * @param maxIterations maximum number of times to do an alternating
     *                      iteration.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setMaxIterations(final int maxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxIterations < MIN_MAX_ITERATIONS) {
            throw new IllegalArgumentException();
        }

        this.maxIterations = maxIterations;
    }

    /**
     * Returns threshold to determine that convergence of the result has been
     * reached.
     *
     * @return threshold to determine that convergence of the result has been
     * reached.
     */
    public double getConvergenceThreshold() {
        return convergenceThreshold;
    }

    /**
     * Sets threshold to determine that convergence of the result has been
     * reached.
     *
     * @param convergenceThreshold threshold to determine that convergence of
     *                             the result has been reached.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setConvergenceThreshold(final double convergenceThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (convergenceThreshold < MIN_CONVERGENCE_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        this.convergenceThreshold = convergenceThreshold;
    }

    /**
     * Returns robust estimator method to be used for radial distortion
     * estimation.
     *
     * @return robust estimator method to be used for radial distortion
     * estimation.
     */
    public RobustEstimatorMethod getDistortionMethod() {
        return distortionMethod;
    }

    /**
     * Sets robust estimator method to be used for radial distortion
     * estimation.
     *
     * @param distortionMethod robust estimator method to be used for
     *                         radial distortion estimation.
     * @throws LockedException if this instance is locked.
     */
    public void setDistortionMethod(final RobustEstimatorMethod distortionMethod) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetDistortionMethod(distortionMethod);
    }

    /**
     * Returns radial distortion estimator, which can be retrieved in case
     * that some additional parameter needed to be adjusted.
     * It is discouraged to directly access the distortion estimator during
     * camera calibration, as it might interfere with the results.
     *
     * @return radial distortion estimator.
     */
    public RadialDistortionRobustEstimator getDistortionEstimator() {
        return distortionEstimator;
    }

    /**
     * Returns threshold to robustly estimate radial distortion.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @return threshold to robustly estimate radial distortion.
     */
    public double getDistortionEstimatorThreshold() {
        return switch (distortionEstimator.getMethod()) {
            case LMEDS -> ((LMedSRadialDistortionRobustEstimator) distortionEstimator).getStopThreshold();
            case MSAC -> ((MSACRadialDistortionRobustEstimator) distortionEstimator).getThreshold();
            case PROSAC -> ((PROSACRadialDistortionRobustEstimator) distortionEstimator).getThreshold();
            case PROMEDS -> ((PROMedSRadialDistortionRobustEstimator) distortionEstimator).getStopThreshold();
            default -> ((RANSACRadialDistortionRobustEstimator) distortionEstimator).getThreshold();
        };
    }

    /**
     * Sets threshold to robustly estimate radial distortion.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finder adjustments.
     *
     * @param distortionEstimatorThreshold threshold to robustly estimate
     *                                     radial distortion .
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setDistortionEstimatorThreshold(final double distortionEstimatorThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        switch (distortionEstimator.getMethod()) {
            case LMEDS:
                ((LMedSRadialDistortionRobustEstimator) distortionEstimator)
                        .setStopThreshold(distortionEstimatorThreshold);
                break;
            case MSAC:
                ((MSACRadialDistortionRobustEstimator) distortionEstimator).setThreshold(distortionEstimatorThreshold);
                break;
            case PROSAC:
                ((PROSACRadialDistortionRobustEstimator) distortionEstimator)
                        .setThreshold(distortionEstimatorThreshold);
                break;
            case PROMEDS:
                ((PROMedSRadialDistortionRobustEstimator) distortionEstimator)
                        .setStopThreshold(distortionEstimatorThreshold);
                break;
            case RANSAC:
            default:
                ((RANSACRadialDistortionRobustEstimator) distortionEstimator)
                        .setThreshold(distortionEstimatorThreshold);
                break;
        }
    }

    /**
     * Returns confidence to robustly estimate radial distortion.
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
    public double getDistortionEstimatorConfidence() {
        return distortionEstimator.getConfidence();
    }

    /**
     * Sets confidence to robustly estimate radial distortion.
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
     * @param distortionEstimatorConfidence confidence to robustly estimate
     *                                      radial distortion.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is not between 0.0 and
     *                                  1.0.
     */
    public void setDistortionEstimatorConfidence(final double distortionEstimatorConfidence) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        distortionEstimator.setConfidence(distortionEstimatorConfidence);
    }

    /**
     * Returns the maximum number of iterations to be done when estimating
     * the radial distortion.
     * If the maximum allowed number of iterations is reached, resulting
     * estimation might not have desired confidence.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @return maximum number of iterations to be done when estimating the
     * homographies.
     */
    public int getDistortionEstimatorMaxIterations() {
        return distortionEstimator.getMaxIterations();
    }

    /**
     * Sets the maximum number of iterations to be done when estimating the
     * radial distortion.
     * If the maximum allowed number of iterations is reached, resulting
     * estimation might not have desired confidence.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     *
     * @param distortionEstimatorMaxIterations maximum number of iterations to
     *                                         be done when estimating radial distortion.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is negative or zero.
     */
    public void setDistortionEstimatorMaxIterations(final int distortionEstimatorMaxIterations) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        distortionEstimator.setMaxIterations(distortionEstimatorMaxIterations);
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
    @Override
    public void calibrate() throws CalibrationException, LockedException, NotReadyException {

        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        locked = true;

        homographyQualityScoresRequired = (distortionEstimator.getMethod() == RobustEstimatorMethod.PROSAC
                || distortionEstimator.getMethod() == RobustEstimatorMethod.PROMEDS);

        if (listener != null) {
            listener.onCalibrateStart(this);
        }

        reset();
        radialDistortionProgress = iterProgress = previousNotifiedProgress = 0.0f;

        final var idealFallbackPatternMarkers = pattern.getIdealPoints();

        try {
            double errorDiff;
            double previousError;
            var currentError = Double.MAX_VALUE;
            var bestError = Double.MAX_VALUE;
            PinholeCameraIntrinsicParameters bestIntrinsic = null;
            RadialDistortion bestDistortion = null;

            // iterate until error converges
            var iter = 0;
            do {
                previousError = currentError;

                // estimate intrinsic parameters
                estimateIntrinsicParameters(idealFallbackPatternMarkers);

                if (!estimateRadialDistortion) {
                    break;
                }

                // estimate radial distortion using estimated intrinsic
                // parameters and camera poses and obtain average re-projection
                // error
                currentError = estimateRadialDistortion(idealFallbackPatternMarkers);

                if (currentError < bestError) {
                    bestError = currentError;
                    bestIntrinsic = intrinsic;
                    bestDistortion = distortion;
                }

                errorDiff = Math.abs(previousError - currentError);
                iter++;
                iterProgress = (float) iter / (float) maxIterations;
                notifyProgress();

            } while (errorDiff > convergenceThreshold && iter < maxIterations);

            if (bestIntrinsic != null) {
                intrinsic = bestIntrinsic;
            }
            if (bestDistortion != null) {
                distortion = bestDistortion;
            }

            if (listener != null) {
                listener.onCalibrateEnd(this);
            }
        } finally {
            locked = false;
        }
    }

    /**
     * Estimates radial distortion using estimated intrinsic parameters among
     * all samples to estimate their camera poses to find non-distorted points
     * and compare them with the sampled ones.
     *
     * @param idealFallbackPatternMarkers ideal pattern markers coordinates
     *                                    These coordinates are used as fallback when a given sample does
     *                                    not have an associated pattern.
     * @return average re-projection error, obtained after projecting ideal
     * pattern markers using estimated camera poses and then doing a comparison
     * with sampled points taking into account estimated distortion to undo
     * their corresponding distortion.
     * @throws CalibrationException if anything fails.
     */
    protected double estimateRadialDistortion(final List<Point2D> idealFallbackPatternMarkers)
            throws CalibrationException {

        radialDistortionProgress = 0.0f;

        if (listener != null) {
            listener.onRadialDistortionEstimationStarts(this);
        }

        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();
        // compute total points for samples where homography could be estimated
        var totalPoints = 0;
        for (final var sample : samples) {
            if (sample.getHomography() != null) {
                totalPoints += sample.getSampledMarkers().size();
            }
        }

        double[] qualityScores = null;
        if (distortionMethod == RobustEstimatorMethod.PROSAC || distortionMethod == RobustEstimatorMethod.PROMEDS) {
            qualityScores = new double[totalPoints];
        }

        // estimate camera pose for each sample
        var pointCounter = 0;
        var sampleCounter = 0;
        for (final var sample : samples) {
            if (sample.getHomography() == null) {
                // homography computation failed, so we cannot compute camera
                // pose for this sample, or use this sample for radial distortion
                // estimation
                continue;
            }
            sample.computeCameraPose(intrinsic);

            // transform ideal pattern markers using estimated homography
            final List<Point2D> idealPatternMarkers;
            if (sample.getPattern() != null) {
                // use points generated by pattern in sample
                idealPatternMarkers = sample.getPattern().getIdealPoints();
            } else {
                // use fallback pattern points
                idealPatternMarkers = idealFallbackPatternMarkers;
            }

            final var transformedIdealPatternMarkers = sample.getHomography()
                    .transformPointsAndReturnNew(idealPatternMarkers);

            // transformedIdealPatternMarkers are considered the undistorted
            // points, because camera follows a pure pinhole model without
            // distortion, and we have transformed the ideal points using a
            // pure projective homography without distortion.
            // sample.getSampledMarkers() contains the sampled coordinates using
            // the actual camera, which will be distorted

            // the sampled markers are the ones considered to be distorted for
            // radial distortion estimation purposes, because they are obtained
            // directly from the camera

            // stack together all distorted and undistorted points from all
            // samples

            distortedPoints.addAll(sample.getSampledMarkers());
            undistortedPoints.addAll(transformedIdealPatternMarkers);

            final var markersSize = transformedIdealPatternMarkers.size();

            // if distortion estimator requires quality scores, set them
            if (qualityScores != null && (distortionMethod == RobustEstimatorMethod.PROSAC
                    || distortionMethod == RobustEstimatorMethod.PROMEDS)) {

                final var sampleQuality = homographyQualityScores[sampleCounter];

                // assign to all points (markers) in the sample the same sample
                // quality
                for (var i = pointCounter; i < pointCounter + markersSize; i++) {
                    qualityScores[i] = sampleQuality;
                }

                pointCounter += markersSize;
                sampleCounter++;
            }
        }

        // estimate radial distortion
        var avgError = 0.0;
        try {
            distortionEstimator.setIntrinsic(intrinsic);
            distortionEstimator.setPoints(distortedPoints, undistortedPoints);
            distortionEstimator.setQualityScores(qualityScores);

            final var distortion = distortionEstimator.estimate();

            // add distortion to undistorted points (which are ideal pattern
            // markers with homography applied)
            final var distortedPoints2 = distortion.distort(undistortedPoints);

            // set undistorted points obtained after un-distorting sampled points
            // to refine homography on next iteration
            for (final var sample : samples) {
                if (sample.getHomography() == null) {
                    continue;
                }

                // undo distortion of distorted (sampled) points using estimated
                // distortion

                final var undistortedPoints2 = distortion.undistort(sample.getSampledMarkers());

                sample.setUndistortedMarkers(undistortedPoints2);
            }

            // compare distortedPoints (obtained by using sampled data)
            // with distortedPoints2 (obtained after applying homography to
            // ideal marker points and applying distortion with estimated
            // distortion)
            Point2D distortedPoint1;
            Point2D distortedPoint2;
            totalPoints = distortedPoints.size();
            var inlierCount = 0;
            for (var i = 0; i < totalPoints; i++) {
                distortedPoint1 = distortedPoints.get(i);
                distortedPoint2 = distortedPoints2.get(i);

                final var distance = distortedPoint1.distanceTo(distortedPoint2);
                if (distance < getDistortionEstimatorThreshold()) {
                    avgError += distance;
                    inlierCount++;
                }
            }

            if (inlierCount == 0) {
                throw new CalibrationException();
            }

            avgError /= inlierCount;

            this.distortion = distortion;

        } catch (final GeometryException | NumericalException | DistortionException e) {
            throw new CalibrationException(e);
        }

        if (listener != null) {
            listener.onRadialDistortionEstimationEnds(this, distortion);
        }

        return avgError;
    }

    /**
     * Returns the camera calibrator method used by this instance.
     *
     * @return the camera calibrator method.
     */
    @Override
    public CameraCalibratorMethod getMethod() {
        return CameraCalibratorMethod.ALTERNATING_CALIBRATOR;
    }

    /**
     * Notifies progress to current listener, if needed.
     */
    @Override
    protected void notifyProgress() {
        final var lambda = 1.0f / maxIterations;
        final var partial = 0.5f * intrinsicProgress + 0.5f * radialDistortionProgress;

        final float progress;
        if (!estimateRadialDistortion) {
            // we do not iterate if there is no need to
            // estimate radial distortion
            progress = partial;
        } else {
            progress = iterProgress + lambda * partial;
        }

        if (listener != null && (progress - previousNotifiedProgress) > progressDelta) {
            listener.onCalibrateProgressChange(this, progress);
            previousNotifiedProgress = progress;
        }
    }

    /**
     * Refreshes listener of distortion estimator
     */
    protected void refreshDistortionEstimatorListener() {
        if (distortionEstimatorListener == null) {
            distortionEstimatorListener = new RadialDistortionRobustEstimatorListener() {

                @Override
                public void onEstimateStart(final RadialDistortionRobustEstimator estimator) {
                    radialDistortionProgress = 0.0f;
                    notifyProgress();
                }

                @Override
                public void onEstimateEnd(final RadialDistortionRobustEstimator estimator) {
                    radialDistortionProgress = 1.0f;
                    notifyProgress();
                }

                @Override
                public void onEstimateNextIteration(
                        final RadialDistortionRobustEstimator estimator, final int iteration) {
                    // not needed
                }

                @Override
                public void onEstimateProgressChange(
                        final RadialDistortionRobustEstimator estimator, final float progress) {
                    radialDistortionProgress = progress;
                    notifyProgress();
                }
            };
        }

        try {
            distortionEstimator.setListener(distortionEstimatorListener);
        } catch (final LockedException e) {
            Logger.getLogger(AlternatingCameraCalibrator.class.getName()).log(Level.WARNING,
                    "Could not set radial distortion estimator listener", e);
        }
    }

    /**
     * Sets robust estimator method to be used for radial distortion estimation.
     * If method changes, then a new radial distortion estimator is created and
     * configured.
     *
     * @param distortionMethod robust estimator method to be used for
     *                         radial distortion estimation.
     */
    private void internalSetDistortionMethod(final RobustEstimatorMethod distortionMethod) {
        // if method changes, recreate estimator
        if (distortionMethod != this.distortionMethod) {
            var previousAvailable = this.distortionMethod != null;
            var threshold = 0.0;
            var confidence = 0.0;
            var maxIters = 0;
            if (previousAvailable) {
                threshold = getDistortionEstimatorThreshold();
                confidence = getDistortionEstimatorConfidence();
                maxIters = getDistortionEstimatorMaxIterations();
            }

            distortionEstimator = RadialDistortionRobustEstimator.create(distortionMethod);

            // configure new estimator
            refreshDistortionEstimatorListener();
            if (previousAvailable) {
                try {
                    setDistortionEstimatorThreshold(threshold);
                    setDistortionEstimatorConfidence(confidence);
                    setDistortionEstimatorMaxIterations(maxIters);
                } catch (final LockedException e) {
                    Logger.getLogger(AlternatingCameraCalibrator.class.getName()).log(Level.WARNING,
                            "Could not reconfigure distortion estimator", e);
                }
            }
        }

        this.distortionMethod = distortionMethod;
    }
}
