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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.ar.calibration.estimators.CameraPoseEstimator;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.geometry.estimators.PointCorrespondenceProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Contains data obtained from a single picture using the camera.
 * Several samples can be used to calibrate the camera. The more samples are
 * used, typically the better the results.
 */
public class CameraCalibratorSample {
    /**
     * Minimum number of sampled markers that must be provided to estimate
     * an homography.
     */
    public static final int MIN_REQUIRED_SAMPLED_MARKERS = 4;

    /**
     * Pattern used for camera calibration. Each pattern contains a unique
     * combination of 2D points that must be sampled using the camera to be
     * calibrated.
     */
    private Pattern2D pattern;

    /**
     * Contains the sampled markers taken from a single picture using the
     * camera.
     */
    private List<Point2D> sampledMarkers;

    /**
     * Contains the sampled markers of the pattern but accounting for the
     * distortion effect introduced by the camera lens.
     */
    private List<Point2D> undistortedMarkers;

    /**
     * Quality scores of sampled markers. These can be used during
     * homography estimation if a robust estimation method such as PROSAC or
     * PROMedS is used.
     */
    private double[] sampledMarkersQualityScores;

    /**
     * 2D homography estimated from the sampled pattern points respect to
     * the ideal ones using a single picture.
     */
    private Transformation2D homography;

    /**
     * Estimated camera rotation. This contains the amount of rotation
     * respect to the plane formed by the pattern markers. This is obtained
     * once the IAC of the camera is estimated.
     */
    private Rotation3D rotation;

    /**
     * Estimated camera center. This determines the amount of translation
     * of the camera respect to the plane formed by the pattern markers. This
     * is obtained once the IAC of the camera is estimated.
     */
    private Point3D cameraCenter;

    /**
     * Estimated camera. Estimated pinhole camera taking into account
     * estimated intrinsic parameters and amount of rotation and translation
     * respect to the plane formed by the pattern markers, but without
     * taking into account any radial distortion introduced by the lens.
     */
    private PinholeCamera camera;

    /**
     * Constructor.
     */
    public CameraCalibratorSample() {
    }

    /**
     * Constructor.
     *
     * @param sampledMarkers sampled markers of the pattern taken from a
     *                       single picture using the camera.
     * @throws IllegalArgumentException if provided number of sampled
     *                                  markers is smaller than the required minimum (4) to estimate an
     *                                  homography.
     */
    public CameraCalibratorSample(final List<Point2D> sampledMarkers) {
        setSampledMarkers(sampledMarkers);
    }

    /**
     * Constructor.
     *
     * @param sampledMarkers              sampled markers of the pattern taken from a
     *                                    single picture using the camera.
     * @param sampledMarkersQualityScores quality scores associated to
     *                                    each provided point for each sampled marker. The higher the value
     *                                    the better the quality assigned to that point.
     * @throws IllegalArgumentException if size of sampled markers or
     *                                  quality scores is smaller than the required minimum (4) to estimate
     *                                  an homography, or if their sizes do not match.
     */
    public CameraCalibratorSample(final List<Point2D> sampledMarkers, final double[] sampledMarkersQualityScores) {
        if (sampledMarkers.size() != sampledMarkersQualityScores.length) {
            throw new IllegalArgumentException();
        }

        setSampledMarkers(sampledMarkers);
        setSampledMarkersQualityScores(sampledMarkersQualityScores);
    }

    /**
     * Constructor.
     *
     * @param pattern        2D pattern to use for calibration.
     * @param sampledMarkers sampled markers of the pattern taken from a
     *                       single picture using the camera.
     * @throws IllegalArgumentException if provided number of sampled
     *                                  markers is smaller than the required minimum (4) to estimate an
     *                                  homography.
     */
    public CameraCalibratorSample(final Pattern2D pattern, final List<Point2D> sampledMarkers) {
        this.pattern = pattern;
        setSampledMarkers(sampledMarkers);
    }

    /**
     * Constructor.
     *
     * @param pattern                     2D pattern to use for calibration.
     * @param sampledMarkers              sampled markers of the pattern taken from a
     *                                    single picture using the camera.
     * @param sampledMarkersQualityScores quality scores associated to
     *                                    each provided point for each sampled marker. The higher the value
     *                                    the better the quality assigned to that point.
     * @throws IllegalArgumentException if size of sampled markers or
     *                                  quality scores is smaller than the required minimum (4) to estimate
     *                                  an homography, or if their sizes do not match.
     */
    public CameraCalibratorSample(final Pattern2D pattern, final List<Point2D> sampledMarkers,
                                  final double[] sampledMarkersQualityScores) {
        if (sampledMarkers.size() != sampledMarkersQualityScores.length) {
            throw new IllegalArgumentException();
        }

        this.pattern = pattern;
        setSampledMarkers(sampledMarkers);
        setSampledMarkersQualityScores(sampledMarkersQualityScores);
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
     */
    public void setPattern(final Pattern2D pattern) {
        this.pattern = pattern;
    }

    /**
     * Obtains sampled markers of the pattern taken from a single picture
     * using the camera.
     *
     * @return sampled markers of the pattern.
     */
    public List<Point2D> getSampledMarkers() {
        return sampledMarkers;
    }

    /**
     * Sets sampled markers of the pattern taken from a single picture
     * using the camera.
     *
     * @param sampledMarkers sampled markers of the pattern.
     * @throws IllegalArgumentException if provided number of sampled
     *                                  markers is smaller than the required minimum (4) to estimate an
     *                                  homography.
     */
    public final void setSampledMarkers(final List<Point2D> sampledMarkers) {
        if (sampledMarkers.size() < MIN_REQUIRED_SAMPLED_MARKERS) {
            throw new IllegalArgumentException();
        }

        this.sampledMarkers = sampledMarkers;
    }

    /**
     * Returns quality scores of sampled markers. The higher the quality
     * score value the better the quality assigned to the associated 2D
     * point of a sampled marker.
     * Quality scores are only used if a robust estimation method such as
     * PROSAC or PROMedS is used for homography estimation
     *
     * @return quality scores of sampled markers.
     */
    public double[] getSampledMarkersQualityScores() {
        return sampledMarkersQualityScores;
    }

    /**
     * Sets quality scores of sampled markers. The higher the quality score
     * value the better the quality assigned to the associated 2D point of
     * a sampled marker.
     * Quality scores are only used if a robust estimation method such as
     * PROSAC or PROMedS is used for homography estimation.
     *
     * @param sampledMarkersQualityScores quality scores of sampled markers.
     * @throws IllegalArgumentException if provided number of quality scores
     *                                  is smaller than the required minimum (4) to estimate an homography.
     */
    public final void setSampledMarkersQualityScores(final double[] sampledMarkersQualityScores) {
        if (sampledMarkersQualityScores.length < MIN_REQUIRED_SAMPLED_MARKERS) {
            throw new IllegalArgumentException();
        }

        this.sampledMarkersQualityScores = sampledMarkersQualityScores;
    }

    /**
     * Computes quality scores of sampled markers by taking into account
     * distance to radial distortion center.
     * Typically, the farther a sample is to the radial distortion, the more
     * likely it is to be distorted, and hence, the less reliable will be.
     *
     * @param sampledMarkers         sampled markers of the pattern.
     * @param radialDistortionCenter location where radial distortion center
     *                               is assumed to be. If null, it is assumed that center is at origin
     *                               of coordinates (i.e. center of image if principal point is also at
     *                               center of image).
     * @return quality scores of sampled markers.
     */
    public static double[] computeSampledMarkersQualityScores(
            final List<Point2D> sampledMarkers, final Point2D radialDistortionCenter) {

        final Point2D center = radialDistortionCenter != null
                ? radialDistortionCenter : new InhomogeneousPoint2D(0.0, 0.0);

        final var qualityScores = new double[sampledMarkers.size()];

        var counter = 0;
        double distance;
        double qualityScore;
        for (final var sampledMarker : sampledMarkers) {
            distance = sampledMarker.distanceTo(center);
            qualityScore = 1.0 / (1.0 + distance);

            qualityScores[counter] = qualityScore;
            counter++;
        }

        return qualityScores;
    }

    /**
     * Computes quality scores of sampled markers by taking into account
     * distance to radial distortion center, which is assumed to be at
     * origin of coordinates (i.e. center of image if principal point is
     * also at center of image).
     *
     * @param sampledMarkers sampled markers of the pattern.
     * @return quality scores of sampled markers.
     */
    public static double[] computeSampledMarkersQualityScores(final List<Point2D> sampledMarkers) {
        return computeSampledMarkersQualityScores(sampledMarkers, null);
    }

    /**
     * Contains the sampled markers of the pattern but accounting for the
     * distortion effect introduced by the camera lens, so that coordinates
     * are undistorted and follow a pure pinhole camera model.
     * Coordinates of undistorted markers might change during camera
     * calibration while the radial distortion parameters are refined.
     *
     * @return sampled markers of the pattern accounting for lens radial
     * distortion.
     */
    protected List<Point2D> getUndistortedMarkers() {
        return undistortedMarkers;
    }

    /**
     * Sets sampled markers of the pattern but accounting for the distortion
     * effect introduced by the camera lens, so that coordinates are
     * undistorted and follow a pure pinhole camera model.
     * This method is for internal purposes only, and it is called while
     * the camera radial distortion parameters are being computed.
     *
     * @param undistortedMarkers sampled markers of the pattern accounting
     *                           for lens radial distortion.
     */
    protected void setUndistortedMarkers(final List<Point2D> undistortedMarkers) {
        this.undistortedMarkers = undistortedMarkers;
    }

    /**
     * Returns 2D homography estimated from the sampled pattern points
     * respect to the ideal ones using a single picture.
     *
     * @return homography of the sampled pattern points respect to the ideal
     * ones.
     */
    public Transformation2D getHomography() {
        return homography;
    }

    /**
     * Sets 2D homography estimated from the sampled pattern points
     * respect to the ideal ones using a single picture.
     * This method is for internal purposes only, and it is called while
     * the IAC is being estimated.
     *
     * @param homography homography to be set.
     */
    protected void setHomography(final Transformation2D homography) {
        this.homography = homography;
    }

    /**
     * Returns estimated camera rotation for this sample. This contains
     * the amount of rotation respect to the plane formed by the pattern
     * markers for the picture associated to this sample. This is obtained
     * once the IAC of the camera is estimated.
     *
     * @return estimated camera rotation for this sample.
     */
    public Rotation3D getRotation() {
        return rotation;
    }

    /**
     * Sets estimated camera rotation for this sample. This contains
     * the amount of rotation respect to the plane formed by the pattern
     * markers for the picture associated to this sample. This is obtained
     * once the IAC of the camera is estimated.
     * This method is for internal purposes only and might only be called
     * if camera rotation is required during radial distortion estimation,
     * or if rotation is requested for some other purpose.
     *
     * @param rotation camera rotation for this sample.
     */
    protected void setRotation(final Rotation3D rotation) {
        this.rotation = rotation;
    }

    /**
     * Returns estimated camera center. This determines the amount of
     * translation of the camera respect to the plane formed by the pattern
     * markers. This is obtained once the IAC of the camera is estimated.
     *
     * @return estimated camera center.
     */
    public Point3D getCameraCenter() {
        return cameraCenter;
    }

    /**
     * Sets estimated camera center. This determines the amount of translation
     * of the camera respect to the plane formed by the pattern markers. This is
     * obtained once the IAC of the camera is estimated.
     *
     * @param cameraCenter estimated camera center.
     */
    protected void setCameraCenter(final Point3D cameraCenter) {
        this.cameraCenter = cameraCenter;
    }

    /**
     * Returns estimated camera. Estimated pinhole camera taking into
     * account estimated intrinsic parameters and amount of rotation and
     * translation respect to the plane formed by the pattern markers, but
     * without taking into account any radial distortion introduced by the
     * lens.
     *
     * @return estimated camera.
     */
    public PinholeCamera getCamera() {
        return camera;
    }

    /**
     * Sets estimated camera taking into account estimated intrinsic
     * parameters, amount of rotation and translation respect to the plane
     * formed by the pattern markers, but without taking into account any
     * radial distortion introduced by the lens.
     * This method is for internal purposes only and might only be called if
     * camera is required during radial distortion estimation, or if camera
     * is requested for some other purpose.
     *
     * @param camera estimated camera.
     */
    protected void setCamera(final PinholeCamera camera) {
        this.camera = camera;
    }

    /**
     * Estimates homography of sampled points respect to the ideal pattern
     * points. Undistorted sampled taking into account radial distortion
     * will be taken into account whenever possible.
     *
     * @param estimator           a robust estimator for the homography. It will only
     *                            be used if more than 4 markers are provided.
     * @param idealPatternMarkers ideal marker coordinates of the pattern.
     *                            This contains measures expressed in meters so that camera can be
     *                            calibrated against real measures.
     * @return an homography.
     * @throws LockedException           if robust estimator is locked because
     *                                   computations are already in progress.
     * @throws NotReadyException         if provided data to compute homography is
     *                                   not enough, or it is invalid.
     * @throws RobustEstimatorException  if robust estimation of homography
     *                                   failed. This typically happens when not enough inliers are found or
     *                                   configuration of points to estimate homography is degenerate.
     * @throws CoincidentPointsException if configuration of points to
     *                                   estimate homography is degenerate.
     */
    protected Transformation2D estimateHomography(
            final PointCorrespondenceProjectiveTransformation2DRobustEstimator estimator,
            final List<Point2D> idealPatternMarkers) throws LockedException, NotReadyException,
            RobustEstimatorException, CoincidentPointsException {

        final var markers = undistortedMarkers != null ? undistortedMarkers : sampledMarkers;

        if (markers.size() < MIN_REQUIRED_SAMPLED_MARKERS) {
            throw new NotReadyException();
        }
        if (markers.size() != idealPatternMarkers.size()) {
            throw new NotReadyException();
        }

        if (markers.size() == MIN_REQUIRED_SAMPLED_MARKERS) {
            // use non-robust projective transformation estimation since it
            // is faster and will produce the same result as a robust
            // estimator
            return new ProjectiveTransformation2D(idealPatternMarkers.get(0), idealPatternMarkers.get(1),
                    idealPatternMarkers.get(2), idealPatternMarkers.get(3), markers.get(0), markers.get(1),
                    markers.get(2), markers.get(3));
        } else {
            // use robust projective transformation estimation
            estimator.setPoints(idealPatternMarkers, markers);
            if (estimator.getMethod() == RobustEstimatorMethod.PROSAC
                    || estimator.getMethod() == RobustEstimatorMethod.PROMEDS) {
                if (sampledMarkersQualityScores == null) {
                    // attempt to estimate quality scores based on distance of
                    // samples to origin of coordinates (i.e. image center)
                    sampledMarkersQualityScores = computeSampledMarkersQualityScores(markers);
                }
                estimator.setQualityScores(sampledMarkersQualityScores);
            }

            return estimator.estimate();
        }
    }

    /**
     * Computes camera pose using estimated homography and provided intrinsic
     * pinhole camera parameters that have been estimated so far.
     *
     * @param intrinsic intrinsic pinhole camera parameters.
     * @throws CalibrationException if something fails.
     */
    protected void computeCameraPose(final PinholeCameraIntrinsicParameters intrinsic) throws CalibrationException {
        try {
            // reset previous values
            rotation = null;
            cameraCenter = null;
            camera = null;

            final var estimator = new CameraPoseEstimator();
            estimator.estimate(intrinsic, homography);

            rotation = estimator.getRotation();
            cameraCenter = estimator.getCameraCenter();
            camera = estimator.getCamera();

        } catch (final AlgebraException | GeometryException e) {
            throw new CalibrationException(e);
        }
    }
}
