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
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.ar.calibration.estimators.LMSERadialDistortionEstimator;
import com.irurueta.ar.calibration.estimators.RadialDistortionEstimatorException;
import com.irurueta.geometry.GeometryException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;

import java.io.Serializable;
import java.util.ArrayList;

/**
 * Class implementing Brown's radial distortion.
 * This kind of distortion is usually modelled after the distortion introduced
 * by lenses on cameras when taking short range pictures using wide angle
 * lenses.
 * Brown's radial distortion typically follow an expression such as:
 * xd = xu + (xu - xc)*(K1*r^2 + K2*r^4 + ...)
 * yd = yu + (yu - yc)*(K1*r^2 + K2*r^4 + ...),
 * where (xu, yu) stands for the undistorted point coordinates,
 * (xd, yd) stands for the distorted point coordinates,
 * (xc, yc) stands for the distortion center,
 * r^2 is typically (xu - xc)^2 + (yu - yc)^2 stands for the squared distance
 * of the distorted point respect to the distortion center. r^2 is computed
 * taking into account provided intrinsic parameters
 * And K1, K2 are the distortion parameters. Usually K1 dominates and K2 is
 * much smaller. Further terms are usually neglected as they are not meaningful
 * and typically produce numerical instabilities, but can also be provided in
 * array form.
 * <p>
 * NOTE: in order to be able to converge to correct values when computing
 * distortions, RadialDistortion should work with normalized point coordinates
 * between -1.0 and 1.0.
 */
@SuppressWarnings("DuplicatedCode")
public class RadialDistortion extends Distortion implements Serializable {

    /**
     * Defines default focal length if none is defined.
     */
    public static final double DEFAULT_FOCAL_LENGTH = 1.0;

    /**
     * Defines default skewness if none is defined.
     */
    public static final double DEFAULT_SKEW = 0.0;

    /**
     * Default maximum number of iterations to do when attempting to un-distort
     * a point if convergence is not reached.
     */
    public static final int DEFAULT_MAX_ITERS = 20;

    /**
     * Default tolerance to consider point convergence when un-distorting a point.
     */
    public static final double DEFAULT_TOLERANCE = 1e-5;

    /**
     * Radial distortion center.
     */
    private Point2D center;

    /**
     * Radial distortion parameters.
     */
    private double[] kParams;

    /**
     * Horizontal focal length expressed in pixels.
     */
    private double horizontalFocalLength;

    /**
     * Vertical focal length expressed in pixels.
     */
    private double verticalFocalLength;

    /**
     * Skew in pixels.
     */
    private double skew;

    /**
     * Inverse of intrinsic parameters' matrix.
     */
    private Matrix kInv;

    /**
     * Constructor.
     */
    public RadialDistortion() {
        try {
            setIntrinsic(null, DEFAULT_FOCAL_LENGTH, DEFAULT_FOCAL_LENGTH, DEFAULT_SKEW);
        } catch (final RadialDistortionException ignore) {
            // never happens
        }
    }

    /**
     * Constructor with radial distortion parameters and center assumed to be
     * at the origin of coordinates (0, 0).
     *
     * @param k1 first degree distortion parameter.
     * @param k2 second degree distortion parameter.
     */
    public RadialDistortion(final double k1, final double k2) {
        kParams = new double[]{k1, k2};
        try {
            setIntrinsic(null, DEFAULT_FOCAL_LENGTH, DEFAULT_FOCAL_LENGTH, DEFAULT_SKEW);
        } catch (final RadialDistortionException ignore) {
            // never happens
        }
    }

    /**
     * Constructor with radial distortion parameters and center assumed to be
     * at the origin of coordinates (0, 0).
     *
     * @param kParams radial distortion parameters of any length.
     * @throws IllegalArgumentException if radial distortion parameters is null.
     */
    public RadialDistortion(final double[] kParams) {
        if (kParams == null) {
            throw new IllegalArgumentException();
        }
        this.kParams = kParams;
        try {
            setIntrinsic(null, DEFAULT_FOCAL_LENGTH, DEFAULT_FOCAL_LENGTH, DEFAULT_SKEW);
        } catch (final RadialDistortionException ignore) {
            // never happens
        }
    }

    /**
     * Constructor with radial distortion parameters and center.
     *
     * @param k1     first degree distortion parameter.
     * @param k2     second degree distortion parameter.
     * @param center center of radial distortion. If null it is assumed to be
     *               at the origin of coordinates (0, 0), which is the typical value.
     */
    public RadialDistortion(final double k1, final double k2, final Point2D center) {
        this(k1, k2);
        try {
            setIntrinsic(center, DEFAULT_FOCAL_LENGTH, DEFAULT_FOCAL_LENGTH, DEFAULT_SKEW);
        } catch (final RadialDistortionException ignore) {
            // never happens
        }
    }

    /**
     * Constructor with radial distortion parameters and center.
     *
     * @param kParams radial distortion parameters of any length.
     * @param center  center of radial distortion. If null it is assumed to be
     *                at the origin of coordinates (0, 0), which is the typical value.
     * @throws IllegalArgumentException if radial distortion parameters is null.
     */
    public RadialDistortion(final double[] kParams, final Point2D center) {
        this(kParams);
        try {
            setIntrinsic(center, DEFAULT_FOCAL_LENGTH, DEFAULT_FOCAL_LENGTH, DEFAULT_SKEW);
        } catch (final RadialDistortionException ignore) {
            // never happens
        }
    }

    /**
     * Constructor with radial distortion parameters, center and other
     * camera intrinsic parameters.
     *
     * @param k1                    first degree distortion parameter.
     * @param k2                    second degree distortion parameter.
     * @param center                center of radial distortion. If null it is assumed to be at
     *                              the origin of coordinates (0, 0), which is the typical value.
     * @param horizontalFocalLength horizontal focal length expressed in pixels.
     * @param verticalFocalLength   vertical focal length expressed in pixels.
     * @param skew                  skew expressed in pixels.
     * @throws RadialDistortionException if provided focal lengths are
     *                                   degenerate (i.e. zero).
     */
    public RadialDistortion(
            final double k1, final double k2, final Point2D center, final double horizontalFocalLength,
            final double verticalFocalLength, final double skew) throws RadialDistortionException {
        this(k1, k2);
        setIntrinsic(center, horizontalFocalLength, verticalFocalLength, skew);
    }

    /**
     * Constructor with radial distortion parameters, center and other camera
     * intrinsic parameters.
     *
     * @param kParams               radial distortion parameters of any length.
     * @param center                center of radial distortion. If null it is assumed to be at
     *                              the origin of coordinates (0, 0), which is the typical value.
     * @param horizontalFocalLength horizontal focal length expressed in pixels.
     * @param verticalFocalLength   vertical focal length expressed in pixels.
     * @param skew                  skew expressed in pixels.
     * @throws RadialDistortionException if provided focal lengths are
     *                                   degenerate (i.e. zero).
     * @throws IllegalArgumentException  if radial distortion parameters is null.
     */
    public RadialDistortion(final double[] kParams, final Point2D center, final double horizontalFocalLength,
                            final double verticalFocalLength, final double skew) throws RadialDistortionException {
        this(kParams);
        setIntrinsic(center, horizontalFocalLength, verticalFocalLength, skew);
    }

    /**
     * Constructor with points and distortion center.
     *
     * @param distortedPoint1   1st distorted point (i.e. measured).
     * @param distortedPoint2   2nd distorted point (i.e. measured).
     * @param undistortedPoint1 1st undistorted point (i.e. ideal).
     * @param undistortedPoint2 2nd undistorted point (i.e. undistorted).
     * @param distortionCenter  distortion center or null if center is at origin
     *                          of coordinates (which is the typical value).
     * @throws RadialDistortionException if distortion could not be estimated.
     */
    public RadialDistortion(final Point2D distortedPoint1, final Point2D distortedPoint2,
                            final Point2D undistortedPoint1, final Point2D undistortedPoint2,
                            final Point2D distortionCenter) throws RadialDistortionException {
        super();

        setFromPointsAndCenter(distortedPoint1, distortedPoint2, undistortedPoint1, undistortedPoint2,
                distortionCenter);
    }

    /**
     * Estimates this radial distortion from points and distortion center.
     *
     * @param distortedPoint1   1st distorted point (i.e. measured).
     * @param distortedPoint2   2nd distorted point (i.e. measured).
     * @param undistortedPoint1 1st undistorted point (i.e. ideal).
     * @param undistortedPoint2 2nd undistorted point (i.e. undistorted).
     * @param distortionCenter  distortion center or null if center is at origin
     *                          of coordinates (which is the typical value).
     * @throws RadialDistortionException if distortion could not be estimated.
     */
    public final void setFromPointsAndCenter(
            final Point2D distortedPoint1, final Point2D distortedPoint2,
            final Point2D undistortedPoint1, final Point2D undistortedPoint2, final Point2D distortionCenter)
            throws RadialDistortionException {

        final var distortedPoints = new ArrayList<Point2D>();
        final var undistortedPoints = new ArrayList<Point2D>();

        distortedPoints.add(distortedPoint1);
        distortedPoints.add(distortedPoint2);

        undistortedPoints.add(undistortedPoint1);
        undistortedPoints.add(undistortedPoint2);

        try {
            final var estimator = new LMSERadialDistortionEstimator(distortedPoints, undistortedPoints,
                    distortionCenter);
            estimator.setLMSESolutionAllowed(false);
            final var distortion = estimator.estimate();

            kParams = distortion.kParams;
            center = distortion.center;
        } catch (final GeometryException | RadialDistortionEstimatorException e) {
            throw new RadialDistortionException(e);
        }
    }

    /**
     * Returns radial distortion center.
     *
     * @return radial distortion center.
     */
    public Point2D getCenter() {
        return center;
    }

    /**
     * Sets radial distortion center.
     *
     * @param center radial distortion center to be set.
     */
    public void setCenter(final Point2D center) {
        try {
            setIntrinsic(center, horizontalFocalLength, verticalFocalLength, skew);
        } catch (final RadialDistortionException ignore) {
            // never happens
        }
    }

    /**
     * Returns horizontal focal length expressed in pixels.
     *
     * @return horizontal focal length expressed in pixels.
     */
    public double getHorizontalFocalLength() {
        return horizontalFocalLength;
    }

    /**
     * Sets horizontal focal length expressed in pixels.
     *
     * @param horizontalFocalLength horizontal focal length expressed in pixels.
     * @throws RadialDistortionException if provided value is degenerate (i.e.
     *                                   zero).
     */
    public void setHorizontalFocalLength(final double horizontalFocalLength) throws RadialDistortionException {
        setIntrinsic(center, horizontalFocalLength, verticalFocalLength, skew);
    }

    /**
     * Returns vertical focal length expressed in pixels.
     *
     * @return vertical focal length expressed in pixels.
     */
    public double getVerticalFocalLength() {
        return verticalFocalLength;
    }

    /**
     * Sets vertical focal length expressed in pixels.
     *
     * @param verticalFocalLength vertical focal length expressed in pixels.
     * @throws RadialDistortionException if provided value is degenerate (i.e.
     *                                   zero).
     */
    public void setVerticalFocalLength(final double verticalFocalLength) throws RadialDistortionException {
        setIntrinsic(center, horizontalFocalLength, verticalFocalLength, skew);
    }

    /**
     * Returns skew expressed in pixels.
     *
     * @return skew expressed in pixels.
     */
    public double getSkew() {
        return skew;
    }

    /**
     * Sets skew expressed in pixels.
     *
     * @param skew skew expressed in pixels.
     */
    public void setSkew(final double skew) {
        try {
            setIntrinsic(center, horizontalFocalLength, verticalFocalLength, skew);
        } catch (final RadialDistortionException ignore) {
            // never happens
        }
    }

    /**
     * Returns pinhole camera intrinsic parameters associated to this distortion.
     *
     * @return pinhole camera intrinsic parameters associated to this distortion.
     */
    public PinholeCameraIntrinsicParameters getIntrinsic() {
        return new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength,
                center != null ? center.getInhomX() : 0.0,
                center != null ? center.getInhomY() : 0.0,
                skew);
    }

    /**
     * Sets intrinsic parameters from pinhole camera.
     *
     * @param intrinsic intrinsic parameters to be set.
     * @throws RadialDistortionException if focal length is degenerate (i.e.
     *                                   zero).
     */
    public void setIntrinsic(final PinholeCameraIntrinsicParameters intrinsic) throws RadialDistortionException {
        setIntrinsic(new InhomogeneousPoint2D(
                intrinsic.getHorizontalPrincipalPoint(), intrinsic.getVerticalPrincipalPoint()),
                intrinsic.getHorizontalFocalLength(),
                intrinsic.getVerticalFocalLength(),
                intrinsic.getSkewness());
    }

    /**
     * Sets intrinsic parameters.
     *
     * @param center                radial distortion center.
     * @param horizontalFocalLength horizontal focal length expressed in pixels.
     * @param verticalFocalLength   vertical focal length expressed in pixels.
     * @param skew                  skew expressed in pixels.
     * @throws RadialDistortionException if focal length is degenerate (i.e.
     *                                   zero).
     */
    public final void setIntrinsic(
            final Point2D center, final double horizontalFocalLength, final double verticalFocalLength,
            final double skew) throws RadialDistortionException {
        this.center = center;
        this.horizontalFocalLength = horizontalFocalLength;
        this.verticalFocalLength = verticalFocalLength;
        this.skew = skew;

        try {
            if (kInv == null) {
                kInv = new Matrix(3, 3);
            }

            // initially matrix is zero
            final var k = new Matrix(3, 3);

            k.setElementAt(0, 0, horizontalFocalLength);
            k.setElementAt(1, 1, verticalFocalLength);
            k.setElementAt(0, 1, skew);
            if (this.center != null) {
                // if center is not provided, values are zero
                k.setElementAt(0, 2, this.center.getInhomX());
                k.setElementAt(1, 2, this.center.getInhomY());
            }
            k.setElementAt(2, 2, 1.0);

            Utils.inverse(k, kInv);
        } catch (final AlgebraException e) {
            throw new RadialDistortionException(e);
        }
    }

    /**
     * Returns first degree distortion parameter or zero if not available.
     *
     * @return first degree distortion parameter or zero if not available.
     */
    public double getK1() {
        return kParams != null && kParams.length > 0 ? kParams[0] : 0.0;
    }

    /**
     * Sets first degree distortion parameter.
     *
     * @param k1 first degree distortion parameter.
     */
    public void setK1(final double k1) {
        if (kParams == null || kParams.length < 1) {
            kParams = new double[]{k1};
        }
    }

    /**
     * Returns second degree distortion parameter or zero if not available.
     *
     * @return second degree distortion parameter or zero if not available.
     */
    public double getK2() {
        return kParams != null && kParams.length > 1 ? kParams[1] : 0.0;
    }

    /**
     * Sets second degree distortion parameter.
     *
     * @param k2 second degree distortion parameter to be set.
     */
    public void setK2(final double k2) {
        if (kParams == null || kParams.length < 2) {
            final var kp = new double[2];
            kp[0] = getK1();
            this.kParams = kp;
        }
        kParams[1] = k2;
    }

    /**
     * Returns all radial distortion parameters. Typically only first and second
     * degree radial distortion parameters are used.
     *
     * @return all radial distortion parameters.
     */
    public double[] getKParams() {
        return kParams;
    }

    /**
     * Sets all radial distortion parameters. With this method more than 2
     * radial distortion parameters can be set if needed.
     *
     * @param kParams radial distortion parameters to be set.
     */
    public void setKParams(final double[] kParams) {
        this.kParams = kParams;
    }

    /**
     * Distorts provided 2D point and stores result into provided distorted
     * point.
     *
     * @param undistortedPoint undistorted point to be undistorted.
     * @param distortedPoint   distorted point where result is stored.
     */
    @Override
    public void distort(final Point2D undistortedPoint, final Point2D distortedPoint) {

        undistortedPoint.normalize();

        final var uHomX = undistortedPoint.getHomX();
        final var uHomY = undistortedPoint.getHomY();
        final var uHomW = undistortedPoint.getHomW();

        // multiply mKinv with homogeneous undistorted point coordinates
        // to normalize them respect to principal point and image size
        final var uNormHomX = kInv.getElementAt(0, 0) * uHomX
                + kInv.getElementAt(0, 1) * uHomY
                + kInv.getElementAt(0, 2) * uHomW;
        final double uNormHomY = kInv.getElementAt(1, 0) * uHomX
                + kInv.getElementAt(1, 1) * uHomY
                + kInv.getElementAt(1, 2) * uHomW;
        final double uNormHomW = kInv.getElementAt(2, 0) * uHomX
                + kInv.getElementAt(2, 1) * uHomY
                + kInv.getElementAt(2, 2) * uHomW;

        final var uNormInhomX = uNormHomX / uNormHomW;
        final var uNormInhomY = uNormHomY / uNormHomW;

        final var r2 = uNormInhomX * uNormInhomX + uNormInhomY * uNormInhomY;
        var r = r2;

        var sum = 0.0;
        if (kParams != null) {
            for (final var kParam : kParams) {
                sum += kParam * r;
                r *= r2;
            }
        }

        final var uInhomX = uHomX / uHomW;
        final var uInhomY = uHomY / uHomW;

        var centerX = 0.0;
        var centerY = 0.0;
        if (center != null) {
            centerX = center.getInhomX();
            centerY = center.getInhomY();
        }

        final var dInhomX = uInhomX + (uInhomX - centerX) * sum;
        final var dInhomY = uInhomY + (uInhomY - centerY) * sum;

        distortedPoint.setInhomogeneousCoordinates(dInhomX, dInhomY);
    }

    /**
     * Un-distorts provided 2D point and stores result into provided undistorted
     * point
     *
     * @param distortedPoint   distorted point to be undistorted
     * @param undistortedPoint undistorted point where result is stored
     */
    @Override
    public void undistort(final Point2D distortedPoint, final Point2D undistortedPoint) {
        undistort(distortedPoint, undistortedPoint, DEFAULT_MAX_ITERS, DEFAULT_TOLERANCE);
    }

    /**
     * Un-distorts provided 2D point and stores result into provided undistorted
     * point.
     *
     * @param distortedPoint   distorted point to be undistorted.
     * @param undistortedPoint undistorted point where result is stored.
     * @param maxIters         maximum number of iterations to un-distort a point in case
     *                         that convergence is not reached.
     * @param tolerance        tolerance to indicate that convergence has been reached.
     */
    public void undistort(final Point2D distortedPoint, final Point2D undistortedPoint, final int maxIters,
                          final double tolerance) {
        if (maxIters <= 0) {
            throw new IllegalArgumentException();
        }
        if (tolerance <= 0.0) {
            throw new IllegalArgumentException();
        }

        distortedPoint.normalize();

        final var dHomX = distortedPoint.getHomX();
        final var dHomY = distortedPoint.getHomY();
        final var dHomW = distortedPoint.getHomW();

        // initial estimate of undistorted point
        undistortedPoint.setHomogeneousCoordinates(dHomX, dHomY, dHomW);

        final var uHomX = undistortedPoint.getHomX();
        final var uHomY = undistortedPoint.getHomY();
        final var uHomW = undistortedPoint.getHomW();

        // multiply mKinv with homogeneous undistorted point coordinates
        final var uHomXDenorm = kInv.getElementAt(0, 0) * uHomX
                + kInv.getElementAt(0, 1) * uHomY
                + kInv.getElementAt(0, 2) * uHomW;
        final double uHomYDenorm = kInv.getElementAt(1, 0) * uHomX
                + kInv.getElementAt(1, 1) * uHomY
                + kInv.getElementAt(1, 2) * uHomW;
        final double uHomWDenorm = kInv.getElementAt(2, 0) * uHomX
                + kInv.getElementAt(2, 1) * uHomY
                + kInv.getElementAt(2, 2) * uHomW;

        final var origX = uHomXDenorm / uHomWDenorm;
        final var origY = uHomYDenorm / uHomWDenorm;
        var uInhomX = origX;
        var uInhomY = origY;

        // radial distortion magnitude
        var sum = 0.0;
        var prevSum = 0.0;
        for (var iter = 0; iter < maxIters; iter++) {
            // estimate the radial distance
            final var r2 = uInhomX * uInhomX + uInhomY * uInhomY;
            var r = r2;

            sum = 0.0;
            if (kParams != null) {
                for (final var kParam : kParams) {
                    sum += kParam * r;
                    r *= r2;
                }
            }

            uInhomX = origX / (1.0 + sum);
            uInhomY = origY / (1.0 + sum);

            if (Math.abs(prevSum - sum) <= tolerance) {
                break;
            } else {
                prevSum = sum;
            }
        }

        final var dInhomX = dHomX / dHomW;
        final var dInhomY = dHomY / dHomW;

        var centerX = 0.0;
        var centerY = 0.0;
        if (center != null) {
            centerX = center.getInhomX();
            centerY = center.getInhomY();
        }

        uInhomX = (dInhomX + centerX * sum) / (1.0 + sum);
        uInhomY = (dInhomY + centerY * sum) / (1.0 + sum);

        undistortedPoint.setInhomogeneousCoordinates(uInhomX, uInhomY);
    }

    /**
     * Indicates whether this instance can distort points.
     * This implementation always returns false, hence attempting to distort
     * a point will result in a NotSupportedException being raised.
     *
     * @return true if points can be distorted, false otherwise.
     */
    @Override
    public boolean canDistort() {
        return true;
    }

    /**
     * Indicates whether this instance can un-distort points.
     * This implementation always returns true.
     *
     * @return true if points can be undistorted, false otherwise.
     */
    @Override
    public boolean canUndistort() {
        return true;
    }


    /**
     * Returns kind of distortion.
     *
     * @return kind of distortion.
     */
    @Override
    public DistortionKind getKind() {
        return DistortionKind.BROWN_RADIAL_DISTORTION;
    }
}
