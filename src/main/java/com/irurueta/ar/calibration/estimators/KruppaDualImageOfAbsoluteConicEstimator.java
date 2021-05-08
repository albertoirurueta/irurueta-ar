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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.CholeskyDecomposer;
import com.irurueta.algebra.Complex;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.ar.calibration.DualImageOfAbsoluteConic;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.polynomials.Polynomial;

/**
 * Estimates the DIAC (Dual Image of Absolute Conic) by solving Kruppa's
 * equations and assuming known principal point and zero skewness.
 * This estimator allows enforcing a known aspect ratio as well.
 * The DIAC can be used to obtain the intrinsic parameters of a pair of
 * cameras related by a fundamental matrix.
 * Hence this class can be used for auto-calibration purposes.
 * Notice that the {@link DualAbsoluteQuadricEstimator} is a more robust method of
 * auto-calibration.
 * This class is based on:
 * S.D. Hippisley-Cox &amp; J.Porrill. Auto-calibration - Kruppa's equations and the
 * intrinsic parameters of a camera. AI Vision Research Unit. University of
 * Sheffield.
 */
@SuppressWarnings("DuplicatedCode")
public class KruppaDualImageOfAbsoluteConicEstimator {

    /**
     * Degree of polynomial to solve Kruppa's equation when aspect ratio is
     * known.
     */
    private static final int POLY_DEGREE_UNKNOWN_ASPECT_RATIO = 4;

    /**
     * Default value for horizontal principal point coordinate.
     */
    public static final double DEFAULT_PRINCIPAL_POINT_X = 0.0;

    /**
     * Default value for vertical principal point coordinate.
     */
    public static final double DEFAULT_PRINCIPAL_POINT_Y = 0.0;

    /**
     * Constant defining whether aspect ratio of focal distance (i.e. vertical
     * focal distance divided by horizontal focal distance) is known or not.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio. Typically LCD sensor cells are square and hence aspect
     * ratio of focal distances is known and equal to 1.
     */
    public static final boolean DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN =
            true;

    /**
     * Constant defining default aspect ratio of focal distances. This constant
     * takes into account that typically LCD sensor cells are square and hence
     * aspect ratio of focal distances is known and equal to 1.
     */
    public static final double DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO = 1.0;

    /**
     * Minimum absolute value allowed for aspect ratio of focal distances.
     */
    public static final double MIN_ABS_FOCAL_DISTANCE_ASPECT_RATIO = 1e-6;

    /**
     * Known horizontal principal point coordinate.
     */
    private double mPrincipalPointX;

    /**
     * Known vertical principal point coordinate.
     */
    private double mPrincipalPointY;

    /**
     * Indicates whether aspect ratio of focal distances (i.e. vertical focal
     * distance divided by horizontal focal distance) is known or not.
     * Notice that focal distance aspect ratio is not related to image aspect
     * ratio. Typically LCD sensor cells are square and hence aspect ratio of
     * focal distances is known and equal to 1.
     */
    private boolean mFocalDistanceAspectRatioKnown;

    /**
     * Contains aspect ratio of focal distances (i.e. vertical focal distance
     * divided by horizontal focal distance).
     * By default this is 1.0, since it is taken into account that typically
     * LCD sensor cells are square and hence aspect ratio focal distance is
     * known and equal to 1.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio.
     */
    private double mFocalDistanceAspectRatio;

    /**
     * True when estimator is estimating the DIAC.
     */
    private boolean mLocked;

    /**
     * Listener to be notified of events such as when estimation starts, ends or
     * estimation progress changes.
     */
    private KruppaDualImageOfAbsoluteConicEstimatorListener mListener;

    /**
     * Fundamental matrix to estimate DIAC from.
     */
    private FundamentalMatrix mFundamentalMatrix;

    /**
     * Constructor.
     */
    public KruppaDualImageOfAbsoluteConicEstimator() {
        mPrincipalPointX = DEFAULT_PRINCIPAL_POINT_X;
        mPrincipalPointY = DEFAULT_PRINCIPAL_POINT_Y;
        mFocalDistanceAspectRatioKnown =
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO_KNOWN;
        mFocalDistanceAspectRatio = DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO;

        mLocked = false;
        mListener = null;
        mFundamentalMatrix = null;
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or estimation progress changes.
     */
    public KruppaDualImageOfAbsoluteConicEstimator(
            final KruppaDualImageOfAbsoluteConicEstimatorListener listener) {
        this();
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix to estimate DIAC from.
     */
    public KruppaDualImageOfAbsoluteConicEstimator(
            final FundamentalMatrix fundamentalMatrix) {
        this();
        mFundamentalMatrix = fundamentalMatrix;
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix to estimate DIAC from.
     * @param listener          listener to be notified of events such as when estimation
     *                          starts, ends or estimation progress changes.
     */
    public KruppaDualImageOfAbsoluteConicEstimator(
            final FundamentalMatrix fundamentalMatrix,
            final KruppaDualImageOfAbsoluteConicEstimatorListener listener) {
        this(fundamentalMatrix);
        mListener = listener;
    }

    /**
     * Gets known horizontal principal point coordinate.
     *
     * @return known horizontal principal point coordinate.
     */
    public double getPrincipalPointX() {
        return mPrincipalPointX;
    }

    /**
     * Sets known horizontal principal point coordinate.
     *
     * @param principalPointX known horizontal principal point coordinate to be
     *                        set.
     * @throws LockedException if estimator is locked.
     */
    public void setPrincipalPointX(final double principalPointX)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPrincipalPointX = principalPointX;
    }

    /**
     * Gets known vertical principal point coordinate.
     *
     * @return known vertical principal point coordinate.
     */
    public double getPrincipalPointY() {
        return mPrincipalPointY;
    }

    /**
     * Sets known vertical principal point coordinate.
     *
     * @param principalPointY known vertical principal point coordinate to be
     *                        set.
     * @throws LockedException if estimator is locked.
     */
    public void setPrincipalPointY(final double principalPointY)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPrincipalPointY = principalPointY;
    }

    /**
     * Returns boolean indicating whether aspect ratio of focal distances (i.e.
     * vertical focal distance divided by horizontal focal distance) is known or
     * not.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio. Typically LCD sensor cells are square and hence aspect
     * ratio of focal distances is known and equal to 1.
     * This value is only taken into account if skewness is assumed to be zero,
     * otherwise it is ignored.
     *
     * @return true if focal distance aspect ratio is known, false otherwise.
     */
    public boolean isFocalDistanceAspectRatioKnown() {
        return mFocalDistanceAspectRatioKnown;
    }

    /**
     * Sets value indicating whether aspect ratio of focal distances (i.e.
     * vertical focal distance divided by horizontal focal distance) is known or
     * not.
     * Notice that focal distance aspect ratio is not related to image size
     * aspect ratio. Typically LCD sensor cells are square and hence aspect
     * ratio of focal distances is known and equal to 1.
     * This value is only taken into account if skewness is assumed to be
     * otherwise it is ignored.
     *
     * @param focalDistanceAspectRatioKnown true if focal distance aspect ratio
     *                                      is known, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setFocalDistanceAspectRatioKnown(
            final boolean focalDistanceAspectRatioKnown) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        mFocalDistanceAspectRatioKnown = focalDistanceAspectRatioKnown;
    }

    /**
     * Returns aspect ratio of focal distances (i.e. vertical focal distance
     * divided by horizontal focal distance).
     * By default this is 1.0, since it is taken into account that typically
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
     * @return aspect ratio of focal distances.
     */
    public double getFocalDistanceAspectRatio() {
        return mFocalDistanceAspectRatio;
    }

    /**
     * Sets aspect ratio of focal distances (i.e. vertical focal distance
     * divided by horizontal focal distance).
     * This value is only taken into account if aspect ratio is marked as known,
     * otherwise it is ignored.
     * By default this is 1.0, since it is taken into account that typically
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
     * @param focalDistanceAspectRatio aspect ratio of focal distances to be
     *                                 set.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if focal distance aspect ratio is too
     *                                  close to zero, as it might produce numerical instabilities.
     */
    public void setFocalDistanceAspectRatio(final double focalDistanceAspectRatio)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (Math.abs(focalDistanceAspectRatio) <
                MIN_ABS_FOCAL_DISTANCE_ASPECT_RATIO) {
            throw new IllegalArgumentException();
        }

        mFocalDistanceAspectRatio = focalDistanceAspectRatio;
    }

    /**
     * Indicates whether this instance is locked.
     *
     * @return true if this estimator is busy estimating a DIAC, false
     * otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }

    /**
     * Returns listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     *
     * @return listener to be notified of events.
     */
    public KruppaDualImageOfAbsoluteConicEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(
            final KruppaDualImageOfAbsoluteConicEstimatorListener listener)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }

    /**
     * Gets fundamental matrix to estimate DIAC from.
     *
     * @return fundamental matrix to estimate DIAC from.
     */
    public FundamentalMatrix getFundamentalMatrix() {
        return mFundamentalMatrix;
    }

    /**
     * Sets fundamental matrix to estimate DIAC from.
     *
     * @param fundamentalMatrix fundamental matrix to estimate DIAC from.
     * @throws LockedException if estimator is locked.
     */
    public void setFundamentalMatrix(final FundamentalMatrix fundamentalMatrix)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mFundamentalMatrix = fundamentalMatrix;
    }

    /**
     * Returns value indicating whether required data has been provided so that
     * DIAC estimation can start.
     * If true, estimator is ready to compute the DIAC, otherwise more data
     * needs to be provided.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mFundamentalMatrix != null;
    }

    /**
     * Estimates Dual Image of Absolute Conic (DIAC).
     *
     * @return estimated DIAC.
     * @throws LockedException                                  if estimator is locked.
     * @throws NotReadyException                                if input has not yet been provided.
     * @throws KruppaDualImageOfAbsoluteConicEstimatorException if an error
     *                                                          occurs during estimation, usually because fundamental
     *                                                          matrix corresponds to degenerate camera movements, or
     *                                                          because of numerical instabilities.
     */
    public DualImageOfAbsoluteConic estimate() throws LockedException,
            NotReadyException,
            KruppaDualImageOfAbsoluteConicEstimatorException {
        final DualImageOfAbsoluteConic result = new DualImageOfAbsoluteConic(0.0, 0.0,
                0.0, 0.0, 0.0, 0.0);
        estimate(result);
        return result;
    }

    /**
     * Estimates Dual Image of Absolute Conic (DIAC).
     *
     * @param result instance where estimated DIAC will be stored.
     * @throws LockedException                                  if estimator is locked.
     * @throws NotReadyException                                if input has not yet been provided.
     * @throws KruppaDualImageOfAbsoluteConicEstimatorException if an error
     *                                                          occurs during estimation, usually because fundamental
     *                                                          matrix corresponds to degenerate camera movements, or
     *                                                          because of numerical instabilities.
     */
    public void estimate(final DualImageOfAbsoluteConic result)
            throws LockedException, NotReadyException,
            KruppaDualImageOfAbsoluteConicEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            mLocked = true;

            if (mListener != null) {
                mListener.onEstimateStart(this);
            }

            if (mFocalDistanceAspectRatioKnown) {
                estimateKnownAspectRatio(result);
            } else {
                estimateUnknownAspectRatio(result);
            }

            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
        } finally {
            mLocked = false;
        }
    }

    /**
     * Builds a DIAC from estimated focal length components and current
     * principal point assuming zero skewness.
     *
     * @param horizontalFocalLength estimated horizontal focal length component.
     * @param verticalFocalLength   estimated vertical focal length component.
     * @param result                instance where estimated DIAC will be stored.
     * @return true if estimated DIAC is valid, false otherwise.
     */
    private boolean buildDiac(final double horizontalFocalLength,
                              final double verticalFocalLength,
                              final DualImageOfAbsoluteConic result) {

        try {
            final PinholeCameraIntrinsicParameters intrinsic = new
                    PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, mPrincipalPointX, mPrincipalPointY, 0.0);
            result.setFromPinholeCameraIntrinsicParameters(intrinsic);

            final Matrix m = result.asMatrix();
            final CholeskyDecomposer decomposer = new CholeskyDecomposer(m);
            decomposer.decompose();
            return decomposer.isSPD();
        } catch (final AlgebraException e) {
            // there are numerical instabilities
            return false;
        }
    }

    /**
     * Estimates the DIAC assuming unknown aspect ratio.
     *
     * @param result instance where estimated DIAC will be stored.
     * @throws KruppaDualImageOfAbsoluteConicEstimatorException if an error
     *                                                          occurs during estimation, usually because fundamental
     *                                                          matrix corresponds to degenerate camera movements, or
     *                                                          because of numerical instabilities.
     */
    private void estimateUnknownAspectRatio(final DualImageOfAbsoluteConic result)
            throws KruppaDualImageOfAbsoluteConicEstimatorException {
        try {
            final double x0 = mPrincipalPointX;
            final double y0 = mPrincipalPointY;

            // SVD decompose fundamental matrix
            mFundamentalMatrix.normalize();
            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    mFundamentalMatrix.getInternalMatrix());
            decomposer.decompose();

            final double[] sigmas = decomposer.getSingularValues();
            final Matrix u = decomposer.getU();
            final Matrix v = decomposer.getV();

            final double sigma1 = sigmas[0];
            final double sigma2 = sigmas[1];

            // Column u1
            final double u11 = u.getElementAt(0, 0);
            final double u21 = u.getElementAt(1, 0);
            final double u31 = u.getElementAt(2, 0);

            // Column u2
            final double u12 = u.getElementAt(0, 1);
            final double u22 = u.getElementAt(1, 1);
            final double u32 = u.getElementAt(2, 1);

            // Column v1
            final double v11 = v.getElementAt(0, 0);
            final double v21 = v.getElementAt(1, 0);
            final double v31 = v.getElementAt(2, 0);

            // Column v2
            final double v12 = v.getElementAt(0, 1);
            final double v22 = v.getElementAt(1, 1);
            final double v32 = v.getElementAt(2, 1);

            // build Kruppa equations
            final double polyA = u12 * u11;
            final double polyB = u22 * u21;
            final double polyC = Math.pow(x0, 2.0) * u12 * u11 + x0 * y0 * u22 * u11 + x0 * u32 * u11 +
                    x0 * y0 * u12 * u21 + Math.pow(y0, 2.0) * u22 * u21 + y0 * u32 * u21 +
                    x0 * u12 * u31 + y0 * u22 * u31 + u32 * u31;
            final double polyD = Math.pow(sigma2, 2.0) * v12 * v12;
            final double polyE = Math.pow(sigma2, 2.0) * v22 * v22;
            final double polyF = Math.pow(sigma2 * x0, 2.0) * v12 * v12 +
                    Math.pow(sigma2, 2.0) * x0 * y0 * v22 * v12 +
                    Math.pow(sigma2, 2.0) * x0 * v32 * v12 +
                    Math.pow(sigma2, 2.0) * x0 * y0 * v12 * v22 +
                    Math.pow(sigma2 * y0, 2.0) * v22 * v22 +
                    Math.pow(sigma2, 2.0) * y0 * v32 * v22 +
                    Math.pow(sigma2, 2.0) * x0 * v12 * v32 +
                    Math.pow(sigma2, 2.0) * y0 * v22 * v32 +
                    Math.pow(sigma2, 2.0) * v32 * v32;
            final double polyG = u11 * u11;
            final double polyH = u21 * u21;
            final double polyI = Math.pow(x0, 2.0) * u11 * u11 + x0 * y0 * u21 * u11 + x0 * u31 * u11 +
                    x0 * y0 * u11 * u21 + Math.pow(y0, 2.0) * u21 * u21 + y0 * u31 * u21 +
                    x0 * u11 * u31 + y0 * u21 * u31 + u31 * u31;
            final double polyJ = sigma1 * sigma2 * v12 * v11;
            final double polyK = sigma1 * sigma2 * v22 * v21;
            final double polyL = sigma1 * sigma2 * Math.pow(x0, 2.0) * v12 * v11 +
                    sigma1 * sigma2 * x0 * y0 * v22 * v11 + sigma1 * sigma2 * x0 * v32 * v11 +
                    sigma1 * sigma2 * x0 * y0 * v12 * v21 +
                    sigma1 * sigma2 * Math.pow(y0, 2.0) * v22 * v21 +
                    sigma1 * sigma2 * y0 * v32 * v21 + sigma1 * sigma2 * x0 * v12 * v31 +
                    sigma1 * sigma2 * y0 * v22 * v31 + sigma1 * sigma2 * v32 * v31;
            final double polyM = Math.pow(sigma1, 2.0) * v11 * v11;
            final double polyN = Math.pow(sigma1, 2.0) * v21 * v21;
            final double polyO = Math.pow(sigma1 * x0, 2.0) * v11 * v11 +
                    Math.pow(sigma1, 2.0) * x0 * y0 * v21 * v11 +
                    Math.pow(sigma1, 2.0) * x0 * v31 * v11 +
                    Math.pow(sigma1, 2.0) * x0 * y0 * v11 * v21 +
                    Math.pow(sigma1 * y0, 2.0) * v21 * v21 +
                    Math.pow(sigma1, 2.0) * y0 * v31 * v21 +
                    Math.pow(sigma1, 2.0) * x0 * v11 * v31 +
                    Math.pow(sigma1, 2.0) * y0 * v21 * v31 +
                    Math.pow(sigma1, 2.0) * v31 * v31;
            final double polyP = u12 * u12;
            final double polyQ = u22 * u22;
            final double polyR = Math.pow(x0, 2.0) * u12 * u12 + x0 * y0 * u22 * u12 + x0 * u32 * u12 +
                    x0 * y0 * u12 * u22 + Math.pow(y0, 2.0) * u22 * u22 + y0 * u32 * u22 +
                    x0 * u12 * u32 + y0 * u22 * u32 + u32 * u32;


            final double tmp = (polyP * polyJ + polyA * polyM) / (polyG * polyM - polyP * polyD);
            final double polyS = (tmp * (polyH * polyN - polyQ * polyE) - (polyQ * polyK + polyB * polyN));
            final double polyT = (tmp * (polyG * polyN + polyH * polyM - polyP * polyE - polyQ * polyD) -
                    (polyP * polyK + polyQ * polyJ + polyA * polyN + polyB * polyM));
            final double polyU = (tmp * (polyG * polyO + polyM * polyI - polyP * polyF - polyD * polyR) -
                    (polyP * polyL + polyJ * polyR + polyA * polyO + polyM * polyC));
            final double polyV = (tmp * (polyH * polyO + polyN * polyI - polyQ * polyF - polyE * polyR) -
                    (polyQ * polyL + polyK * polyR + polyB * polyO + polyN * polyC));
            final double polyW = (tmp * (polyO * polyI - polyF * polyR) - (polyL * polyR + polyO * polyC));

            // assuming that x = ax^2, y = ay^2 which are the horizontal and
            // vertical focal lengths, we obtain the following equations

            // x = (-y^2*S - y*V - W) / (y*T + U)
            // (-y^2*S - y*V - W)^2 *(-A*D - G*J) + y^2*(y*T + U)^2*(-B*E - H*K) +
            // (-y^2*S - y*V - W)*(y*T + U)*y*(-A*E - B*D - G*K - H*J) +
            // (-y^2*S - y*V - W)*(y*T + U)*(-A*F - D*C - G*L - J*I) +
            // y*(y*T + U)^2*(-B*F - E*C - H*L - K*I) +
            // (y*T + U)^2*(- F*C - L*I) = 0
            // (-y^2*S - y*V - W)^2*(G*M - P*D) + y^2*(y*T + U)^2*(H*N - Q*E) +
            // (-y^2*S - y*V - W)*(y*T + U)*y*(G*N + H*M - P*E - Q*D) +
            // (-y^2*S - y*V - W)*(y*T + U)*(G*O + M*I - P*F - D*R) +
            // y*(y*T + U)^2*(H*O + N*I - Q*F - E*R) + (y*T + U)^2*(O*I - F*R) = 0

            // where we can solve y using any of the two latter equations, and
            // then use obtained y to solve x
            final Complex[] roots = unknownAspectRatioRoots(polyA, polyB, polyC,
                    polyD, polyE, polyF, polyG, polyH,
                    polyI, polyJ, polyK, polyL, polyM,
                    polyN, polyO, polyP, polyQ, polyR,
                    polyS, polyT, polyU, polyV, polyW);

            // roots contain possible y values. We use only their real part
            // and find x = (-y^2*S - y*V - W) / (y*T + U)

            // pick the best x, y values that produce a positive definite DIAC
            // matrix
            final DualImageOfAbsoluteConic diac = new DualImageOfAbsoluteConic(
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            boolean valid = false;
            if (roots != null) {
                for (final Complex root : roots) {
                    final double y = root.getReal();
                    final double x = getXFromY(y, polyS, polyT, polyU, polyV, polyW);

                    // build DIAC matrix and check if it is positive definite
                    if (x >= 0.0 && y >= 0.0) {
                        final double horizontalFocalLength = Math.sqrt(x);
                        final double verticalFocalLength = Math.sqrt(y);
                        valid = buildDiac(horizontalFocalLength,
                                verticalFocalLength, diac);
                    }

                    if (valid) {
                        break;
                    }
                }
            } else {
                throw new KruppaDualImageOfAbsoluteConicEstimatorException();
            }

            if (valid) {
                // copy to result
                result.setParameters(diac.getA(), diac.getB(), diac.getC(),
                        diac.getD(), diac.getE(), diac.getF());
            } else {
                // no valid DIAC could be found
                throw new KruppaDualImageOfAbsoluteConicEstimatorException();
            }

        } catch (final KruppaDualImageOfAbsoluteConicEstimatorException ex) {
            throw ex;
        } catch (final Exception ex) {
            throw new KruppaDualImageOfAbsoluteConicEstimatorException(ex);
        }
    }

    /**
     * Gets x value from current y value.
     * X and y values are the squared values of estimated focal length
     * components.
     * This method is used internally when aspect ratio is not known.
     *
     * @param y y value to obtain x value from.
     * @param s internal value from Kruppa's equations.
     * @param t internal value from Kruppa's equations.
     * @param u internal value from Kruppa's equations.
     * @param v internal value from Kruppa's equations.
     * @param w internal value from Kruppa's equations.
     * @return x value.
     */
    private double getXFromY(final double y, final double s, final double t,
                             final double u, final double v, final double w) {
        return (-Math.pow(y, 2.0) * s - y * v - w) / (y * t + u);
    }

    /**
     * One of Kruppa's equations expressed as a polynomial of degree 4 to solve
     * y value, which is the squared value of vertical focal length.
     * This method is only used when aspect ratio is unknown.
     *
     * @param a internal value from Kruppa's equations.
     * @param b internal value from Kruppa's equations.
     * @param c internal value from Kruppa's equations.
     * @param d internal value from Kruppa's equations.
     * @param e internal value from Kruppa's equations.
     * @param f internal value from Kruppa's equations.
     * @param g internal value from Kruppa's equations.
     * @param h internal value from Kruppa's equations.
     * @param i internal value from Kruppa's equations.
     * @param j internal value from Kruppa's equations.
     * @param k internal value from Kruppa's equations.
     * @param l internal value from Kruppa's equations.
     * @param s internal value from Kruppa's equations.
     * @param t internal value from Kruppa's equations.
     * @param u internal value from Kruppa's equations.
     * @param v internal value from Kruppa's equations.
     * @param w internal value from Kruppa's equations.
     * @return a polynomial.
     */
    private Polynomial buildPolynomial1(
            final double a, final double b, final double c, final double d,
            final double e, final double f, final double g, final double h,
            final double i, final double j, final double k, final double l,
            final double s, final double t, final double u, final double v,
            final double w) {
        // (-y^2*S - y*V - W)^2 *(-A*D - G*J) + y^2*(y*T + U)^2*(-B*E - H*K) +
        // (-y^2*S - y*V - W)*(y*T + U)*y*(-A*E - B*D - G*K - H*J) +
        // (-y^2*S - y*V - W)*(y*T + U)*(-A*F - D*C - G*L - J*I) +
        // y*(y*T + U)^2*(-B*F - E*C - H*L - K*I) + (y*T + U)^2*(- F*C - L*I) = 0

        final Polynomial result = new Polynomial(
                POLY_DEGREE_UNKNOWN_ASPECT_RATIO + 1);

        // (-y^2*S - y*V - W)^2 *(-A*D - G*J)
        final Polynomial tmp = new Polynomial(-w, -v, -s);
        final Polynomial tmp2 = new Polynomial(-w, -v, -s);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(-a * d - g * j);
        result.add(tmp);

        // y^2*(y*T + U)^2*(-B*E - H*K)
        tmp.setPolyParams(0.0, 0.0, 1.0);
        tmp2.setPolyParams(u, t);
        tmp.multiply(tmp2);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(-b * e - h * k);
        result.add(tmp);

        // (-y^2*S - y*V - W)*(y*T + U)*y*(-A*E - B*D - G*K - H*J)
        tmp.setPolyParams(-w, -v, -s);
        tmp2.setPolyParams(u, t);
        tmp.multiply(tmp2);
        tmp2.setPolyParams(0.0, 1);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(-a * e - b * d - g * k - h * j);
        result.add(tmp);

        // (-y^2*S - y*V - W)*(y*T + U)*(-A*F - D*C - G*L - J*I)
        tmp.setPolyParams(-w, -v, -s);
        tmp2.setPolyParams(u, t);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(-a * f - d * c - g * l - j * i);
        result.add(tmp);

        // y*(y*T + U)^2*(-B*F - E*C - H*L - K*I)
        tmp.setPolyParams(0.0, 1.0);
        tmp2.setPolyParams(u, t);
        tmp.multiply(tmp2);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(-b * f - e * c - h * l - k * i);
        result.add(tmp);

        // (y*T + U)^2*(- F*C - L*I)
        tmp.setPolyParams(u, t);
        tmp2.setPolyParams(u, t);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(-f * c - l * i);
        result.add(tmp);

        return result;
    }

    /**
     * Another of Kruppa's equations expressed as a polynomial of degree 4 to
     * solve y value, which is the squared value of vertical focal length.
     * This method is only used when aspect ratio is unknown.
     *
     * @param d internal value from Kruppa's equations.
     * @param e internal value from Kruppa's equations.
     * @param f internal value from Kruppa's equations.
     * @param g internal value from Kruppa's equations.
     * @param h internal value from Kruppa's equations.
     * @param i internal value from Kruppa's equations.
     * @param m internal value from Kruppa's equations.
     * @param n internal value from Kruppa's equations.
     * @param o internal value from Kruppa's equations.
     * @param p internal value from Kruppa's equations.
     * @param q internal value from Kruppa's equations.
     * @param r internal value from Kruppa's equations.
     * @param s internal value from Kruppa's equations.
     * @param t internal value from Kruppa's equations.
     * @param u internal value from Kruppa's equations.
     * @param v internal value from Kruppa's equations.
     * @param w internal value from Kruppa's equations.
     * @return a polynomial.
     */
    private Polynomial buildPolynomial2(
            final double d, final double e, final double f, final double g,
            final double h, final double i, final double m, final double n,
            final double o, final double p, final double q, final double r,
            final double s, final double t, final double u, final double v,
            final double w) {
        // (-y^2*S - y*V - W)^2*(G*M - P*D) + y^2*(y*T + U)^2*(H*N - Q*E) +
        // (-y^2*S - y*V - W)*(y*T + U)*y*(G*N + H*M - P*E - Q*D) +
        // (-y^2*S - y*V - W)*(y*T + U)*(G*O + M*I - P*F - D*R) +
        // y*(y*T + U)^2*(H*O + N*I - Q*F - E*R) + (y*T + U)^2*(O*I - F*R) = 0
        final Polynomial result = new Polynomial(
                POLY_DEGREE_UNKNOWN_ASPECT_RATIO + 1);

        // (-y^2*S - y*V - W)^2*(G*M - P*D)
        final Polynomial tmp = new Polynomial(-w, -v, -s);
        final Polynomial tmp2 = new Polynomial(-w, -v, -s);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(g * m - p * d);
        result.add(tmp);

        // y^2*(y*T + U)^2*(H*N - Q*E)
        tmp.setPolyParams(0.0, 0.0, 1.0);
        tmp2.setPolyParams(u, t);
        tmp.multiply(tmp2);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(h * n - q * e);
        result.add(tmp);

        // (-y^2*S - y*V - W)*(y*T + U)*y*(G*N + H*M - P*E - Q*D)
        tmp.setPolyParams(-w, -v, -s);
        tmp2.setPolyParams(u, t);
        tmp.multiply(tmp2);
        tmp2.setPolyParams(0.0, 1.0);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(g * n + h * m - p * e - q * d);
        result.add(tmp);

        // (-y^2*S - y*V - W)*(y*T + U)*(G*O + M*I - P*F - D*R)
        tmp.setPolyParams(-w, -v, -s);
        tmp2.setPolyParams(u, t);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(g * o + m * i - p * f - d * r);
        result.add(tmp);

        // y*(y*T + U)^2*(H*O + N*I - Q*F - E*R)
        tmp.setPolyParams(0.0, 1.0);
        tmp2.setPolyParams(u, t);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(h * o + n * i - q * f - e * r);
        result.add(tmp);

        // (y*T + U)^2*(O*I - F*R)
        tmp.setPolyParams(u, t);
        tmp2.setPolyParams(u, t);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(o * i - f * r);
        result.add(tmp);

        return result;
    }

    /**
     * Estimates the DIAC assuming known aspect ratio.
     *
     * @param result instance where estimated DIAC will be stored.
     * @throws KruppaDualImageOfAbsoluteConicEstimatorException if an error
     *                                                          occurs during estimation, usually because fundamental
     *                                                          matrix corresponds to degenerate camera movements, or
     *                                                          because of numerical instabilities.
     */
    private void estimateKnownAspectRatio(final DualImageOfAbsoluteConic result)
            throws KruppaDualImageOfAbsoluteConicEstimatorException {
        try {
            final double x0 = mPrincipalPointX;
            final double y0 = mPrincipalPointY;

            // SVD decompose fundamental matrix
            mFundamentalMatrix.normalize();
            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    mFundamentalMatrix.getInternalMatrix());
            decomposer.decompose();

            final double[] sigmas = decomposer.getSingularValues();
            final Matrix u = decomposer.getU();
            final Matrix v = decomposer.getV();

            final double sigma1 = sigmas[0];
            final double sigma2 = sigmas[1];

            // Column u1
            final double u11 = u.getElementAt(0, 0);
            final double u21 = u.getElementAt(1, 0);
            final double u31 = u.getElementAt(2, 0);

            // Column u2
            final double u12 = u.getElementAt(0, 1);
            final double u22 = u.getElementAt(1, 1);
            final double u32 = u.getElementAt(2, 1);

            // Column v1
            final double v11 = v.getElementAt(0, 0);
            final double v21 = v.getElementAt(1, 0);
            final double v31 = v.getElementAt(2, 0);

            // Column v2
            final double v12 = v.getElementAt(0, 1);
            final double v22 = v.getElementAt(1, 1);
            final double v32 = v.getElementAt(2, 1);

            // build Kruppa equations
            final double polyA = u12 * u11;
            final double polyB = u22 * u21;
            final double polyC = Math.pow(x0, 2.0) * u12 * u11 + x0 * y0 * u22 * u11 + x0 * u32 * u11 +
                    x0 * y0 * u12 * u21 + Math.pow(y0, 2.0) * u22 * u21 + y0 * u32 * u21 +
                    x0 * u12 * u31 + y0 * u22 * u31 + u32 * u31;
            final double polyD = Math.pow(sigma2, 2.0) * v12 * v12;
            final double polyE = Math.pow(sigma2, 2.0) * v22 * v22;
            final double polyF = Math.pow(sigma2 * x0, 2.0) * v12 * v12 +
                    Math.pow(sigma2, 2.0) * x0 * y0 * v22 * v12 +
                    Math.pow(sigma2, 2.0) * x0 * v32 * v12 +
                    Math.pow(sigma2, 2.0) * x0 * y0 * v12 * v22 +
                    Math.pow(sigma2 * y0, 2.0) * v22 * v22 +
                    Math.pow(sigma2, 2.0) * y0 * v32 * v22 +
                    Math.pow(sigma2, 2.0) * x0 * v12 * v32 +
                    Math.pow(sigma2, 2.0) * y0 * v22 * v32 +
                    Math.pow(sigma2, 2.0) * v32 * v32;
            final double polyG = u11 * u11;
            final double polyH = u21 * u21;
            final double polyI = Math.pow(x0, 2.0) * u11 * u11 + x0 * y0 * u21 * u11 + x0 * u31 * u11 +
                    x0 * y0 * u11 * u21 + Math.pow(y0, 2.0) * u21 * u21 + y0 * u31 * u21 +
                    x0 * u11 * u31 + y0 * u21 * u31 + u31 * u31;
            final double polyJ = sigma1 * sigma2 * v12 * v11;
            final double polyK = sigma1 * sigma2 * v22 * v21;
            final double polyL = sigma1 * sigma2 * Math.pow(x0, 2.0) * v12 * v11 +
                    sigma1 * sigma2 * x0 * y0 * v22 * v11 + sigma1 * sigma2 * x0 * v32 * v11 +
                    sigma1 * sigma2 * x0 * y0 * v12 * v21 +
                    sigma1 * sigma2 * Math.pow(y0, 2.0) * v22 * v21 +
                    sigma1 * sigma2 * y0 * v32 * v21 + sigma1 * sigma2 * x0 * v12 * v31 +
                    sigma1 * sigma2 * y0 * v22 * v31 + sigma1 * sigma2 * v32 * v31;
            final double polyM = Math.pow(sigma1, 2.0) * v11 * v11;
            final double polyN = Math.pow(sigma1, 2.0) * v21 * v21;
            final double polyO = Math.pow(sigma1 * x0, 2.0) * v11 * v11 +
                    Math.pow(sigma1, 2.0) * x0 * y0 * v21 * v11 +
                    Math.pow(sigma1, 2.0) * x0 * v31 * v11 +
                    Math.pow(sigma1, 2.0) * x0 * y0 * v11 * v21 +
                    Math.pow(sigma1 * y0, 2.0) * v21 * v21 +
                    Math.pow(sigma1, 2.0) * y0 * v31 * v21 +
                    Math.pow(sigma1, 2.0) * x0 * v11 * v31 +
                    Math.pow(sigma1, 2.0) * y0 * v21 * v31 +
                    Math.pow(sigma1, 2.0) * v31 * v31;
            final double polyP = u12 * u12;
            final double polyQ = u22 * u22;
            final double polyR = Math.pow(x0, 2.0) * u12 * u12 + x0 * y0 * u22 * u12 + x0 * u32 * u12 +
                    x0 * y0 * u12 * u22 + Math.pow(y0, 2.0) * u22 * u22 + y0 * u32 * u22 +
                    x0 * u12 * u32 + y0 * u22 * u32 + u32 * u32;

            // try to solve any of Kruppa's equations
            final Complex[] roots = knownAspectRatioRoots(polyA, polyB, polyC, polyD,
                    polyE, polyF, polyG, polyH, polyI, polyJ, polyK, polyL, polyM,
                    polyN, polyO, polyP, polyQ, polyR);

            // roots contain possible x values. We use only their real part

            // pick the best x, y values that produce a positive definite DIAC
            // matrix
            final DualImageOfAbsoluteConic diac = new DualImageOfAbsoluteConic(
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            boolean valid = false;
            if (roots != null) {
                final double r = mFocalDistanceAspectRatio;
                final double r2 = r * r;
                for (final Complex root : roots) {
                    final double x = root.getReal();
                    final double y = r2 * x;

                    // build DIAC matrix and check if it is positive definite
                    if (x >= 0.0 && y >= 0.0) {
                        final double horizontalFocalLength = Math.sqrt(x);
                        final double verticalFocalLength = Math.sqrt(y);
                        valid = buildDiac(horizontalFocalLength,
                                verticalFocalLength, diac);
                    }

                    if (valid) {
                        break;
                    }
                }
            } else {
                throw new KruppaDualImageOfAbsoluteConicEstimatorException();
            }

            if (valid) {
                // copy to result
                result.setParameters(diac.getA(), diac.getB(), diac.getC(),
                        diac.getD(), diac.getE(), diac.getF());
            } else {
                // no valid DIAC could be found
                throw new KruppaDualImageOfAbsoluteConicEstimatorException();
            }

        } catch (final KruppaDualImageOfAbsoluteConicEstimatorException ex) {
            throw ex;
        } catch (final Exception ex) {
            throw new KruppaDualImageOfAbsoluteConicEstimatorException(ex);
        }
    }

    /**
     * Another of Kruppa's equations expressed as a polynomial of degree 2 to
     * solve x value, which is the squared value of horizontal focal length.
     * This method is only used when aspect ratio is known.
     *
     * @param a internal value from Kruppa's equations.
     * @param b internal value from Kruppa's equations.
     * @param c internal value from Kruppa's equations.
     * @param d internal value from Kruppa's equations.
     * @param e internal value from Kruppa's equations.
     * @param f internal value from Kruppa's equations.
     * @param g internal value from Kruppa's equations.
     * @param h internal value from Kruppa's equations.
     * @param i internal value from Kruppa's equations.
     * @param j internal value from Kruppa's equations.
     * @param k internal value from Kruppa's equations.
     * @param l internal value from Kruppa's equations.
     * @return a polynomial.
     */
    private Polynomial buildPolynomial3(
            final double a, final double b, final double c, final double d,
            final double e, final double f, final double g, final double h,
            final double i, final double j, final double k, final double l) {

        // x^2*((-A*D - G*J) + r^4*(-B*E - H*K) + r^2*(-A*E - B*D - G*K - H*J)) +
        // x*((-A*F - D*C - G*L - J*I) + r^2*(-B*F - E*C - H*L - K*I)) +
        // (- F*C - L*I) = 0
        final double r = mFocalDistanceAspectRatio;
        final double r2 = r * r;
        final double r4 = r2 * r2;
        return new Polynomial(-f * c - l * i,
                (-a * f - d * c - g * l - j * i) + r2 * (-b * f - e * c - h * l - k * i),
                ((-a * d - g * j) + r4 * (-b * e - h * k) + r2 * (-a * e - b * d - g * k - h * j)));
    }

    /**
     * Another of Kruppa's equations expressed as a polynomial of degree 2 to
     * solve x value, which is the squared value of horizontal focal length.
     * This method is only used when aspect ratio is known.
     *
     * @param d internal value from Kruppa's equations.
     * @param e internal value from Kruppa's equations.
     * @param f internal value from Kruppa's equations.
     * @param g internal value from Kruppa's equations.
     * @param h internal value from Kruppa's equations.
     * @param i internal value from Kruppa's equations.
     * @param m internal value from Kruppa's equations.
     * @param n internal value from Kruppa's equations.
     * @param o internal value from Kruppa's equations.
     * @param p internal value from Kruppa's equations.
     * @param q internal value from Kruppa's equations.
     * @param r internal value from Kruppa's equations.
     * @return a polynomial.
     */
    private Polynomial buildPolynomial4(
            final double d, final double e, final double f, final double g,
            final double h, final double i, final double m, final double n,
            final double o, final double p, final double q, final double r) {

        // x^2*((G*M - P*D) + r^4*(H*N - Q*E) + r^2*(G*N + H*M - P*E - Q*D)) +
        // x*((G*O + M*I - P*F - D*R) + r^2*(H*O + N*I - Q*F - E*R)) +
        // (O*I - F*R) = 0
        final double r1 = mFocalDistanceAspectRatio;
        final double r2 = r1 * r1;
        final double r4 = r2 * r2;
        return new Polynomial(o * i - f * r,
                (g * o + m * i - p * f - d * r) + r2 * (h * o + n * i - q * f - e * r),
                (g * m - p * d) + r4 * (h * n - q * e) + r2 * (g * n + h * m - p * e - q * d));
    }

    /**
     * Another of Kruppa's equations expressed as a polynomial of degree 2 to
     * solve x value, which is the squared value of horizontal focal length.
     * This method is only used when aspect ratio is known.
     *
     * @param a internal value from Kruppa's equations.
     * @param b internal value from Kruppa's equations.
     * @param c internal value from Kruppa's equations.
     * @param j internal value from Kruppa's equations.
     * @param k internal value from Kruppa's equations.
     * @param l internal value from Kruppa's equations.
     * @param m internal value from Kruppa's equations.
     * @param n internal value from Kruppa's equations.
     * @param o internal value from Kruppa's equations.
     * @param p internal value from Kruppa's equations.
     * @param q internal value from Kruppa's equations.
     * @param r internal value from Kruppa's equations.
     * @return a polynomial.
     */
    private Polynomial buildPolynomial5(
            final double a, final double b, final double c, final double j,
            final double k, final double l, final double m, final double n,
            final double o, final double p, final double q, final double r) {

        // x^2*((P*J + A*M) + r^4*(Q*K + B*N) + r^2*(P*K + Q*J + A*N + B*M)) +
        // x*((P*L + J*R + A*O + M*C) + r^2*(Q*L + K*R + B*O + N*C)) +
        // (L*R + O*C) = 0
        final double r1 = mFocalDistanceAspectRatio;
        final double r2 = r1 * r1;
        final double r4 = r2 * r2;
        return new Polynomial(l * r + o * c,
                (p * l + j * r + a * o + m * c) + r2 * (q * l + k * r + b * o + n * c),
                (p * j + a * m) + r4 * (q * k + b * n) + r2 * (p * k + q * j + a * n + b * m));
    }

    /**
     * Solves Kruppa's equations when aspect ratio is unknown
     *
     * @param polyA A parameter of Kruppa's polynomial equation.
     * @param polyB B parameter of Kruppa's polynomial equation.
     * @param polyC C parameter of Kruppa's polynomial equation.
     * @param polyD D parameter of Kruppa's polynomial equation.
     * @param polyE E parameter of Kruppa's polynomial equation.
     * @param polyF F parameter of Kruppa's polynomial equation.
     * @param polyG G parameter of Kruppa's polynomial equation.
     * @param polyH H parameter of Kruppa's polynomial equation.
     * @param polyI I parameter of Kruppa's polynomial equation.
     * @param polyJ J parameter of Kruppa's polynomial equation.
     * @param polyK K parameter of Kruppa's polynomial equation.
     * @param polyL L parameter of Kruppa's polynomial equation.
     * @param polyM M parameter of Kruppa's polynomial equation.
     * @param polyN N parameter of Kruppa's polynomial equation.
     * @param polyO O parameter of Kruppa's polynomial equation.
     * @param polyP P parameter of Kruppa's polynomial equation.
     * @param polyQ Q parameter of Kruppa's polynomial equation.
     * @param polyR R parameter of Kruppa's polynomial equation.
     * @param polyS S parameter of Kruppa's polynomial equation.
     * @param polyT T parameter of Kruppa's polynomial equation.
     * @param polyU U parameter of Kruppa's polynomial equation.
     * @param polyV V parameter of Kruppa's polynomial equation.
     * @param polyW W parameter of Kruppa's polynomial equation.
     * @return roots solving Kruppa's equations.
     * @throws NumericalException if there are numerical instabilities.
     */
    private Complex[] unknownAspectRatioRoots(
            final double polyA, final double polyB, final double polyC, final double polyD,
            final double polyE, final double polyF, final double polyG, final double polyH,
            final double polyI, final double polyJ, final double polyK, final double polyL,
            final double polyM, final double polyN, final double polyO, final double polyP,
            final double polyQ, final double polyR, final double polyS, final double polyT,
            final double polyU, final double polyV, final double polyW) throws NumericalException {
        Complex[] roots;
        try {
            final Polynomial poly1 = buildPolynomial1(polyA, polyB, polyC, polyD, polyE, polyF, polyG, polyH, polyI,
                    polyJ, polyK, polyL, polyS, polyT, polyU, polyV, polyW);
            roots = poly1.getRoots();
        } catch (final NumericalException ex1) {
            // if solution for poly1 fails, try with second polynomial
            final Polynomial poly2 = buildPolynomial2(polyD, polyE, polyF, polyG, polyH, polyI,
                    polyM, polyN, polyO, polyP, polyQ, polyR, polyS, polyT, polyU, polyV, polyW);
            roots = poly2.getRoots();
        }

        return roots;
    }

    /**
     * Solves Kruppa's equations when aspect ratio is known
     *
     * @param polyA A parameter of Kruppa's polynomial equation.
     * @param polyB B parameter of Kruppa's polynomial equation.
     * @param polyC C parameter of Kruppa's polynomial equation.
     * @param polyD D parameter of Kruppa's polynomial equation.
     * @param polyE E parameter of Kruppa's polynomial equation.
     * @param polyF F parameter of Kruppa's polynomial equation.
     * @param polyG G parameter of Kruppa's polynomial equation.
     * @param polyH H parameter of Kruppa's polynomial equation.
     * @param polyI I parameter of Kruppa's polynomial equation.
     * @param polyJ J parameter of Kruppa's polynomial equation.
     * @param polyK K parameter of Kruppa's polynomial equation.
     * @param polyL L parameter of Kruppa's polynomial equation.
     * @param polyM M parameter of Kruppa's polynomial equation.
     * @param polyN N parameter of Kruppa's polynomial equation.
     * @param polyO O parameter of Kruppa's polynomial equation.
     * @param polyP P parameter of Kruppa's polynomial equation.
     * @param polyQ Q parameter of Kruppa's polynomial equation.
     * @param polyR R parameter of Kruppa's polynomial equation.
     * @return roots solving Kruppa's equations.
     * @throws NumericalException if there are numerical instabilities.
     */
    private Complex[] knownAspectRatioRoots(
            final double polyA, final double polyB, final double polyC, final double polyD,
            final double polyE, final double polyF, final double polyG, final double polyH,
            final double polyI, final double polyJ, final double polyK, final double polyL,
            final double polyM, final double polyN, final double polyO, final double polyP,
            final double polyQ, final double polyR) throws NumericalException {
        Complex[] roots;
        try {
            final Polynomial poly3 = buildPolynomial3(polyA, polyB, polyC, polyD, polyE, polyF, polyG, polyH, polyI,
                    polyJ, polyK, polyL);
            roots = poly3.getRoots();
        } catch (final NumericalException e3) {
            try {
                // if solution for poly3 fails, try with 4th polynomial
                final Polynomial poly4 = buildPolynomial4(polyD, polyE, polyF, polyG, polyH,
                        polyI, polyM, polyN, polyO, polyP, polyQ, polyR);
                roots = poly4.getRoots();
            } catch (final NumericalException e4) {
                final Polynomial poly5 = buildPolynomial5(polyA, polyB, polyC,
                        polyJ, polyK, polyL, polyM, polyN, polyO, polyP, polyQ, polyR);
                roots = poly5.getRoots();
            }
        }

        return roots;
    }
}
