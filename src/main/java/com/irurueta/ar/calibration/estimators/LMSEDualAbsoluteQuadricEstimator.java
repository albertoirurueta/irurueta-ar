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
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.ar.calibration.DualAbsoluteQuadric;
import com.irurueta.geometry.BaseQuadric;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.NumericalException;

import java.util.List;

/**
 * Implementation of a Dual Absolute Quadric estimator using an LMSE (Least Mean
 * Squared Error) solution for provided pinhole cameras.
 * This implementation assumes that:
 * - cameras are arbitrary (usually the initial camera is the identity and must
 * be discarded) as it creates a numerical degeneracy.
 * - all provided cameras have the same intrinsic parameters.
 * - it is assumed that skewness is zero, the principal point is at the center
 * of the image plane (zero), and both horizontal and vertical focal planes are
 * equal.
 */
@SuppressWarnings("DuplicatedCode")
public class LMSEDualAbsoluteQuadricEstimator extends DualAbsoluteQuadricEstimator {

    /**
     * Indicates if by default an LMSE (the Least Mean Square Error) solution is
     * allowed if more pinhole cameras than the minimum are provided.
     */
    public static final boolean DEFAULT_ALLOW_LMSE_SOLUTION = false;

    /**
     * Indicates if by default an LMSE (the Least Mean Square Error) solution is
     * allowed if more pinhole cameras than the minimum are provided.
     */
    private boolean allowLMSESolution;

    /**
     * Constructor.
     */
    public LMSEDualAbsoluteQuadricEstimator() {
        super();
        allowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }

    /**
     * Constructor with listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or estimation progress changes.
     */
    public LMSEDualAbsoluteQuadricEstimator(final DualAbsoluteQuadricEstimatorListener listener) {
        super(listener);
        allowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }

    /**
     * Constructor.
     *
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric
     *                (DAQ).
     * @throws IllegalArgumentException if list of cameras is null or invalid
     *                                  for default constraints.
     */
    public LMSEDualAbsoluteQuadricEstimator(final List<PinholeCamera> cameras) {
        super(cameras);
        allowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }

    /**
     * Constructor.
     *
     * @param cameras  list of cameras used to estimate the Dual Absolute Quadric
     *                 (DAQ).
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if list of cameras is null or invalid
     *                                  for default constraints.
     */
    public LMSEDualAbsoluteQuadricEstimator(
            final List<PinholeCamera> cameras, final DualAbsoluteQuadricEstimatorListener listener) {
        super(cameras, listener);
        allowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }

    /**
     * Indicates if an LMSE (the Least Mean Square Error) solution is allowed if
     * more pinhole cameras than the minimum are provided. If false, the
     * exceeding correspondences will be ignored and only the 2 first ones
     * will be used.
     *
     * @return true if LMSE solution is allowed, false otherwise.
     */
    public boolean isLMSESolutionAllowed() {
        return allowLMSESolution;
    }

    /**
     * Specifies if an LMSE (the Least Mean Square Error) solution is allowed if
     * more pinhole cameras than the minimum are provided. If false, the
     * exceeding correspondences will be ignored and only the 2 first ones
     * will be used.
     *
     * @param allowed true if LMSE solution is allowed, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setLMSESolutionAllowed(final boolean allowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        allowLMSESolution = allowed;
    }

    /**
     * Indicates whether current constraints are enough to start the estimation.
     * In order to obtain a linear solution for the DAQ estimation, we need at
     * least the principal point at origin constraint.
     *
     * @return true if constraints are valid, false otherwise.
     */
    @Override
    public boolean areValidConstraints() {
        final var valid = super.areValidConstraints();
        if (!valid) {
            return false;
        }

        if (allowLMSESolution) {
            return !(focalDistanceAspectRatioKnown && singularityEnforced);
        } else {
            return true;
        }
    }


    /**
     * Estimates the Dual Absolute Quadric using provided cameras.
     *
     * @param result instance where estimated Dual Absolute Quadric (DAQ) will
     *               be stored.
     * @throws LockedException                       if estimator is locked.
     * @throws NotReadyException                     if no valid input data has already been
     *                                               provided.
     * @throws DualAbsoluteQuadricEstimatorException if an error occurs during
     *                                               estimation, usually because input data is not valid or
     *                                               numerically unstable.
     */
    @Override
    public void estimate(final DualAbsoluteQuadric result) throws LockedException, NotReadyException,
            DualAbsoluteQuadricEstimatorException {

        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            locked = true;
            if (listener != null) {
                listener.onEstimateStart(this);
            }

            if (principalPointAtOrigin) {
                if (zeroSkewness) {
                    if (focalDistanceAspectRatioKnown) {
                        estimateZeroSkewnessPrincipalPointAtOriginAndKnownFocalDistanceAspectRatio(result);
                    } else {
                        estimateZeroSkewnessAndPrincipalPointAtOrigin(result);
                    }
                } else {
                    estimatePrincipalPointAtOrigin(result);
                }
            }

            if (listener != null) {
                listener.onEstimateEnd(this);
            }
        } finally {
            locked = false;
        }
    }

    /**
     * Returns type of Dual Absolute Quadric estimator.
     *
     * @return type of DAQ estimator.
     */
    @Override
    public DualAbsoluteQuadricEstimatorType getType() {
        return DualAbsoluteQuadricEstimatorType.LMSE_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR;
    }

    /**
     * Gets minimum number of equations required to find a solution.
     *
     * @return minimum number of equations required to find a solution.
     */
    private int getMinRequiredEquations() {
        return singularityEnforced ? MIN_REQUIRED_EQUATIONS - 1 : MIN_REQUIRED_EQUATIONS;
    }

    /**
     * Estimates Dual Absolute Quadric (DAQ) assuming that skewness is zero,
     * principal point is located at origin of coordinates and that aspect ratio
     * of focal distances is known.
     *
     * @param result instance where resulting estimated Dual Absolute Quadric
     *               will be stored.
     * @throws DualAbsoluteQuadricEstimatorException if an error occurs during
     *                                               estimation, usually because repeated cameras are
     *                                               provided, or cameras corresponding to critical motion
     *                                               sequences such as pure parallel translations are
     *                                               provided, where no additional data is really provided.
     */
    private void estimateZeroSkewnessPrincipalPointAtOriginAndKnownFocalDistanceAspectRatio(
            final DualAbsoluteQuadric result) throws DualAbsoluteQuadricEstimatorException {
        try {
            final var nCams = cameras.size();

            final Matrix a;
            if (isLMSESolutionAllowed()) {
                a = new Matrix(4 * nCams, BaseQuadric.N_PARAMS);
            } else {
                a = new Matrix(singularityEnforced ? 8 : 12, BaseQuadric.N_PARAMS);
            }

            Matrix cameraMatrix;
            double p11;
            double p12;
            double p13;
            double p14;
            double p21;
            double p22;
            double p23;
            double p24;
            double p31;
            double p32;
            double p33;
            double p34;
            var eqCounter = 0;
            final var minReqEqs = getMinRequiredEquations();
            for (final var camera : cameras) {

                // normalize cameras to increase accuracy
                camera.normalize();

                cameraMatrix = camera.getInternalMatrix();

                p11 = cameraMatrix.getElementAt(0, 0);
                p21 = cameraMatrix.getElementAt(1, 0);
                p31 = cameraMatrix.getElementAt(2, 0);

                p12 = cameraMatrix.getElementAt(0, 1);
                p22 = cameraMatrix.getElementAt(1, 1);
                p32 = cameraMatrix.getElementAt(2, 1);

                p13 = cameraMatrix.getElementAt(0, 2);
                p23 = cameraMatrix.getElementAt(1, 2);
                p33 = cameraMatrix.getElementAt(2, 2);

                p14 = cameraMatrix.getElementAt(0, 3);
                p24 = cameraMatrix.getElementAt(1, 3);
                p34 = cameraMatrix.getElementAt(2, 3);

                // 1st row
                fill2ndRowAnd1stRowEquation(p11, p21, p12, p22, p13, p23, p14, p24, a, eqCounter);
                eqCounter++;

                // 2nd row
                fill3rdRowAnd1stRowEquation(p11, p31, p12, p32, p13, p33, p14, p34, a, eqCounter);
                eqCounter++;

                // 3rd row
                fill3rdRowAnd2ndRowEquation(p21, p31, p22, p32, p23, p33, p24, p34, a, eqCounter);
                eqCounter++;

                // 4th row
                fill1stRowEqualTo2ndRowEquation(p11, p21, p12, p22, p13, p23, p14, p24, a, eqCounter);
                eqCounter++;

                if (!isLMSESolutionAllowed() && eqCounter >= minReqEqs) {
                    break;
                }
            }

            final var decomposer = new SingularValueDecomposer(a);
            enforceRank3IfNeeded(decomposer, result);

        } catch (final AlgebraException | NumericalException e) {
            throw new DualAbsoluteQuadricEstimatorException(e);
        }
    }

    /**
     * Estimates Dual Absolute Quadric (DAQ) assuming that skewness is zero,
     * and principal point is located at origin of coordinates.
     *
     * @param result instance where resulting estimated Dual Absolute Quadric
     *               will be stored.
     * @throws DualAbsoluteQuadricEstimatorException if an error occurs during
     *                                               estimation, usually because repeated cameras are
     *                                               provided, or cameras corresponding to critical motion
     *                                               sequences such as pure parallel translations are
     *                                               provided, where no additional data is really provided.
     */
    private void estimateZeroSkewnessAndPrincipalPointAtOrigin(final DualAbsoluteQuadric result)
            throws DualAbsoluteQuadricEstimatorException {
        try {
            final var nCams = cameras.size();

            final Matrix a;
            if (isLMSESolutionAllowed()) {
                a = new Matrix(3 * nCams, BaseQuadric.N_PARAMS);
            } else {
                a = new Matrix(9, BaseQuadric.N_PARAMS);
            }

            Matrix cameraMatrix;
            double p11;
            double p12;
            double p13;
            double p14;
            double p21;
            double p22;
            double p23;
            double p24;
            double p31;
            double p32;
            double p33;
            double p34;
            var eqCounter = 0;
            final var minReqEqs = getMinRequiredEquations();
            for (final var camera : cameras) {

                // normalize cameras to increase accuracy
                camera.normalize();

                cameraMatrix = camera.getInternalMatrix();

                p11 = cameraMatrix.getElementAt(0, 0);
                p21 = cameraMatrix.getElementAt(1, 0);
                p31 = cameraMatrix.getElementAt(2, 0);

                p12 = cameraMatrix.getElementAt(0, 1);
                p22 = cameraMatrix.getElementAt(1, 1);
                p32 = cameraMatrix.getElementAt(2, 1);

                p13 = cameraMatrix.getElementAt(0, 2);
                p23 = cameraMatrix.getElementAt(1, 2);
                p33 = cameraMatrix.getElementAt(2, 2);

                p14 = cameraMatrix.getElementAt(0, 3);
                p24 = cameraMatrix.getElementAt(1, 3);
                p34 = cameraMatrix.getElementAt(2, 3);

                // 1st row
                fill2ndRowAnd1stRowEquation(p11, p21, p12, p22, p13, p23, p14, p24, a, eqCounter);
                eqCounter++;

                // 2nd row
                fill3rdRowAnd1stRowEquation(p11, p31, p12, p32, p13, p33, p14, p34, a, eqCounter);
                eqCounter++;

                // 3rd row
                fill3rdRowAnd2ndRowEquation(p21, p31, p22, p32, p23, p33, p24, p34, a, eqCounter);
                eqCounter++;

                if (!isLMSESolutionAllowed() && eqCounter >= minReqEqs) {
                    break;
                }
            }

            final var decomposer = new SingularValueDecomposer(a);
            enforceRank3IfNeeded(decomposer, result);

        } catch (final AlgebraException | NumericalException e) {
            throw new DualAbsoluteQuadricEstimatorException(e);
        }
    }

    /**
     * Estimates Dual Absolute Quadric (DAQ) assuming that principal point is
     * zero.
     *
     * @param result instance where resulting estimated Dual Absolute Quadric
     *               will be stored.
     * @throws DualAbsoluteQuadricEstimatorException if an error occurs during
     *                                               estimation, usually because repeated cameras are
     *                                               provided, or cameras corresponding to critical motion
     *                                               sequences such as pure parallel translations are
     *                                               provided, where no additional data is really provided.
     */
    private void estimatePrincipalPointAtOrigin(final DualAbsoluteQuadric result)
            throws DualAbsoluteQuadricEstimatorException {
        try {
            final var nCams = cameras.size();

            final Matrix a;
            if (isLMSESolutionAllowed()) {
                a = new Matrix(2 * nCams, BaseQuadric.N_PARAMS);
            } else {
                a = new Matrix(singularityEnforced ? 8 : 10, BaseQuadric.N_PARAMS);
            }

            Matrix cameraMatrix;
            double p11;
            double p12;
            double p13;
            double p14;
            double p21;
            double p22;
            double p23;
            double p24;
            double p31;
            double p32;
            double p33;
            double p34;
            var eqCounter = 0;
            final var minReqEqs = getMinRequiredEquations();
            for (final var camera : cameras) {

                // normalize cameras to increase accuracy
                camera.normalize();

                cameraMatrix = camera.getInternalMatrix();

                p11 = cameraMatrix.getElementAt(0, 0);
                p21 = cameraMatrix.getElementAt(1, 0);
                p31 = cameraMatrix.getElementAt(2, 0);

                p12 = cameraMatrix.getElementAt(0, 1);
                p22 = cameraMatrix.getElementAt(1, 1);
                p32 = cameraMatrix.getElementAt(2, 1);

                p13 = cameraMatrix.getElementAt(0, 2);
                p23 = cameraMatrix.getElementAt(1, 2);
                p33 = cameraMatrix.getElementAt(2, 2);

                p14 = cameraMatrix.getElementAt(0, 3);
                p24 = cameraMatrix.getElementAt(1, 3);
                p34 = cameraMatrix.getElementAt(2, 3);

                // 1st row
                fill3rdRowAnd1stRowEquation(p11, p31, p12, p32, p13, p33, p14, p34, a, eqCounter);
                eqCounter++;

                // 2nd row
                fill3rdRowAnd2ndRowEquation(p21, p31, p22, p32, p23, p33, p24, p34, a, eqCounter);
                eqCounter++;

                if (!isLMSESolutionAllowed() && eqCounter >= minReqEqs) {
                    break;
                }
            }

            final var decomposer = new SingularValueDecomposer(a);
            enforceRank3IfNeeded(decomposer, result);

        } catch (final AlgebraException | NumericalException e) {
            throw new DualAbsoluteQuadricEstimatorException(e);
        }
    }
}
