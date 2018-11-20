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

import com.irurueta.algebra.*;
import com.irurueta.ar.calibration.DualImageOfAbsoluteConic;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.ar.epipolar.FundamentalMatrix;
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
 * Hence this class can be used for autocalibration purposes.
 * Notice that the DualAbsoluteQuadricEstimator is a more robust method of
 * autocalibration.
 * This class is based on:
 * S.D. Hippisley-Cox &amp; J.Porrill. Auto-calibration - Kruppa's equations and the
 * intrinsic parameters of a camera. AI Vision Research Unit. University of 
 * Sheffield.
 */
@SuppressWarnings("WeakerAccess")
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
     * Notic that focal distance aspect ratio is not related to image aspect 
     * ratio. Typically LCD sensor cells are square and hence aspectratio of 
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
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public KruppaDualImageOfAbsoluteConicEstimator(
            KruppaDualImageOfAbsoluteConicEstimatorListener listener) {
        this();
        mListener = listener;
    }
    
    /**
     * Constructor.
     * @param fundamentalMatrix fundamental matrix to estimate DIAC from.
     */
    public KruppaDualImageOfAbsoluteConicEstimator(
            FundamentalMatrix fundamentalMatrix) {
        this();
        mFundamentalMatrix = fundamentalMatrix;
    }
    
    /**
     * Constructor.
     * @param fundamentalMatrix fundamental matrix to estimate DIAC from.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public KruppaDualImageOfAbsoluteConicEstimator(
            FundamentalMatrix fundamentalMatrix, 
            KruppaDualImageOfAbsoluteConicEstimatorListener listener) {
        this(fundamentalMatrix);
        mListener = listener;
    }
    
    /**
     * Gets known horizontal principal point coordinate.
     * @return known horizontal principal point coordinate.
     */
    public double getPrincipalPointX() {
        return mPrincipalPointX;
    }
    
    /**
     * Sets known horizontal principal point coordinate.
     * @param principalPointX known horizontal principal point coordinate to be 
     * set.
     * @throws LockedException if estimator is locked.
     */
    public void setPrincipalPointX(double principalPointX) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPrincipalPointX = principalPointX;
    }

    /**
     * Gets known vertical principal point coordinate.
     * @return known vertical principal point coordinate.
     */
    public double getPrincipalPointY() {
        return mPrincipalPointY;
    }
    
    /**
     * Sets known vertical principal point coordinate.
     * @param principalPointY known vertical principal point coordinate to be
     * set.
     * @throws LockedException if estimator is locked.
     */
    public void setPrincipalPointY(double principalPointY) 
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
     * @param focalDistanceAspectRatioKnown true if focal distance aspect ratio
     * is known, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setFocalDistanceAspectRatioKnown(
            boolean focalDistanceAspectRatioKnown) throws LockedException {
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
     * @param focalDistanceAspectRatio aspect raito of focal distances to be 
     * set.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if focal distance aspect ratio is too 
     * close to zero, as it might produce numerical instabilities.
     */
    public void setFocalDistanceAspectRatio(double focalDistanceAspectRatio)
            throws LockedException, IllegalArgumentException {
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
     * @return true if this estimator is busy estimating a DIAC, false
     * otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }
    
    /**
     * Returns listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     * @return listener to be notified of events.
     */
    public KruppaDualImageOfAbsoluteConicEstimatorListener getListener() {
        return mListener;
    }
    
    /**
     * Sets listener to be notified of events such as when estimation starts,
     * ends or estimation progress changes.
     * @param listener listener to be notified of events.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(
            KruppaDualImageOfAbsoluteConicEstimatorListener listener) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }
    
    /**
     * Gets fundamental matrix to estimate DIAC from.
     * @return fundamental matrix to estimate DIAC from.
     */
    public FundamentalMatrix getFundamentalMatrix() {
        return mFundamentalMatrix;
    }
    
    /**
     * Sets fundamental matrix to estimate DIAC from.
     * @param fundamentalMatrix fundamental matrix to estimate DIAC from.
     * @throws LockedException if estimator is locked.
     */
    public void setFundamentalMatrix(FundamentalMatrix fundamentalMatrix) 
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
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mFundamentalMatrix != null;
    }
        
    /**
     * Estimates Dual Image of Absolute Conic (DIAC).
     * @return estimated DIAC.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if input has not yet been provided.
     * @throws KruppaDualImageOfAbsoluteConicEstimatorException if an error 
     * occurs during estimation, usually because fundamental matrix corresponds
     * to degenerate camera movements, or because of numerical unstabilities.
     */
    public DualImageOfAbsoluteConic estimate() throws LockedException,
            NotReadyException, 
            KruppaDualImageOfAbsoluteConicEstimatorException {
        DualImageOfAbsoluteConic result = new DualImageOfAbsoluteConic(0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0);
        estimate(result);
        return result;
    }
    
    /**
     * Estimates Dual Image of Absolute Conic (DIAC).
     * @param result instance where estimated DIAC will be stored.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if input has not yet been provided.
     * @throws KruppaDualImageOfAbsoluteConicEstimatorException if an error
     * occurs during estimation, usually because fundamental matrix corresponds
     * to degenerate camera movements, or because of numerical unstabilities.
     */
    public void estimate(DualImageOfAbsoluteConic result) 
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
     * @param horizontalFocalLength estimated horizontal focal length component.
     * @param verticalFocalLength estimated vertical focal length component.
     * @param result instance where estimated DIAC will be stored.
     * @return true if estimated DIAC is valid, false otherwise.
     * @throws AlgebraException if it cannot be determined whether DIAC is valid
     * or not due to numerical unstabilities.
     */
    private boolean buildDiac(double horizontalFocalLength, 
            double verticalFocalLength, DualImageOfAbsoluteConic result) 
            throws AlgebraException {
        
        PinholeCameraIntrinsicParameters intrinsic = new 
        PinholeCameraIntrinsicParameters(horizontalFocalLength, 
                verticalFocalLength, mPrincipalPointX, mPrincipalPointY, 0.0);
        result.setFromPinholeCameraIntrinsicParameters(intrinsic);
        
        Matrix m = result.asMatrix();
        CholeskyDecomposer decomposer = new CholeskyDecomposer(m);
        decomposer.decompose();
        return decomposer.isSPD();
    }    
    
    /**
     * Estimates the DIAC assuming unknown aspect ratio.
     * @param result instance where estimated DIAC will be stored.
     * @throws KruppaDualImageOfAbsoluteConicEstimatorException if an error
     * occurs during estimation, usually because fundamental matrix corresponds
     * to degenerate camera movements, or because of numerical unstabilities.
     */
    private void estimateUnknownAspectRatio(DualImageOfAbsoluteConic result)
            throws KruppaDualImageOfAbsoluteConicEstimatorException {
        try {
            double x0 = mPrincipalPointX;
            double y0 = mPrincipalPointY;      
        
            //SVD decompose fundamental matrix
            mFundamentalMatrix.normalize();
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    mFundamentalMatrix.getInternalMatrix());
            decomposer.decompose();
            
            double[] sigmas = decomposer.getSingularValues();
            Matrix u = decomposer.getU();
            Matrix v = decomposer.getV();
            
            double sigma1 = sigmas[0];
            double sigma2 = sigmas[1];
            
            //Column u1
            double u11 = u.getElementAt(0, 0);
            double u21 = u.getElementAt(1, 0);
            double u31 = u.getElementAt(2, 0);
            
            //Column u2
            double u12 = u.getElementAt(0, 1);
            double u22 = u.getElementAt(1, 1);
            double u32 = u.getElementAt(2, 1);
            
            //Column v1
            double v11 = v.getElementAt(0, 0);
            double v21 = v.getElementAt(1, 0);
            double v31 = v.getElementAt(2, 0);
            
            //Column v2
            double v12 = v.getElementAt(0, 1);
            double v22 = v.getElementAt(1, 1);
            double v32 = v.getElementAt(2, 1);
            
            //build Kruppa equations
            double A = u12*u11;
            double B = u22*u21;
            double C = Math.pow(x0,2.0)*u12*u11 + x0*y0*u22*u11 + x0*u32*u11 + 
                    x0*y0*u12*u21 + Math.pow(y0,2.0)*u22*u21 + y0*u32*u21 + 
                    x0*u12*u31 + y0*u22*u31 + u32*u31;
            double D = Math.pow(sigma2,2.0)*v12*v12;
            double E = Math.pow(sigma2,2.0)*v22*v22;
            double F = Math.pow(sigma2*x0,2.0)*v12*v12 + 
                    Math.pow(sigma2,2.0)*x0*y0*v22*v12 + 
                    Math.pow(sigma2,2.0)*x0*v32*v12 + 
                    Math.pow(sigma2,2.0)*x0*y0*v12*v22 + 
                    Math.pow(sigma2*y0,2)*v22*v22 + 
                    Math.pow(sigma2,2.0)*y0*v32*v22 + 
                    Math.pow(sigma2,2.0)*x0*v12*v32 + 
                    Math.pow(sigma2,2.0)*y0*v22*v32 + 
                    Math.pow(sigma2,2.0)*v32*v32;
            double G = u11*u11;
            double H = u21*u21;
            double I = Math.pow(x0,2.0)*u11*u11 + x0*y0*u21*u11 + x0*u31*u11 + 
                    x0*y0*u11*u21 + Math.pow(y0,2.0)*u21*u21 + y0*u31*u21 + 
                    x0*u11*u31 + y0*u21*u31 + u31*u31;
            double J = sigma1*sigma2*v12*v11;
            double K = sigma1*sigma2*v22*v21;
            double L = sigma1*sigma2*Math.pow(x0,2.0)*v12*v11 + 
                    sigma1*sigma2*x0*y0*v22*v11 + sigma1*sigma2*x0*v32*v11 + 
                    sigma1*sigma2*x0*y0*v12*v21 + 
                    sigma1*sigma2*Math.pow(y0,2.0)*v22*v21 + 
                    sigma1*sigma2*y0*v32*v21 + sigma1*sigma2*x0*v12*v31 + 
                    sigma1*sigma2*y0*v22*v31 + sigma1*sigma2*v32*v31;
            double M = Math.pow(sigma1,2.0)*v11*v11;
            double N = Math.pow(sigma1,2.0)*v21*v21;
            double O = Math.pow(sigma1*x0,2.0)*v11*v11 + 
                    Math.pow(sigma1,2.0)*x0*y0*v21*v11 + 
                    Math.pow(sigma1,2.0)*x0*v31*v11 + 
                    Math.pow(sigma1,2.0)*x0*y0*v11*v21 + 
                    Math.pow(sigma1*y0,2.0)*v21*v21 + 
                    Math.pow(sigma1,2.0)*y0*v31*v21 + 
                    Math.pow(sigma1,2.0)*x0*v11*v31 + 
                    Math.pow(sigma1,2.0)*y0*v21*v31 + 
                    Math.pow(sigma1,2.0)*v31*v31;
            double P = u12*u12;
            double Q = u22*u22;
            double R = Math.pow(x0,2.0)*u12*u12 + x0*y0*u22*u12 + x0*u32*u12 + 
                    x0*y0*u12*u22 + Math.pow(y0,2.0)*u22*u22 + y0*u32*u22 + 
                    x0*u12*u32 + y0*u22*u32 + u32*u32;
            
            
            double S = ((P*J + A*M)/(G*M - P*D)*(H*N - Q*E) - (Q*K + B*N));
            double T = ((P*J + A*M)/(G*M - P*D)*(G*N + H*M - P*E - Q*D) - 
                    (P*K + Q*J + A*N + B*M));
            double U = ((P*J + A*M)/(G*M - P*D)*(G*O + M*I - P*F - D*R) - 
                    (P*L + J*R + A*O + M*C));
            double V = ((P*J + A*M)/(G*M - P*D)*(H*O + N*I - Q*F - E*R) - 
                    (Q*L + K*R + B*O + N*C));
            double W = ((P*J + A*M)/(G*M - P*D)*(O*I - F*R) - (L*R + O*C));
            
            //assuming that x = ax^2, y = ay^2 which are the horizontal and
            //vertical focal lengths, we obtain the following equations
            
            //x = (-y^2*S - y*V - W) / (y*T + U)
            //(-y^2*S - y*V - W)^2 *(-A*D - G*J) + y^2*(y*T + U)^2*(-B*E - H*K) + 
            //(-y^2*S - y*V - W)*(y*T + U)*y*(-A*E - B*D - G*K - H*J) + 
            //(-y^2*S - y*V - W)*(y*T + U)*(-A*F - D*C - G*L - J*I) + 
            //y*(y*T + U)^2*(-B*F - E*C - H*L - K*I) + 
            //(y*T + U)^2*(- F*C - L*I) = 0
            //(-y^2*S - y*V - W)^2*(G*M - P*D) + y^2*(y*T + U)^2*(H*N - Q*E) + 
            //(-y^2*S - y*V - W)*(y*T + U)*y*(G*N + H*M - P*E - Q*D) + 
            //(-y^2*S - y*V - W)*(y*T + U)*(G*O + M*I - P*F - D*R) + 
            //y*(y*T + U)^2*(H*O + N*I - Q*F - E*R) + (y*T + U)^2*(O*I - F*R) = 0
            
            //where we can solve y using any of the two latter equations, and
            //then use obtained y to solve x
            Complex[] roots;
            try {
                Polynomial poly1 = buildPolynomial1(A, B, C, D, E, F, G, H, I, 
                        J, K, L, S, T, U, V, W);
                roots = poly1.getRoots();
            } catch (NumericalException e1) {
                //if solution for poly1 fails, try with second polynomial
                Polynomial poly2 = buildPolynomial2(D, E, F, G, H, I,
                        M, N, O, P, Q, R, S, T, U, V, W);
                roots = poly2.getRoots();
            }
            
            //roots contain possible y values. We use only their real part
            //and find x = (-y^2*S - y*V - W) / (y*T + U)
            
            //pick the best x, y values that produce a positive definite DIAC
            //matrix
            DualImageOfAbsoluteConic diac = new DualImageOfAbsoluteConic(
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            boolean valid = false;
            if (roots != null) {
                for (Complex root : roots) {
                    double y = root.getReal();
                    double x = getXFromY(y, S, T, U, V, W);
                    
                    //build DIAC matrix and check if it is positive definite
                    if (x >= 0.0 && y >= 0.0) {
                        double horizontalFocalLength = Math.sqrt(x);
                        double verticalFocalLength = Math.sqrt(y);
                        try {
                            valid = buildDiac(horizontalFocalLength, 
                                verticalFocalLength, diac);
                        } catch (AlgebraException e) {
                            valid = false;
                        }
                    }
                    
                    if (valid) {
                        break;
                    }
                }
            } else {
                throw new KruppaDualImageOfAbsoluteConicEstimatorException();
            }
            
            if (valid) {
                //copy to result
                result.setParameters(diac.getA(), diac.getB(), diac.getC(), 
                        diac.getD(), diac.getE(), diac.getF());
            } else {
                //no valid DIAC could be found
                throw new KruppaDualImageOfAbsoluteConicEstimatorException();
            }
            
        } catch (KruppaDualImageOfAbsoluteConicEstimatorException e) {
            throw e;
        } catch (Exception e) {
            throw new KruppaDualImageOfAbsoluteConicEstimatorException(e);
        }        
    }
        
    /**
     * Gets x value from current y value.
     * X and y values are the squared values of estimated focal lngth 
     * components.
     * This method is used internally when aspect ratio is not known.
     * @param y y value to obtian x value from.
     * @param S internal value from Kruppa's equations.
     * @param T internal value from Kruppa's equations.
     * @param U internal value from Kruppa's equations.
     * @param V internal value from Kruppa's equations.
     * @param W internal value from Kruppa's equations.
     * @return x value.
     */
    private double getXFromY(double y, double S, double T, double U, double V, 
            double W) {
        return (-Math.pow(y, 2.0)*S - y*V - W) / (y*T + U);
    }
    
    /**
     * One of Kruppa's equations expressed as a polynomial of degree 4 to solve
     * y value, which is the squared value of vertical focal length.
     * This method is only used when aspect ratio is unknown.
     * @param A internal value from Kruppa's equations.
     * @param B internal value from Kruppa's equations.
     * @param C internal value from Kruppa's equations.
     * @param D internal value from Kruppa's equations.
     * @param E internal value from Kruppa's equations.
     * @param F internal value from Kruppa's equations.
     * @param G internal value from Kruppa's equations.
     * @param H internal value from Kruppa's equations.
     * @param I internal value from Kruppa's equations.
     * @param J internal value from Kruppa's equations.
     * @param K internal value from Kruppa's equations.
     * @param L internal value from Kruppa's equations.
     * @param S internal value from Kruppa's equations.
     * @param T internal value from Kruppa's equations.
     * @param U internal value from Kruppa's equations.
     * @param V internal value from Kruppa's equations.
     * @param W internal value from Kruppa's equations.
     * @return a polynomial.
     */
    private Polynomial buildPolynomial1(double A, double B, double C, double D,
            double E, double F, double G, double H, double I, double J, 
            double K, double L, double S, double T, double U, double V,
            double W) {
        //(-y^2*S - y*V - W)^2 *(-A*D - G*J) + y^2*(y*T + U)^2*(-B*E - H*K) + 
        //(-y^2*S - y*V - W)*(y*T + U)*y*(-A*E - B*D - G*K - H*J) + 
        //(-y^2*S - y*V - W)*(y*T + U)*(-A*F - D*C - G*L - J*I) + 
        //y*(y*T + U)^2*(-B*F - E*C - H*L - K*I) + (y*T + U)^2*(- F*C - L*I) = 0
        
        Polynomial result = new Polynomial(
                POLY_DEGREE_UNKNOWN_ASPECT_RATIO + 1);
        
        //(-y^2*S - y*V - W)^2 *(-A*D - G*J)
        Polynomial tmp = new Polynomial(-W, -V, -S);
        Polynomial tmp2 = new Polynomial(-W, -V, -S);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(-A*D - G*J);
        result.add(tmp);
        
        //y^2*(y*T + U)^2*(-B*E - H*K)
        tmp.setPolyParams(0.0, 0.0, 1.0);
        tmp2.setPolyParams(U, T);
        tmp.multiply(tmp2);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(-B*E - H*K);
        result.add(tmp);
        
        //(-y^2*S - y*V - W)*(y*T + U)*y*(-A*E - B*D - G*K - H*J)
        tmp.setPolyParams(-W, -V, -S);
        tmp2.setPolyParams(U, T);
        tmp.multiply(tmp2);
        tmp2.setPolyParams(0.0, 1);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(-A*E - B*D - G*K - H*J);
        result.add(tmp);
        
        //(-y^2*S - y*V - W)*(y*T + U)*(-A*F - D*C - G*L - J*I)
        tmp.setPolyParams(-W, -V, -S);
        tmp2.setPolyParams(U, T);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(-A*F - D*C - G*L - J*I);
        result.add(tmp);
        
        //y*(y*T + U)^2*(-B*F - E*C - H*L - K*I)
        tmp.setPolyParams(0.0, 1.0);
        tmp2.setPolyParams(U, T);
        tmp.multiply(tmp2);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(-B*F - E*C - H*L - K*I);
        result.add(tmp);
        
        //(y*T + U)^2*(- F*C - L*I)
        tmp.setPolyParams(U, T);
        tmp2.setPolyParams(U, T);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(-F*C - L*I);
        result.add(tmp);
        
        return result;
    }
    
    /**
     * Another of Kruppa's equations expressed as a polynomial of degree 4 to
     * solve y value, which is the squared value of vertical focal length.
     * This method is only used when aspect ratio is unknown.
     * @param D internal value from Kruppa's equations.
     * @param E internal value from Kruppa's equations.
     * @param F internal value from Kruppa's equations.
     * @param G internal value from Kruppa's equations.
     * @param H internal value from Kruppa's equations.
     * @param I internal value from Kruppa's equations.
     * @param M internal value from Kruppa's equations.
     * @param N internal value from Kruppa's equations.
     * @param O internal value from Kruppa's equations.
     * @param P internal value from Kruppa's equations.
     * @param Q internal value from Kruppa's equations.
     * @param R internal value from Kruppa's equations.
     * @param S internal value from Kruppa's equations.
     * @param T internal value from Kruppa's equations.
     * @param U internal value from Kruppa's equations.
     * @param V internal value from Kruppa's equations.
     * @param W internal value from Kruppa's equations.
     * @return a polynomial.
     */
    private Polynomial buildPolynomial2(double D,
            double E, double F, double G, double H, double I,
            double M, double N, double O, double P,
            double Q, double R, double S, double T, double U, double V, 
            double W) {
        //(-y^2*S - y*V - W)^2*(G*M - P*D) + y^2*(y*T + U)^2*(H*N - Q*E) + 
        //(-y^2*S - y*V - W)*(y*T + U)*y*(G*N + H*M - P*E - Q*D) + 
        //(-y^2*S - y*V - W)*(y*T + U)*(G*O + M*I - P*F - D*R) + 
        //y*(y*T + U)^2*(H*O + N*I - Q*F - E*R) + (y*T + U)^2*(O*I - F*R) = 0
        Polynomial result = new Polynomial(
                POLY_DEGREE_UNKNOWN_ASPECT_RATIO + 1);
        
        //(-y^2*S - y*V - W)^2*(G*M - P*D)
        Polynomial tmp = new Polynomial(-W, -V, -S);
        Polynomial tmp2 = new Polynomial(-W, -V, -S);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(G*M - P*D);
        result.add(tmp);
        
        //y^2*(y*T + U)^2*(H*N - Q*E)
        tmp.setPolyParams(0.0, 0.0, 1.0);
        tmp2.setPolyParams(U, T);
        tmp.multiply(tmp2);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(H*N - Q*E);
        result.add(tmp);
        
        //(-y^2*S - y*V - W)*(y*T + U)*y*(G*N + H*M - P*E - Q*D)
        tmp.setPolyParams(-W, -V, -S);
        tmp2.setPolyParams(U, T);
        tmp.multiply(tmp2);
        tmp2.setPolyParams(0.0, 1.0);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(G*N + H*M - P*E - Q*D);
        result.add(tmp);
        
        //(-y^2*S - y*V - W)*(y*T + U)*(G*O + M*I - P*F - D*R)
        tmp.setPolyParams(-W, -V, -S);
        tmp2.setPolyParams(U, T);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(G*O + M*I - P*F - D*R);
        result.add(tmp);
        
        //y*(y*T + U)^2*(H*O + N*I - Q*F - E*R)
        tmp.setPolyParams(0.0, 1.0);
        tmp2.setPolyParams(U, T);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(H*O + N*I - Q*F - E*R);
        result.add(tmp);
        
        //(y*T + U)^2*(O*I - F*R)
        tmp.setPolyParams(U, T);
        tmp2.setPolyParams(U, T);
        tmp.multiply(tmp2);
        tmp.multiplyByScalar(O*I - F*R);
        result.add(tmp);
        
        return result;
    }
    
    /**
     * Estimates the DIAC assuming known aspect ratio.
     * @param result instance where estimated DIAC will be stored.
     * @throws KruppaDualImageOfAbsoluteConicEstimatorException if an error
     * occurs during estimation, usually because fundamental matrix corresponds
     * to degenerate camera movements, or because of numerical unstabilities.
     */
    private void estimateKnownAspectRatio(DualImageOfAbsoluteConic result)
            throws KruppaDualImageOfAbsoluteConicEstimatorException {
        try {
            double x0 = mPrincipalPointX;
            double y0 = mPrincipalPointY;      
        
            //SVD decompose fundamental matrix
            mFundamentalMatrix.normalize();
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    mFundamentalMatrix.getInternalMatrix());
            decomposer.decompose();
            
            double[] sigmas = decomposer.getSingularValues();
            Matrix u = decomposer.getU();
            Matrix v = decomposer.getV();
            
            double sigma1 = sigmas[0];
            double sigma2 = sigmas[1];
            
            //Column u1
            double u11 = u.getElementAt(0, 0);
            double u21 = u.getElementAt(1, 0);
            double u31 = u.getElementAt(2, 0);
            
            //Column u2
            double u12 = u.getElementAt(0, 1);
            double u22 = u.getElementAt(1, 1);
            double u32 = u.getElementAt(2, 1);
            
            //Column v1
            double v11 = v.getElementAt(0, 0);
            double v21 = v.getElementAt(1, 0);
            double v31 = v.getElementAt(2, 0);
            
            //Column v2
            double v12 = v.getElementAt(0, 1);
            double v22 = v.getElementAt(1, 1);
            double v32 = v.getElementAt(2, 1);
            
            //build Kruppa equations
            double A = u12*u11;
            double B = u22*u21;
            double C = Math.pow(x0,2.0)*u12*u11 + x0*y0*u22*u11 + x0*u32*u11 + 
                    x0*y0*u12*u21 + Math.pow(y0,2.0)*u22*u21 + y0*u32*u21 + 
                    x0*u12*u31 + y0*u22*u31 + u32*u31;
            double D = Math.pow(sigma2,2.0)*v12*v12;
            double E = Math.pow(sigma2,2.0)*v22*v22;
            double F = Math.pow(sigma2*x0,2.0)*v12*v12 + 
                    Math.pow(sigma2,2.0)*x0*y0*v22*v12 + 
                    Math.pow(sigma2,2.0)*x0*v32*v12 + 
                    Math.pow(sigma2,2.0)*x0*y0*v12*v22 + 
                    Math.pow(sigma2*y0,2)*v22*v22 + 
                    Math.pow(sigma2,2.0)*y0*v32*v22 + 
                    Math.pow(sigma2,2.0)*x0*v12*v32 + 
                    Math.pow(sigma2,2.0)*y0*v22*v32 + 
                    Math.pow(sigma2,2.0)*v32*v32;
            double G = u11*u11;
            double H = u21*u21;
            double I = Math.pow(x0,2.0)*u11*u11 + x0*y0*u21*u11 + x0*u31*u11 + 
                    x0*y0*u11*u21 + Math.pow(y0,2.0)*u21*u21 + y0*u31*u21 + 
                    x0*u11*u31 + y0*u21*u31 + u31*u31;
            double J = sigma1*sigma2*v12*v11;
            double K = sigma1*sigma2*v22*v21;
            double L = sigma1*sigma2*Math.pow(x0,2.0)*v12*v11 + 
                    sigma1*sigma2*x0*y0*v22*v11 + sigma1*sigma2*x0*v32*v11 + 
                    sigma1*sigma2*x0*y0*v12*v21 + 
                    sigma1*sigma2*Math.pow(y0,2.0)*v22*v21 + 
                    sigma1*sigma2*y0*v32*v21 + sigma1*sigma2*x0*v12*v31 + 
                    sigma1*sigma2*y0*v22*v31 + sigma1*sigma2*v32*v31;
            double M = Math.pow(sigma1,2.0)*v11*v11;
            double N = Math.pow(sigma1,2.0)*v21*v21;
            double O = Math.pow(sigma1*x0,2.0)*v11*v11 + 
                    Math.pow(sigma1,2.0)*x0*y0*v21*v11 + 
                    Math.pow(sigma1,2.0)*x0*v31*v11 + 
                    Math.pow(sigma1,2.0)*x0*y0*v11*v21 + 
                    Math.pow(sigma1*y0,2.0)*v21*v21 + 
                    Math.pow(sigma1,2.0)*y0*v31*v21 + 
                    Math.pow(sigma1,2.0)*x0*v11*v31 + 
                    Math.pow(sigma1,2.0)*y0*v21*v31 + 
                    Math.pow(sigma1,2.0)*v31*v31;
            double P = u12*u12;
            double Q = u22*u22;
            double R = Math.pow(x0,2.0)*u12*u12 + x0*y0*u22*u12 + x0*u32*u12 + 
                    x0*y0*u12*u22 + Math.pow(y0,2.0)*u22*u22 + y0*u32*u22 + 
                    x0*u12*u32 + y0*u22*u32 + u32*u32;

            //try to solve any of Kruppa's equations
            Complex[] roots;
            try {
                Polynomial poly3 = buildPolynomial3(A, B, C, D, E, F, G, H, I, 
                        J, K, L);
                roots = poly3.getRoots();
            } catch (NumericalException e3) {
                try {
                    //if solution for poly3 fails, try with 4th polynomial
                    Polynomial poly4 = buildPolynomial4(D, E, F, G, H,
                            I, M, N, O, P, Q, R);
                    roots = poly4.getRoots();
                } catch (NumericalException e4) {
                    Polynomial poly5 = buildPolynomial5(A, B, C,
                            J, K, L, M, N, O, P, Q, R);
                    roots = poly5.getRoots();
                }
            }
            
            //roots contain possible x values. We use only their real part
            
            //pick the best x, y values that produce a positive definite DIAC
            //matrix
            DualImageOfAbsoluteConic diac = new DualImageOfAbsoluteConic(
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            boolean valid = false;
            if (roots != null) {
                double r = mFocalDistanceAspectRatio;
                double r2 = r*r;
                for (Complex root : roots) {
                    double x = root.getReal();
                    double y = r2*x;
                    
                    //build DIAC matrix and check if it is positive definite
                    if (x >= 0.0 && y >= 0.0) {
                        double horizontalFocalLength = Math.sqrt(x);
                        double verticalFocalLength = Math.sqrt(y);
                        try {
                            valid = buildDiac(horizontalFocalLength, 
                                verticalFocalLength, diac);
                        } catch (AlgebraException e) {
                            valid = false;
                        }
                    }
                    
                    if (valid) {
                        break;
                    }
                }
            } else {
                throw new KruppaDualImageOfAbsoluteConicEstimatorException();
            }
            
            if (valid) {
                //copy to result
                result.setParameters(diac.getA(), diac.getB(), diac.getC(), 
                        diac.getD(), diac.getE(), diac.getF());
            } else {
                //no valid DIAC could be found
                throw new KruppaDualImageOfAbsoluteConicEstimatorException();
            }
            
        } catch (KruppaDualImageOfAbsoluteConicEstimatorException e) {
            throw e;
        } catch (Exception e) {
            throw new KruppaDualImageOfAbsoluteConicEstimatorException(e);
        }                
    }
    
    /**
     * Another of Kruppa's equations expressed as a polynomial of degree 2 to 
     * solve x value, which is the squared value of horizontal focal length.
     * This method is only used when aspect ratio is known.
     * @param A internal value from Kruppa's equations.
     * @param B internal value from Kruppa's equations.
     * @param C internal value from Kruppa's equations.
     * @param D internal value from Kruppa's equations.
     * @param E internal value from Kruppa's equations.
     * @param F internal value from Kruppa's equations.
     * @param G internal value from Kruppa's equations.
     * @param H internal value from Kruppa's equations.
     * @param I internal value from Kruppa's equations.
     * @param J internal value from Kruppa's equations.
     * @param K internal value from Kruppa's equations.
     * @param L internal value from Kruppa's equations.
     * @return a polynomial.
     */
    private Polynomial buildPolynomial3(double A, double B, double C, double D,
            double E, double F, double G, double H, double I, double J, 
            double K, double L) {
        
        //x^2*((-A*D - G*J) + r^4*(-B*E - H*K) + r^2*(-A*E - B*D - G*K - H*J)) + 
        //x*((-A*F - D*C - G*L - J*I) + r^2*(-B*F - E*C - H*L - K*I)) + 
        //(- F*C - L*I) = 0
        double r = mFocalDistanceAspectRatio;
        double r2 = r*r;
        double r4 = r2*r2;
        return new Polynomial(-F*C -L*I, 
                (-A*F - D*C - G*L - J*I) + r2*(-B*F - E*C - H*L - K*I), 
                ((-A*D - G*J) + r4*(-B*E - H*K) + r2*(-A*E - B*D - G*K - H*J)));
    }

    /**
     * Another of Kruppa's equations expressed as a polynomial of degree 2 to 
     * solve x value, which is the squared value of horizontal focal length.
     * This method is only used when aspect ratio is known.
     * @param D internal value from Kruppa's equations.
     * @param E internal value from Kruppa's equations.
     * @param F internal value from Kruppa's equations.
     * @param G internal value from Kruppa's equations.
     * @param H internal value from Kruppa's equations.
     * @param I internal value from Kruppa's equations.
     * @param M internal value from Kruppa's equations.
     * @param N internal value from Kruppa's equations.
     * @param O internal value from Kruppa's equations.
     * @param P internal value from Kruppa's equations.
     * @param Q internal value from Kruppa's equations.
     * @param R internal value from Kruppa's equations.
     * @return a polynomial.
     */
    private Polynomial buildPolynomial4(double D,
            double E, double F, double G, double H, double I,
            double M, double N, double O, double P,
            double Q, double R) {
        
        //x^2*((G*M - P*D) + r^4*(H*N - Q*E) + r^2*(G*N + H*M - P*E - Q*D)) + 
        //x*((G*O + M*I - P*F - D*R) + r^2*(H*O + N*I - Q*F - E*R)) + 
        //(O*I - F*R) = 0
        double r = mFocalDistanceAspectRatio;
        double r2 = r*r;
        double r4 = r2*r2;
        return new Polynomial(O*I - F*R, 
                (G*O + M*I - P*F - D*R) + r2*(H*O + N*I - Q*F - E*R),
                (G*M - P*D) + r4*(H*N - Q*E) + r2*(G*N + H*M - P*E - Q*D));
    }

    /**
     * Another of Kruppa's equations expressed as a polynomial of degree 2 to 
     * solve x value, which is the squared value of horizontal focal length.
     * This method is only used when aspect ratio is known.
     * @param A internal value from Kruppa's equations.
     * @param B internal value from Kruppa's equations.
     * @param C internal value from Kruppa's equations.
     * @param J internal value from Kruppa's equations.
     * @param K internal value from Kruppa's equations.
     * @param L internal value from Kruppa's equations.
     * @param M internal value from Kruppa's equations.
     * @param N internal value from Kruppa's equations.
     * @param O internal value from Kruppa's equations.
     * @param P internal value from Kruppa's equations.
     * @param Q internal value from Kruppa's equations.
     * @param R internal value from Kruppa's equations.
     * @return a polynomial.
     */
    private Polynomial buildPolynomial5(double A, double B, double C, double J,
            double K, double L, double M, double N, double O, double P, 
            double Q, double R) {
        
        //x^2*((P*J + A*M) + r^4*(Q*K + B*N) + r^2*(P*K + Q*J + A*N + B*M)) + 
        //x*((P*L + J*R + A*O + M*C) + r^2*(Q*L + K*R + B*O + N*C)) + 
        //(L*R + O*C) = 0
        double r = mFocalDistanceAspectRatio;
        double r2 = r*r;
        double r4 = r2*r2;
        return new Polynomial(L*R + O*C, 
                (P*L + J*R + A*O + M*C) + r2*(Q*L + K*R + B*O + N*C),
                (P*J + A*M) + r4*(Q*K + B*N) + r2*(P*K + Q*J + A*N + B*M));
    }
    
}
