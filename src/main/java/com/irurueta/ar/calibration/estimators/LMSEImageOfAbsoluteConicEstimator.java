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
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.ar.calibration.ImageOfAbsoluteConic;
import com.irurueta.geometry.ProjectiveTransformation2D;
import com.irurueta.geometry.Transformation2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.util.List;

/**
 * This class defines an LMSE (Least Mean Square Error) estimator of Image
 * of Absolute Conic (IAC).
 * Aside from enabling constraints whenever possible to obtain more stable and
 * accurate results, it is also discouraged to enable LMSE solutions, or at 
 * least if LMSE must be used, the minimum possible number of homographies 
 * should be provided in order to introduce the least amount of rounding errors 
 * possible.
 * If a large number of homographies is available (assuming constant IAC), 
 * instead a robust estimation method should be chosen to discard outliers and 
 * obtain the most accurate and stable solution possible.
 */
public class LMSEImageOfAbsoluteConicEstimator extends 
        ImageOfAbsoluteConicEstimator {
    
    /**
     * Indicates if by default an LMSE (Least Mean Square Error) solution is
     * allowed if more homographies than the minimum are provided.
     */
    public static final boolean DEFAULT_ALLOW_LMSE_SOLUTION = false;
            
    /**
     * Indicates if an LMSE (Least Mean Square Error) solution is allowed if
     * more homographies than the minimum are provided. If false, the 
     * exceeding homographies will be ignored and only the first required
     * homographies will be used.
     */
    private boolean mAllowLMSESolution;

    /**
     * Constructor.
     */
    public LMSEImageOfAbsoluteConicEstimator() {
        super();
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public LMSEImageOfAbsoluteConicEstimator(
            ImageOfAbsoluteConicEstimatorListener listener) {
        super(listener);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Constructor
     * @param homographies list of homographies (2D transformations) used to
     * estimate the image of absolute conic (IAC), which can be used to obtain
     * pinhole camera intrinsic parameters.
     * @throws IllegalArgumentException if not enough homographies are provided
     * for default IAC estimation constraints.
     */
    public LMSEImageOfAbsoluteConicEstimator(
            List<Transformation2D> homographies)
            throws IllegalArgumentException {
        super(homographies);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Constructor
     * @param homographies list of homographies (2D transformations) used to
     * estimate the image of absolute conic (IAC), which can be used to obtain
     * pinhole camera intrinsic parameters.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if not enough homographies are provided
     * for default IAC estimation constraints.
     */
    public LMSEImageOfAbsoluteConicEstimator(
            List<Transformation2D> homographies,
            ImageOfAbsoluteConicEstimatorListener listener) 
            throws IllegalArgumentException {
        super(homographies, listener);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Indicates if an LMSE (Least Mean Square Error) solution is allowed if 
     * more homographies than the minimum are provided. If false, the 
     * exceeding homographies will be ignored and only the first required
     * correspondences will be used.
     * @return true if LMSE solution is allowed, false otherwise.
     */
    public boolean isLMSESolutionAllowed() {
        return mAllowLMSESolution;
    }
    
    /**
     * Specifies if an LMSE (Least Mean Square Error) solution is allowed if
     * more homographies than the minimum are provided. If false, the
     * exceeding homographies will be ignored and only the first required ones
     * will be used.
     * @param allowed true if LMSE solution is allowed, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setLMSESolutionAllowed(boolean allowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mAllowLMSESolution = allowed;
    }        

    /**
     * Estimates Image of Absolute Conic (IAC).
     * @return estimated IAC.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if input has not yet been provided.
     * @throws ImageOfAbsoluteConicEstimatorException if an error occurs during
     * estimation, usually because repeated homographies are provided, or 
     * homographies corresponding to degenerate camera movements such as pure
     * parallel translations where no additional data is really provided. 
     * Indeed, if provided homographies belong to the group of affine 
     * transformations (or other groups contained within such as metric or 
     * euclidean ones), this exception will raise because camera movements will
     * be degenerate. To avoid this exception, homographies must be purely
     * projective.
     */    
    @Override
    public ImageOfAbsoluteConic estimate() throws LockedException, 
            NotReadyException, ImageOfAbsoluteConicEstimatorException {
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
            
            ImageOfAbsoluteConic iac;
            if (mZeroSkewness && mPrincipalPointAtOrigin) {
                if (mFocalDistanceAspectRatioKnown) {
                    iac = estimateZeroSkewnessPrincipalPointAtOriginAndKnownFocalDistanceAspectRatio();
                } else {
                    iac = estimateZeroSkewnessAndPrincipalPointAtOrigin();
                }
            } else if (mZeroSkewness) { //&& !mPrincipalPointAtOrigin
                if (mFocalDistanceAspectRatioKnown) {
                    iac = estimateZeroSkewnessAndKnownFocalDistanceAspectRatio();
                } else {
                    iac = estimateZeroSkewness();
                }
            }else if (mPrincipalPointAtOrigin) { //&& !mZeroSkewness
                iac = estimatePrincipalPointAtOrigin();
            } else {
                iac = estimateNoConstraints();
            }
            
            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
            
            return iac;
        } finally {
            mLocked = false;
        }
    }

    /**
     * Returns type of IAC estimator.
     * @return type of IAC estimator.
     */    
    @Override
    public ImageOfAbsoluteConicEstimatorType getType() {
        return ImageOfAbsoluteConicEstimatorType.LMSE_IAC_ESTIMATOR;
    }
    
    /**
     * Estimates Image of Absolute Conic (IAC) without contraints.
     * @return estimated IAC.
     * @throws ImageOfAbsoluteConicEstimatorException if an error occurs during
     * estimation, usually because repeated homographies are provided, or 
     * homographies corresponding to degenerate camera movements such as pure
     * parallel translations where no additional data is really provided.
     */
    private ImageOfAbsoluteConic estimateNoConstraints() 
            throws ImageOfAbsoluteConicEstimatorException {
        
        try {
            int nHomographies = mHomographies.size();

            Matrix a;
            if (isLMSESolutionAllowed()) {
                //initialize new matrix to zero when LMSE is enabled
                a = new Matrix(2 * nHomographies, 6);
            } else {
                //When LMSE is disabled, initialize new matrix to zero only with
                //5 equations
                a = new Matrix(MIN_REQUIRED_EQUATIONS, 6);
            }
            
            int counter = 0;
            ProjectiveTransformation2D t = null;
            Matrix h = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 
                    ProjectiveTransformation2D.HOM_COORDS);
            //elements ij of homography (last column is not required)
            double h11, h12, h21, h22, h31, h32, rowNorm;
            for (Transformation2D homography : mHomographies) {
                //convert homography into projective so it can be normalized
                homography.asMatrix(h);
                if (t == null) {
                    t = new ProjectiveTransformation2D(h);
                } else {
                    t.setT(h);
                }
                
                //normalize
                t.normalize();
                
                //obtain elements of projective transformation matrix
                //there is no need to retrieve internal matrix h, as we already
                //hold a reference
                h11 = h.getElementAt(0, 0);
                h12 = h.getElementAt(0, 1);
                
                h21 = h.getElementAt(1, 0);
                h22 = h.getElementAt(1, 1);
                
                h31 = h.getElementAt(2, 0);
                h32 = h.getElementAt(2, 1);
                
                //fill first equation
                a.setElementAt(counter, 0, h11 * h12);
                a.setElementAt(counter, 1, h11 * h22 + h21 * h12);
                a.setElementAt(counter, 2, h21 * h22);
                a.setElementAt(counter, 3, h11 * h32 + h31 * h12);
                a.setElementAt(counter, 4, h21 * h32 + h31 * h22);
                a.setElementAt(counter, 5, h31 * h32);
                
                //normalize row 
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 0), 2.0) +
                        Math.pow(a.getElementAt(counter, 1), 2.0) +
                        Math.pow(a.getElementAt(counter, 2), 2.0) +
                        Math.pow(a.getElementAt(counter, 3), 2.0) +
                        Math.pow(a.getElementAt(counter, 4), 2.0) +
                        Math.pow(a.getElementAt(counter, 5), 2.0));
                
                a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                        rowNorm);
                a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                        rowNorm);
                a.setElementAt(counter, 2, a.getElementAt(counter, 2) / 
                        rowNorm);
                a.setElementAt(counter, 3, a.getElementAt(counter, 3) / 
                        rowNorm);
                a.setElementAt(counter, 4, a.getElementAt(counter, 4) / 
                        rowNorm);
                a.setElementAt(counter, 5, a.getElementAt(counter, 5) / 
                        rowNorm);

                counter++;
                
                //in case we want an exact solution (up to scale) when LMSE is
                //disabled, we stop after 5 equations
                if (!isLMSESolutionAllowed() && (counter >= MIN_REQUIRED_EQUATIONS)) {
                    break;
                }
                
                //fill second equation
                a.setElementAt(counter, 0, Math.pow(h11, 2.0) - Math.pow(h12, 2.0));
                a.setElementAt(counter, 1, 2.0 * (h11 * h21 - h12 * h22));
                a.setElementAt(counter, 2, Math.pow(h21, 2.0) - Math.pow(h22, 2.0));
                a.setElementAt(counter, 3, 2.0 * (h11 * h31 - h12 * h32));
                a.setElementAt(counter, 4, 2.0 * (h21 * h31 - h22 * h32));
                a.setElementAt(counter, 5, Math.pow(h31, 2.0) - Math.pow(h32, 2.0));
                
                //normalize row
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 0), 2.0) +
                        Math.pow(a.getElementAt(counter, 1), 2.0) +
                        Math.pow(a.getElementAt(counter, 2), 2.0) +
                        Math.pow(a.getElementAt(counter, 3), 2.0) +
                        Math.pow(a.getElementAt(counter, 4), 2.0) +
                        Math.pow(a.getElementAt(counter, 5), 2.0));

                a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                        rowNorm);
                a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                        rowNorm);
                a.setElementAt(counter, 2, a.getElementAt(counter, 2) / 
                        rowNorm);
                a.setElementAt(counter, 3, a.getElementAt(counter, 3) / 
                        rowNorm);
                a.setElementAt(counter, 4, a.getElementAt(counter, 4) / 
                        rowNorm);
                a.setElementAt(counter, 5, a.getElementAt(counter, 5) / 
                        rowNorm);
                
                counter++;
            }
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();
            
            if (decomposer.getNullity() > 1) {
                //homographies constitute a degenerate camera movement.
                //A linear combination of possible IAC's exist (i.e. solution is
                //not unique up to scale)
                throw new ImageOfAbsoluteConicEstimatorException();
            }
            
            Matrix v = decomposer.getV();
            
            //use last column of V as IAC vector
            
            //the last column of V contains IAC matrix (B), which is symmetric
            //and positive definite, ordered as follows: B11, B12, B22, B13, 
            //B23, B33
            double b11 = v.getElementAt(0, 5);
            double b12 = v.getElementAt(1, 5);
            double b22 = v.getElementAt(2, 5);
            double b13 = v.getElementAt(3, 5);
            double b23 = v.getElementAt(4, 5);
            double b33 = v.getElementAt(5, 5);
                        
            //A conic is defined as [A  B   D]
            //                      [B  C   E]
            //                      [D  E   F]
            return new ImageOfAbsoluteConic(b11, b12, b22, b13, b23, b33);            
        } catch (AlgebraException e) {
            throw new ImageOfAbsoluteConicEstimatorException(e);
        }
    }
        
    /**
     * Estimates Image of Absolute Conic (IAC) assuming that skewness is zero.
     * @return estimated IAC
     * @throws ImageOfAbsoluteConicEstimatorException if an error occurs during
     * estimation, usually because repeated homographies are provided, or 
     * homographies corresponding to degenerate camera movements such as pure
     * parallel translations where no additional data is really provided
     */    
    private ImageOfAbsoluteConic estimateZeroSkewness() 
            throws ImageOfAbsoluteConicEstimatorException {
        try {
            int nHomographies = mHomographies.size();

            Matrix a;
            if (isLMSESolutionAllowed()) {
                //initialize new matrix to zero when LMSE is enabled
                a = new Matrix(2 * nHomographies, 5);
            } else {
                //When LMSE is disabled, initialize new matrix to zero only with
                //4 equations
                a = new Matrix(MIN_REQUIRED_EQUATIONS - 1, 5);
            }
            
            int counter = 0;
            ProjectiveTransformation2D t = null;
            Matrix h = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 
                    ProjectiveTransformation2D.HOM_COORDS);
            //elements ij of homography (last column is not required)
            double h11, h12, h21, h22, h31, h32, rowNorm;
            for (Transformation2D homography : mHomographies) {
                //convert homography into projective so it can be normalized
                homography.asMatrix(h);
                if (t == null) {
                    t = new ProjectiveTransformation2D(h);
                } else {
                    t.setT(h);
                }
                
                //normalize
                t.normalize();
                
                //obtain elements of projective transformation matrix
                //there is no need to retrieve internal matrix h, as we already
                //hold a reference
                h11 = h.getElementAt(0, 0);
                h12 = h.getElementAt(0, 1);
                
                h21 = h.getElementAt(1, 0);
                h22 = h.getElementAt(1, 1);
                
                h31 = h.getElementAt(2, 0);
                h32 = h.getElementAt(2, 1);
                
                //fill first equation
                a.setElementAt(counter, 0, h11 * h12);
                a.setElementAt(counter, 1, h21 * h22);
                a.setElementAt(counter, 2, h11 * h32 + h31 * h12);
                a.setElementAt(counter, 3, h21 * h32 + h31 * h22);
                a.setElementAt(counter, 4, h31 * h32);
                
                //normalize row 
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 0), 2.0) +
                        Math.pow(a.getElementAt(counter, 1), 2.0) +
                        Math.pow(a.getElementAt(counter, 2), 2.0) +
                        Math.pow(a.getElementAt(counter, 3), 2.0) +
                        Math.pow(a.getElementAt(counter, 4), 2.0));
                
                a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                        rowNorm);
                a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                        rowNorm);
                a.setElementAt(counter, 2, a.getElementAt(counter, 2) / 
                        rowNorm);
                a.setElementAt(counter, 3, a.getElementAt(counter, 3) / 
                        rowNorm);
                a.setElementAt(counter, 4, a.getElementAt(counter, 4) / 
                        rowNorm);

                counter++;
                                
                //fill second equation
                a.setElementAt(counter, 0, Math.pow(h11, 2.0) - Math.pow(h12, 2.0));
                a.setElementAt(counter, 1, Math.pow(h21, 2.0) - Math.pow(h22, 2.0));
                a.setElementAt(counter, 2, 2.0 * (h11 * h31 - h12 * h32));
                a.setElementAt(counter, 3, 2.0 * (h21 * h31 - h22 * h32));
                a.setElementAt(counter, 4, Math.pow(h31, 2.0) - Math.pow(h32, 2.0));
                
                //normalize row
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 0), 2.0) +
                        Math.pow(a.getElementAt(counter, 1), 2.0) +
                        Math.pow(a.getElementAt(counter, 2), 2.0) +
                        Math.pow(a.getElementAt(counter, 3), 2.0) +
                        Math.pow(a.getElementAt(counter, 4), 2.0));

                a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                        rowNorm);
                a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                        rowNorm);
                a.setElementAt(counter, 2, a.getElementAt(counter, 2) / 
                        rowNorm);
                a.setElementAt(counter, 3, a.getElementAt(counter, 3) / 
                        rowNorm);
                a.setElementAt(counter, 4, a.getElementAt(counter, 4) / 
                        rowNorm);
                
                counter++;
                
                //in case we want an exact solution (up to scale) when LMSE is
                //disabled, we stop after 4 equations
                if (!isLMSESolutionAllowed() && (counter >= (MIN_REQUIRED_EQUATIONS - 1))) {
                    break;
                }
            }
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();
            
            if (decomposer.getNullity() > 1) {
                //homographies constitute a degenerate camera movement.
                //A linear combination of possible IAC's exist (i.e. solution is
                //not unique up to scale)
                throw new ImageOfAbsoluteConicEstimatorException();
            }
            
            Matrix v = decomposer.getV();
            
            //use last column of V as IAC vector
            
            //the last column of V contains IAC matrix (B), which is symmetric
            //and positive definite, ordered as follows: B11, B12, B22, B13, 
            //B23, B33
            double b11 = v.getElementAt(0, 4);
            double b22 = v.getElementAt(1, 4);
            double b13 = v.getElementAt(2, 4);
            double b23 = v.getElementAt(3, 4);
            double b33 = v.getElementAt(4, 4);
                        
            //A conic is defined as [A  B   D]
            //                      [B  C   E]
            //                      [D  E   F]
            //Since skewness is zero b12 = B = 0.0
            return new ImageOfAbsoluteConic(b11, 0.0, b22, b13, b23, b33);            
        } catch (AlgebraException e) {
            throw new ImageOfAbsoluteConicEstimatorException(e);
        }
    }
    
    /**
     * Estimates Image of Absolute Conic (IAC) assuming that principal point is
     * located at origin of coordinates.
     * @return estimated IAC
     * @throws ImageOfAbsoluteConicEstimatorException if an error occurs during
     * estimation, usually because repeated homographies are provided, or 
     * homographies corresponding to degenerate camera movements such as pure
     * parallel translations where no additional data is really provided
     */    
    private ImageOfAbsoluteConic estimatePrincipalPointAtOrigin() 
            throws ImageOfAbsoluteConicEstimatorException {
        
        try {
            int nHomographies = mHomographies.size();

            Matrix a;
            if (isLMSESolutionAllowed()) {
                //initialize new matrix to zero when LMSE is enabled
                a = new Matrix(2 * nHomographies, 4);
            } else {
                //When LMSE is disabled, initialize new matrix to zero only with
                //2 equations
                a = new Matrix(MIN_REQUIRED_EQUATIONS - 2, 4);
            }
            
            int counter = 0;
            ProjectiveTransformation2D t = null;
            Matrix h = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 
                    ProjectiveTransformation2D.HOM_COORDS);
            //elements ij of homography (last column is not required)
            double h11, h12, h21, h22, h31, h32, rowNorm;
            for (Transformation2D homography : mHomographies) {
                //convert homography into projective so it can be normalized
                homography.asMatrix(h);
                if (t == null) {
                    t = new ProjectiveTransformation2D(h);
                } else {
                    t.setT(h);
                }
                
                //normalize
                t.normalize();
                
                //obtain elements of projective transformation matrix
                //there is no need to retrieve internal matrix h, as we already
                //hold a reference
                h11 = h.getElementAt(0, 0);
                h12 = h.getElementAt(0, 1);
                
                h21 = h.getElementAt(1, 0);
                h22 = h.getElementAt(1, 1);
                
                h31 = h.getElementAt(2, 0);
                h32 = h.getElementAt(2, 1);
                                
                //fill first equation
                a.setElementAt(counter, 0, h11 * h12);
                a.setElementAt(counter, 1, h11 * h22 + h21 * h12);
                a.setElementAt(counter, 2, h21 * h22);
                a.setElementAt(counter, 3, h31 * h32);
                
                //normalize row 
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 0), 2.0) +
                        Math.pow(a.getElementAt(counter, 1), 2.0) +
                        Math.pow(a.getElementAt(counter, 2), 2.0) +
                        Math.pow(a.getElementAt(counter, 3), 2.0));
                
                a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                        rowNorm);
                a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                        rowNorm);
                a.setElementAt(counter, 2, a.getElementAt(counter, 2) / 
                        rowNorm);
                a.setElementAt(counter, 3, a.getElementAt(counter, 3) / 
                        rowNorm);

                counter++;
                
                //in case we want an exact solution (up to scale) when LMSE is
                //disabled, we stop after 2 equations
                if (!isLMSESolutionAllowed() && (counter >= MIN_REQUIRED_EQUATIONS - 2)) {
                    break;
                }
                                
                //fill second equation
                a.setElementAt(counter, 0, Math.pow(h11, 2.0) - Math.pow(h12, 2.0));
                a.setElementAt(counter, 1, 2.0 * (h11 * h21 - h12 * h22));
                a.setElementAt(counter, 2, Math.pow(h21, 2.0) - Math.pow(h22, 2.0));
                a.setElementAt(counter, 3, Math.pow(h31, 2.0) - Math.pow(h32, 2.0));
                
                //normalize row
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 0), 2.0) +
                        Math.pow(a.getElementAt(counter, 1), 2.0) +
                        Math.pow(a.getElementAt(counter, 2), 2.0) +
                        Math.pow(a.getElementAt(counter, 3), 2.0));

                a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                        rowNorm);
                a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                        rowNorm);
                a.setElementAt(counter, 2, a.getElementAt(counter, 2) / 
                        rowNorm);
                a.setElementAt(counter, 3, a.getElementAt(counter, 3) / 
                        rowNorm);
                
                counter++;                
            }
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();
            
            if (decomposer.getNullity() > 1) {
                //homographies constitute a degenerate camera movement.
                //A linear combination of possible IAC's exist (i.e. solution is
                //not unique up to scale)
                throw new ImageOfAbsoluteConicEstimatorException();
            }
            
            Matrix v = decomposer.getV();
            
            //use last column of V as IAC vector
            
            //the last column of V contains IAC matrix (B), which is symmetric
            //and positive definite, ordered as follows: B11, B12, B22, B13, 
            //B23, B33
            double b11 = v.getElementAt(0, 3);
            double b12 = v.getElementAt(1, 3);
            double b22 = v.getElementAt(2, 3);
            double b33 = v.getElementAt(3, 3);
                        
            //A conic is defined as [A  B   D]
            //                      [B  C   E]
            //                      [D  E   F]
            //Since principal point is at origin of coordinates 
            //b13 = D = 0.0, b23 = E = 0.0
            return new ImageOfAbsoluteConic(b11, b12, b22, 0.0, 0.0, b33);
        } catch (AlgebraException e) {
            throw new ImageOfAbsoluteConicEstimatorException(e);
        }
    }
    
    /**
     * Estimates Image of Absolute Conic (IAC) assuming that skewness is zero 
     * and that principal point is located at origin of coordinates.
     * @return estimated IAC
     * @throws ImageOfAbsoluteConicEstimatorException if an error occurs during
     * estimation, usually because repeated homographies are provided, or 
     * homographies corresponding to degenerate camera movements such as pure
     * parallel translations where no additional data is really provided
     */    
    private ImageOfAbsoluteConic estimateZeroSkewnessAndPrincipalPointAtOrigin() 
            throws ImageOfAbsoluteConicEstimatorException {

        try {
            int nHomographies = mHomographies.size();

            Matrix a;
            if (isLMSESolutionAllowed()) {
                //initialize new matrix to zero when LMSE is enabled
                a = new Matrix(2 * nHomographies, 3);
            } else {
                //When LMSE is disabled, initialize new matrix to zero only with
                //2 equations
                a = new Matrix(MIN_REQUIRED_EQUATIONS - 3, 3);
            }
            
            int counter = 0;
            ProjectiveTransformation2D t = null;
            Matrix h = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 
                    ProjectiveTransformation2D.HOM_COORDS);
            //elements ij of homography (last column is not required)
            double h11, h12, h21, h22, h31, h32, rowNorm;
            for (Transformation2D homography : mHomographies) {
                //convert homography into projective so it can be normalized
                homography.asMatrix(h);
                if (t == null) {
                    t = new ProjectiveTransformation2D(h);
                } else {
                    t.setT(h);
                }
                
                //normalize
                t.normalize();
                
                //obtain elements of projective transformation matrix
                //there is no need to retrieve internal matrix h, as we already
                //hold a reference
                h11 = h.getElementAt(0, 0);
                h12 = h.getElementAt(0, 1);
                
                h21 = h.getElementAt(1, 0);
                h22 = h.getElementAt(1, 1);
                
                h31 = h.getElementAt(2, 0);
                h32 = h.getElementAt(2, 1);
                                
                //fill first equation
                a.setElementAt(counter, 0, h11 * h12);
                a.setElementAt(counter, 1, h21 * h22);
                a.setElementAt(counter, 2, h31 * h32);
                
                //normalize row 
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 0), 2.0) +
                        Math.pow(a.getElementAt(counter, 1), 2.0) +
                        Math.pow(a.getElementAt(counter, 2), 2.0));
                
                a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                        rowNorm);
                a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                        rowNorm);
                a.setElementAt(counter, 2, a.getElementAt(counter, 2) / 
                        rowNorm);

                counter++;
                                                
                //fill second equation
                a.setElementAt(counter, 0, Math.pow(h11, 2.0) - Math.pow(h12, 2.0));
                a.setElementAt(counter, 1, Math.pow(h21, 2.0) - Math.pow(h22, 2.0));
                a.setElementAt(counter, 2, Math.pow(h31, 2.0) - Math.pow(h32, 2.0));
                
                //normalize row
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 0), 2.0) +
                        Math.pow(a.getElementAt(counter, 1), 2.0) +
                        Math.pow(a.getElementAt(counter, 2), 2.0));

                a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                        rowNorm);
                a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                        rowNorm);
                a.setElementAt(counter, 2, a.getElementAt(counter, 2) / 
                        rowNorm);
                
                counter++;   
                
                //in case we want an exact solution (up to scale) when LMSE is
                //disabled, we stop after 2 equations
                if (!isLMSESolutionAllowed() && (counter >= MIN_REQUIRED_EQUATIONS - 3)) {
                    break;
                }
            }
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();
            
            if (decomposer.getNullity() > 1) {
                //homographies constitute a degenerate camera movement.
                //A linear combination of possible IAC's exist (i.e. solution is
                //not unique up to scale)
                throw new ImageOfAbsoluteConicEstimatorException();
            }
            
            Matrix v = decomposer.getV();
            
            //use last column of V as IAC vector
            
            //the last column of V contains IAC matrix (B), which is symmetric
            //and positive definite, ordered as follows: B11, B12, B22, B13, 
            //B23, B33
            double b11 = v.getElementAt(0, 2);
            double b22 = v.getElementAt(1, 2);
            double b33 = v.getElementAt(2, 2);
                        
            //A conic is defined as [A  B   D]
            //                      [B  C   E]
            //                      [D  E   F]
            //Since principal point is at origin of coordinates 
            //b12 = B = 0, b13 = D = 0.0, b23 = E = 0.0
            return new ImageOfAbsoluteConic(b11, 0.0, b22, 0.0, 0.0, b33);
        } catch (AlgebraException e) {
            throw new ImageOfAbsoluteConicEstimatorException(e);
        }        
    }    
    
    /**
     * Estimates Image of Absolute Conic (IAC) assuming that skewness is zero
     * and that aspect ratio of focal distances is known.
     * @return estimated IAC
     * @throws ImageOfAbsoluteConicEstimatorException if an error occurs during
     * estimation, usually because repeated homographies are provided, or 
     * homographies corresponding to degenerate camera movements such as pure
     * parallel translations where no additional data is really provided
     */    
    private ImageOfAbsoluteConic 
            estimateZeroSkewnessAndKnownFocalDistanceAspectRatio() 
            throws ImageOfAbsoluteConicEstimatorException {
        try {
            int nHomographies = mHomographies.size();

            Matrix a;
            if (isLMSESolutionAllowed()) {
                //initialize new matrix to zero when LMSE is enabled
                a = new Matrix(2 * nHomographies, 4);
            } else {
                //When LMSE is disabled, initialize new matrix to zero only with
                //4 equations
                a = new Matrix(MIN_REQUIRED_EQUATIONS - 2, 4);
            }
            
            double sqrAspectRatio = Math.pow(mFocalDistanceAspectRatio, 2.0);
            
            int counter = 0;
            ProjectiveTransformation2D t = null;
            Matrix h = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 
                    ProjectiveTransformation2D.HOM_COORDS);
            //elements ij of homography (last column is not required)
            double h11, h12, h21, h22, h31, h32, rowNorm;
            for (Transformation2D homography : mHomographies) {
                //convert homography into projective so it can be normalized
                homography.asMatrix(h);
                if (t == null) {
                    t = new ProjectiveTransformation2D(h);
                } else {
                    t.setT(h);
                }
                
                //normalize
                t.normalize();
                
                //obtain elements of projective transformation matrix
                //there is no need to retrieve internal matrix h, as we already
                //hold a reference
                h11 = h.getElementAt(0, 0);
                h12 = h.getElementAt(0, 1);
                
                h21 = h.getElementAt(1, 0);
                h22 = h.getElementAt(1, 1);
                
                h31 = h.getElementAt(2, 0);
                h32 = h.getElementAt(2, 1);
                
                //fill first equation
                a.setElementAt(counter, 0, h11 * h12 + 
                        h21 * h22 / sqrAspectRatio);
                a.setElementAt(counter, 1, h11 * h32 + h31 * h12);
                a.setElementAt(counter, 2, h21 * h32 + h31 * h22);
                a.setElementAt(counter, 3, h31 * h32);
                
                //normalize row 
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 0), 2.0) +
                        Math.pow(a.getElementAt(counter, 1), 2.0) +
                        Math.pow(a.getElementAt(counter, 2), 2.0) +
                        Math.pow(a.getElementAt(counter, 3), 2.0));
                
                a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                        rowNorm);
                a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                        rowNorm);
                a.setElementAt(counter, 2, a.getElementAt(counter, 2) / 
                        rowNorm);
                a.setElementAt(counter, 3, a.getElementAt(counter, 3) / 
                        rowNorm);

                counter++;
                
                //in case we want an exact solution (up to scale) when LMSE is
                //disabled, we stop after 4 equations
                if (!isLMSESolutionAllowed() && (counter >= (MIN_REQUIRED_EQUATIONS - 2))) {
                    break;
                }
                                
                //fill second equation
                a.setElementAt(counter, 0, Math.pow(h11, 2.0) - Math.pow(h12, 2.0) +
                        (Math.pow(h21, 2.0) - Math.pow(h22, 2.0))/ sqrAspectRatio);
                a.setElementAt(counter, 1, 2.0 * (h11 * h31 - h12 * h32));
                a.setElementAt(counter, 2, 2.0 * (h21 * h31 - h22 * h32));
                a.setElementAt(counter, 3, Math.pow(h31, 2.0) - Math.pow(h32, 2.0));
                
                //normalize row
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 0), 2.0) +
                        Math.pow(a.getElementAt(counter, 1), 2.0) +
                        Math.pow(a.getElementAt(counter, 2), 2.0) +
                        Math.pow(a.getElementAt(counter, 3), 2.0));

                a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                        rowNorm);
                a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                        rowNorm);
                a.setElementAt(counter, 2, a.getElementAt(counter, 2) / 
                        rowNorm);
                a.setElementAt(counter, 3, a.getElementAt(counter, 3) / 
                        rowNorm);
                
                counter++;                
            }
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();
            
            if (decomposer.getNullity() > 1) {
                //homographies constitute a degenerate camera movement.
                //A linear combination of possible IAC's exist (i.e. solution is
                //not unique up to scale)
                throw new ImageOfAbsoluteConicEstimatorException();
            }
            
            Matrix v = decomposer.getV();
            
            //use last column of V as IAC vector
            
            //the last column of V contains IAC matrix (B), which is symmetric
            //and positive definite, ordered as follows: B11, B12, B22, B13, 
            //B23, B33
            double b11 = v.getElementAt(0, 3);
            double b13 = v.getElementAt(1, 3);
            double b23 = v.getElementAt(2, 3);
            double b33 = v.getElementAt(3, 3);
            
            double b22 = b11 / sqrAspectRatio;  
                        
            //A conic is defined as [A  B   D]
            //                      [B  C   E]
            //                      [D  E   F]
            //Since skewness is zero b12 = B = 0.0
            return new ImageOfAbsoluteConic(b11, 0.0, b22, b13, b23, b33);            
        } catch(AlgebraException e) {
            throw new ImageOfAbsoluteConicEstimatorException(e);
        }
    }
    
    /**
     * Estimates Image of Absolute Conic (IAC) assuming that skewness is zero, 
     * principal point is located at origin of coordinates and that aspect ratio
     * of focal distances is known.
     * @return estimated IAC
     * @throws ImageOfAbsoluteConicEstimatorException if an error occurs during
     * estimation, usually because repeated homographies are provided, or 
     * homographies corresponding to degenerate camera movements such as pure
     * parallel translations where no additional data is really provided
     */    
    private ImageOfAbsoluteConic 
            estimateZeroSkewnessPrincipalPointAtOriginAndKnownFocalDistanceAspectRatio() 
            throws ImageOfAbsoluteConicEstimatorException {

        try {
            double sqrAspectRatio = Math.pow(mFocalDistanceAspectRatio, 2.0);
            
            double b11, b33;
            
            ProjectiveTransformation2D t = null;
            Matrix h = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 
                    ProjectiveTransformation2D.HOM_COORDS);
            double h11, h12, h21, h22, h31, h32;

            if (!isLMSESolutionAllowed()) {
                //NO LMSE
                
                //For a single homography we have two equations, but indeed we 
                //only need 1 to solve b11 because b33 is defined up to scale 
                //(i.e. b33 = 1.0)
                
                //Hence
                //b11 * (h11 * h12 + h21 * h22 / sqrAspectRatio) + b33 * h31 * h32 = 0
                //b11 = -b33 * h31 * h32 / (h11 * h12 + h21 * h22 / sqrAspectRatio)
                
                Transformation2D homography = mHomographies.get(0);
                
                //convert homography into projective so it can be normalized
                homography.asMatrix(h);
                t = new ProjectiveTransformation2D(h);

                //normalize
                t.normalize();
                
                //obtain elements of projective transformation matrix
                //there is no need to retrieve internal matrix h, as we already
                //hold a reference
                h11 = h.getElementAt(0, 0);
                h12 = h.getElementAt(0, 1);
                
                h21 = h.getElementAt(1, 0);
                h22 = h.getElementAt(1, 1);
                
                h31 = h.getElementAt(2, 0);
                h32 = h.getElementAt(2, 1);                
                
                b33 = 1.0;
                b11 = -h31 * h32 / (h11 * h12 + h21 * h22 / sqrAspectRatio);
            } else {
                int nHomographies = mHomographies.size();

                Matrix a = new Matrix(2 * nHomographies, 2);
            
                int counter = 0;
                //elements ij of homography (last column is not required)
                double rowNorm;
                for (Transformation2D homography : mHomographies) {
                    //convert homography into projective so it can be normalized
                    homography.asMatrix(h);
                    if (t == null) {
                        t = new ProjectiveTransformation2D(h);
                    } else {
                        t.setT(h);
                    }
                
                    //normalize
                    t.normalize();
                
                    //obtain elements of projective transformation matrix
                    //there is no need to retrieve internal matrix h, as we already
                    //hold a reference
                    h11 = h.getElementAt(0, 0);
                    h12 = h.getElementAt(0, 1);
                
                    h21 = h.getElementAt(1, 0);
                    h22 = h.getElementAt(1, 1);
                
                    h31 = h.getElementAt(2, 0);
                    h32 = h.getElementAt(2, 1);
                                
                    //fill first equation
                    a.setElementAt(counter, 0, h11 * h12 + 
                            h21 * h22 / sqrAspectRatio);
                    a.setElementAt(counter, 1, h31 * h32);
                
                    //normalize row 
                    rowNorm = Math.sqrt(
                            Math.pow(a.getElementAt(counter, 0), 2.0) +
                            Math.pow(a.getElementAt(counter, 1), 2.0));
                
                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                            rowNorm);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                            rowNorm);

                    counter++;
                                                
                    //fill second equation
                    a.setElementAt(counter, 0, Math.pow(h11, 2.0) - Math.pow(h12, 2.0) +
                            (Math.pow(h21, 2.0) - Math.pow(h22, 2.0)) / sqrAspectRatio);
                    a.setElementAt(counter, 1, Math.pow(h31, 2.0) - Math.pow(h32, 2.0));
                
                    //normalize row
                    rowNorm = Math.sqrt(
                            Math.pow(a.getElementAt(counter, 0), 2.0) +
                            Math.pow(a.getElementAt(counter, 1), 2.0));

                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                            rowNorm);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                            rowNorm);
                
                    counter++;                   
                }
            
                SingularValueDecomposer decomposer = 
                        new SingularValueDecomposer(a);
                decomposer.decompose();
            
                if (decomposer.getNullity() > 1) {
                    //homographies constitute a degenerate camera movement.
                    //A linear combination of possible IAC's exist (i.e. 
                    //solution is not unique up to scale)
                    throw new ImageOfAbsoluteConicEstimatorException();
                }
            
                Matrix v = decomposer.getV();
            
                //use last column of V as IAC vector
            
                //the last column of V contains IAC matrix (B), which is 
                //symmetric and positive definite, ordered as follows: B11, B12, 
                //B22, B13, B23, B33
                b11 = v.getElementAt(0, 1);
                b33 = v.getElementAt(1, 1);
            }
            
            double b22 = b11 / sqrAspectRatio;
                        
            //A conic is defined as [A  B   D]
            //                      [B  C   E]
            //                      [D  E   F]
            //Since principal point is at origin of coordinates 
            //b12 = B = 0, b13 = D = 0.0, b23 = E = 0.0
            return new ImageOfAbsoluteConic(b11, 0.0, b22, 0.0, 0.0, b33);
        } catch (AlgebraException e) {
            throw new ImageOfAbsoluteConicEstimatorException(e);
        }        
    }    
            
}
