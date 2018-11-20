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
import com.irurueta.numerical.robust.WeightSelection;
import com.irurueta.sorting.SortingException;

import java.util.List;

/**
 * This class implements an Image of Absolute Conic (IAC) estimator using a
 * weighted algorithm and correspondences.
 * Weights can be used so that homographies assumed to have a better quality
 * (i.e. more precisely estimated) are considered to be more relevant.
 * Aside from enabling constraints whenever possible to obtain more stable and
 * accurate results, its is discouraged to use a large number of homographies, 
 * even if they are correctly weighted, since as the number of homographies
 * increase so do the rounding errors.
 */
public class WeightedImageOfAbsoluteConicEstimator extends 
        ImageOfAbsoluteConicEstimator {
    
    /**
     * Default number of homographies to be weighted and taken into account.
     */
    public static final int DEFAULT_MAX_HOMOGRAPHIES = 50;
    
    /**
     * Indicates if weights are sorted by default so that largest weighted
     * correspondences are used first.
     */
    public static final boolean DEFAULT_SORT_WEIGHTS = true;
    
    /**
     * Maximum number of homographies (i.e. correspondences) to be weighted and
     * taken into account.
     */
    private int mMaxHomographies;
    
    /**
     * Indicates if weights are sorted by default so that largest weighted
     * correspondences are used first.
     */
    private boolean mSortWeights;
    
    /**
     * Array containing weights for all homographies.
     */
    private double[] mWeights;

    /**
     * Constructor.
     */
    public WeightedImageOfAbsoluteConicEstimator() {
        super();
        mMaxHomographies = DEFAULT_MAX_HOMOGRAPHIES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public WeightedImageOfAbsoluteConicEstimator(
            ImageOfAbsoluteConicEstimatorListener listener) {
        super(listener);
        mMaxHomographies = DEFAULT_MAX_HOMOGRAPHIES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
    }
    
    /**
     * Constructor.
     * @param homographies list of homographies (2D transformations) used to
     * estimate the image of absolute conic (IAC), which can be used to obtain
     * pinhole camera intrinsic parameters.
     * @param weights array containing a weight amount for each homography.
     * The larger the value of a weight, the most significant the correspondence
     * will be.
     * @throws IllegalArgumentException if not enough homographies are provided
     * for default constraints during IAC estimation.
     */
    public WeightedImageOfAbsoluteConicEstimator(
            List<Transformation2D> homographies, double[] weights)
            throws IllegalArgumentException {
        super();
        internalSetHomographiesAndWeights(homographies, weights);
        mMaxHomographies = DEFAULT_MAX_HOMOGRAPHIES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
    }
    
    /**
     * Constructor.
     * @param homographies list of homographies (2D transformations) used to
     * estimate the image of absolute conic (IAC), which can be used to obtain
     * pinhole camera intrinsic parameters.
     * @param weights array containing a weight amount for each homography.
     * The larger the value of a weight, the most significant the correspondence
     * will be.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if not enough homographies are provided
     * for default constraints during IAC estimation.
     */
    public WeightedImageOfAbsoluteConicEstimator(
            List<Transformation2D> homographies, double[] weights,
            ImageOfAbsoluteConicEstimatorListener listener) 
            throws IllegalArgumentException {
        super(listener);
        internalSetHomographiesAndWeights(homographies, weights);
        mMaxHomographies = DEFAULT_MAX_HOMOGRAPHIES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
    }
    
    /**
     * Sets list of homographies to estimate IAC.
     * This method override always throws an IllegalArgumentException because
     * it is expected to provide both homographies and their weights.
     * @param homographies list of homographies to estimate IAC.
     * @throws IllegalArgumentException always thrown in this implementation.
     */
    @Override
    public void setHomographies(List<Transformation2D> homographies)
            throws IllegalArgumentException {
        throw new IllegalArgumentException();        
    }
    
    /**
     * Sets list of homographies to estimate IAC along with their corresponding
     * weights.
     * @param homographies list of homographies to estimate IAC.
     * @param weights array containing a weight amount for each homography.
     * The larger the value of a weight, the most significant the correspondence
     * will be.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if not enough homographies are provided
     * for default constraints during IAC estimation.
     */
    public void setHomographiesAndWeights(List<Transformation2D> homographies,
            double[] weights) throws LockedException, IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetHomographiesAndWeights(homographies, weights);
    }
    
    /**
     * Returns array containing a weight amount for each homography.
     * The larger the value of a weight, the most significant the correspondence
     * will be.
     * @return array containing weights for each correspondence.
     */
    public double[] getWeights() {
        return mWeights;
    }
    
    /**
     * Returns boolean indicating whether weights have been provided and are
     * available for retrieval.
     * @return true if weights are available, false otherwise.
     */
    public boolean areWeightsAvailable() {
        return mWeights != null;
    }
    
    /**
     * Returns maximum number of homographies to be weighted and taken into 
     * account.
     * @return maximum number of homographies to be weighted.
     */
    public int getMaxHomographies() {
        return mMaxHomographies;
    }
    
    /**
     * Sets maximum number of homographies to be weighted and taken into 
     * account.
     * This method must be called after enforcing constraints, because the 
     * minimum number of required homographies will be checked based on current
     * settings.
     * @param maxHomographies maximum number of homographies to be weighted.
     * @throws IllegalArgumentException if provided value is less than the
     * minimum allowed number of homographies.
     * @throws LockedException if this instance is locked.
     */
    public void setMaxHomographies(int maxHomographies) 
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxHomographies < getMinNumberOfRequiredHomographies()) {
            throw new IllegalArgumentException();
        }
        
        mMaxHomographies = maxHomographies;
    }
    
    /**
     * Indicates if weights are sorted by so that largest weighted homographies
     * are used first.
     * @return true if weights are sorted, false otherwise.
     */
    public boolean isSortWeightsEnabled() {
        return mSortWeights;
    }
    
    /**
     * Specifies whether weights are sorted by so that largest weighted
     * homographies are used first.
     * @param sortWeights true if weights are sorted, false otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setSortWeightsEnabled(boolean sortWeights)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mSortWeights = sortWeights;
    }
    
    /**
     * Indicates if this estimator is ready to start the estimation.
     * Estimator will be ready once both list of homographies and weights are
     * available and enough homobraphies have been provided
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && areWeightsAvailable() &&
                mHomographies.size() == mWeights.length;
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
            } else if (mPrincipalPointAtOrigin){ //&& !mZeroSkewness
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
        return ImageOfAbsoluteConicEstimatorType.WEIGHTED_IAC_ESTIMATOR;
    }
    
    /**
     * Sets list of homographies to estimate IAC.
     * This method does not check whether estimator is locked.
     * @param homographies list of homographies to estimate IAC.
     * @param weights array containing a weight amount for each homography.
     * The larger the value of a weight, the more significant the correspondence
     * will be.
     * @throws IllegalArgumentException if provided list of homographies does not
     * contain enough elements to estimate the IAC using current settings.
     */
    private void internalSetHomographiesAndWeights(
            List<Transformation2D> homographies, double[] weights) 
            throws IllegalArgumentException {
        if (weights == null || homographies == null || 
                weights.length != homographies.size()) {
            throw new IllegalArgumentException();
        }
        internalSetHomographies(homographies);
        mWeights = weights;
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
            WeightSelection selection = WeightSelection.selectWeights(mWeights, 
                    mSortWeights, mMaxHomographies);
            boolean[] selected = selection.getSelected();
            int nHomographies = selection.getNumSelected();

            Matrix a = new Matrix(2 * nHomographies, 6);
            
            int index = 0, counter = 0;
            ProjectiveTransformation2D t = null;
            Matrix h = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 
                    ProjectiveTransformation2D.HOM_COORDS);
            //elements ij of homography (last column is not required)
            double h11, h12, h21, h22, h31, h32, rowNorm, weight, factor;
            for (Transformation2D homography : mHomographies) {
                
                if (selected[index]) {
                    weight = mWeights[index];
                    
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
                    factor = weight / rowNorm ;

                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) * 
                            factor);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) *
                            factor);
                    a.setElementAt(counter, 2, a.getElementAt(counter, 2) *
                            factor);
                    a.setElementAt(counter, 3, a.getElementAt(counter, 3) *
                            factor);
                    a.setElementAt(counter, 4, a.getElementAt(counter, 4) *
                            factor);
                    a.setElementAt(counter, 5, a.getElementAt(counter, 5) *
                            factor);

                    counter++;

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
                    factor = weight / rowNorm;

                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) *
                            factor);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) *
                            factor);
                    a.setElementAt(counter, 2, a.getElementAt(counter, 2) *
                            factor);
                    a.setElementAt(counter, 3, a.getElementAt(counter, 3) *
                            factor);
                    a.setElementAt(counter, 4, a.getElementAt(counter, 4) *
                            factor);
                    a.setElementAt(counter, 5, a.getElementAt(counter, 5) *
                            factor);

                    counter++;
                }
                
                index++;
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
        } catch (AlgebraException | SortingException e) {
            throw new ImageOfAbsoluteConicEstimatorException(e);
        }
    }
        
    /**
     * Estimates Image of Absolute Conic (IAC) assuming that skewness is zero.
     * @return estimated IAC.
     * @throws ImageOfAbsoluteConicEstimatorException if an error occurs during
     * estimation, usually because repeated homographies are provided, or 
     * homographies corresponding to degenerate camera movements such as pure
     * parallel translations where no additional data is really provided.
     */    
    private ImageOfAbsoluteConic estimateZeroSkewness() 
            throws ImageOfAbsoluteConicEstimatorException {
        try {
            int nHomographies = mHomographies.size();
            
            WeightSelection selection = WeightSelection.selectWeights(mWeights, 
                    mSortWeights, mMaxHomographies);
            boolean[] selected = selection.getSelected();

            Matrix a = new Matrix(2 * nHomographies, 5);
            
            int index = 0, counter = 0;
            ProjectiveTransformation2D t = null;
            Matrix h = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 
                    ProjectiveTransformation2D.HOM_COORDS);
            //elements ij of homography (last column is not required)
            double h11, h12, h21, h22, h31, h32, rowNorm, weight, factor;
            for (Transformation2D homography : mHomographies){
                
                if (selected[index]) {
                    weight = mWeights[index];
                    
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
                    factor = weight / rowNorm;
                
                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) *
                            factor);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) *
                            factor);
                    a.setElementAt(counter, 2, a.getElementAt(counter, 2) *
                            factor);
                    a.setElementAt(counter, 3, a.getElementAt(counter, 3) *
                            factor);
                    a.setElementAt(counter, 4, a.getElementAt(counter, 4) *
                            factor);

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
                    factor = weight / rowNorm;

                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) *
                            factor);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) *
                            factor);
                    a.setElementAt(counter, 2, a.getElementAt(counter, 2) *
                            factor);
                    a.setElementAt(counter, 3, a.getElementAt(counter, 3) *
                            factor);
                    a.setElementAt(counter, 4, a.getElementAt(counter, 4) *
                            factor);
                
                    counter++;
                }
                
                index++;
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
        } catch (AlgebraException | SortingException e) {
            throw new ImageOfAbsoluteConicEstimatorException(e);
        }
    }
    
    /**
     * Estimates Image of Absolute Conic (IAC) assuming that principal point is
     * located at origin of coordinates.
     * @return estimated IAC.
     * @throws ImageOfAbsoluteConicEstimatorException if an error occurs during
     * estimation, usually because repeated homographies are provided, or 
     * homographies corresponding to degenerate camera movements such as pure
     * parallel translations where no additional data is really provided.
     */    
    private ImageOfAbsoluteConic estimatePrincipalPointAtOrigin() 
            throws ImageOfAbsoluteConicEstimatorException {
        
        try {
            int nHomographies = mHomographies.size();
            
            WeightSelection selection = WeightSelection.selectWeights(mWeights, 
                    mSortWeights, mMaxHomographies);
            boolean[] selected = selection.getSelected();

            Matrix a = new Matrix(2 * nHomographies, 4);
            
            int index = 0, counter = 0;
            ProjectiveTransformation2D t = null;
            Matrix h = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 
                    ProjectiveTransformation2D.HOM_COORDS);
            //elements ij of homography (last column is not required)
            double h11, h12, h21, h22, h31, h32, rowNorm, weight, factor;
            for (Transformation2D homography : mHomographies) {
                
                if (selected[index]) {
                    weight = mWeights[index];
                    
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
                    factor = weight / rowNorm;
                
                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) *
                            factor);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) *
                            factor);
                    a.setElementAt(counter, 2, a.getElementAt(counter, 2) *
                            factor);
                    a.setElementAt(counter, 3, a.getElementAt(counter, 3) *
                            factor);

                    counter++;
                                                
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
                    factor = weight / rowNorm;

                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) *
                            factor);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) *
                            factor);
                    a.setElementAt(counter, 2, a.getElementAt(counter, 2) *
                            factor);
                    a.setElementAt(counter, 3, a.getElementAt(counter, 3) *
                            factor);
                
                    counter++;   
                }
                
                index++;
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
        } catch (AlgebraException | SortingException e) {
            throw new ImageOfAbsoluteConicEstimatorException(e);
        }
    }
    
    /**
     * Estimates Image of Absolute Conic (IAC) assuming that skewness is zero 
     * and that principal point is located at origin of coordinates.
     * @return estimated IAC.
     * @throws ImageOfAbsoluteConicEstimatorException if an error occurs during
     * estimation, usually because repeated homographies are provided, or
     * homographies corresponding to degenerate camera movements such as pure
     * parallel translations where no additional data is really provided.
     */    
    private ImageOfAbsoluteConic estimateZeroSkewnessAndPrincipalPointAtOrigin() 
            throws ImageOfAbsoluteConicEstimatorException {

        try {
            int nHomographies = mHomographies.size();

            WeightSelection selection = WeightSelection.selectWeights(mWeights, 
                    mSortWeights, mMaxHomographies);
            boolean[] selected = selection.getSelected();
            
            Matrix a = new Matrix(2 * nHomographies, 3);
            
            int index = 0, counter = 0;
            ProjectiveTransformation2D t = null;
            Matrix h = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 
                    ProjectiveTransformation2D.HOM_COORDS);
            //elements ij of homography (last column is not required)
            double h11, h12, h21, h22, h31, h32, rowNorm, weight, factor;
            for (Transformation2D homography : mHomographies) {
                
                if (selected[index]) {
                    weight = mWeights[index];
                    
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
                    factor = weight / rowNorm;
                
                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) *
                            factor);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) *
                            factor);
                    a.setElementAt(counter, 2, a.getElementAt(counter, 2) *
                            factor);

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
                    factor = weight / rowNorm;

                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) *
                            factor);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) *
                            factor);
                    a.setElementAt(counter, 2, a.getElementAt(counter, 2) *
                            factor);
                
                    counter++;                   
                }
                
                index++;
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
        } catch (AlgebraException | SortingException e) {
            throw new ImageOfAbsoluteConicEstimatorException(e);
        }
    }        
    
    /**
     * Estimates Image of Absolute Conic (IAC) assuming that skewness is zero 
     * and that aspect ratio of focal distances is known.
     * @return estimated IAC.
     * @throws ImageOfAbsoluteConicEstimatorException if an error occurs during
     * estimation, usually because repeated homographies are provided, or 
     * homographies corresponding to degenerate camera movements such as pure
     * parallel translations where no additional data is really provided.
     */    
    private ImageOfAbsoluteConic estimateZeroSkewnessAndKnownFocalDistanceAspectRatio() 
            throws ImageOfAbsoluteConicEstimatorException {
        try {
            int nHomographies = mHomographies.size();
            
            WeightSelection selection = WeightSelection.selectWeights(mWeights, 
                    mSortWeights, mMaxHomographies);
            boolean[] selected = selection.getSelected();

            Matrix a = new Matrix(2 * nHomographies, 4);
            double sqrAspectRatio = Math.pow(mFocalDistanceAspectRatio, 2.0);
            
            int index = 0, counter = 0;
            ProjectiveTransformation2D t = null;
            Matrix h = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 
                    ProjectiveTransformation2D.HOM_COORDS);
            //elements ij of homography (last column is not required)
            double h11, h12, h21, h22, h31, h32, rowNorm, weight, factor;
            for (Transformation2D homography : mHomographies) {
                
                if (selected[index]) {
                    weight = mWeights[index];
                    
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
                    factor = weight / rowNorm;
                
                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) *
                            factor);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) *
                            factor);
                    a.setElementAt(counter, 2, a.getElementAt(counter, 2) *
                            factor);
                    a.setElementAt(counter, 3, a.getElementAt(counter, 3) *
                            factor);

                    counter++;
                                
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
                    factor = weight / rowNorm;

                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) *
                            factor);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) *
                            factor);
                    a.setElementAt(counter, 2, a.getElementAt(counter, 2) *
                            factor);
                    a.setElementAt(counter, 3, a.getElementAt(counter, 3) *
                            factor);
                
                    counter++;
                }
                
                index++;
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
        }catch(AlgebraException | SortingException e) {
            throw new ImageOfAbsoluteConicEstimatorException(e);
        }
    }
    
    /**
     * Estimates Image of Absolute Conic (IAC) assuming that skewness is zero, 
     * principal point is located at origin of coordinates and that aspect ratio
     * of focal distances is known.
     * @return estimated IAC.
     * @throws ImageOfAbsoluteConicEstimatorException if an error occurs during
     * estimation, usually because repeated homographies are provided, or 
     * homographies corresponding to degenerate camera movements such as pure
     * parallel translations where no additional data is really provided.
     */    
    private ImageOfAbsoluteConic estimateZeroSkewnessPrincipalPointAtOriginAndKnownFocalDistanceAspectRatio() 
            throws ImageOfAbsoluteConicEstimatorException {

        try {
            int nHomographies = mHomographies.size();

            WeightSelection selection = WeightSelection.selectWeights(mWeights, 
                    mSortWeights, mMaxHomographies);
            boolean[] selected = selection.getSelected();
            
            Matrix a = new Matrix(2 * nHomographies, 2);
            
            double sqrAspectRatio = Math.pow(mFocalDistanceAspectRatio, 2.0);
            
            int index = 0, counter = 0;
            ProjectiveTransformation2D t = null;
            Matrix h = new Matrix(ProjectiveTransformation2D.HOM_COORDS, 
                    ProjectiveTransformation2D.HOM_COORDS);
            //elements ij of homography (last column is not required)
            double h11, h12, h21, h22, h31, h32, rowNorm, weight, factor;
            for (Transformation2D homography : mHomographies) {
                
                if (selected[index]) {
                    weight = mWeights[index];
                    
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
                    factor = weight / rowNorm;
                
                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) *
                            factor);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) *
                            factor);

                    counter++;
                                                
                    //fill second equation
                    a.setElementAt(counter, 0, Math.pow(h11, 2.0) - Math.pow(h12, 2.0) +
                            (Math.pow(h21, 2.0) - Math.pow(h22, 2.0)) / sqrAspectRatio);
                    a.setElementAt(counter, 1, Math.pow(h31, 2.0) - Math.pow(h32, 2.0));
                
                    //normalize row
                    rowNorm = Math.sqrt(
                            Math.pow(a.getElementAt(counter, 0), 2.0) +
                            Math.pow(a.getElementAt(counter, 1), 2.0));
                    factor = weight / rowNorm;

                    a.setElementAt(counter, 0, a.getElementAt(counter, 0) *
                            factor);
                    a.setElementAt(counter, 1, a.getElementAt(counter, 1) *
                            factor);
                
                    counter++;                   
                }
                
                index++;
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
            double b11 = v.getElementAt(0, 1);
            double b33 = v.getElementAt(1, 1);
            
            double b22 = b11 / sqrAspectRatio;
            
            //A conic is defined as [A  B   D]
            //                      [B  C   E]
            //                      [D  E   F]
            //Since principal point is at origin of coordinates 
            //b12 = B = 0, b13 = D = 0.0, b23 = E = 0.0
            return new ImageOfAbsoluteConic(b11, 0.0, b22, 0.0, 0.0, b33);
        } catch (AlgebraException | SortingException e) {
            throw new ImageOfAbsoluteConicEstimatorException(e);
        }
    }
}
