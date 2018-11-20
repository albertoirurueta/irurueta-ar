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
package com.irurueta.ar.epipolar;

import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;

import java.util.Random;

/**
 * Compares two fundamental matrices by estimating average epipolar distances.
 * This class uses epipolar geometry to determine how similar fundamental 
 * matrices are. This is done by assuming a certain size on the retinal planes
 * and generating random 2D points to obtain their associated epipolar lines and
 * the distances of matched points to those epipolar lines
 * This class simply computes the norm of the difference of both fundamental
 * matrices. The smaller the value the more similar the fundamental matrices
 * will be from a pure algebraic point of view.
 */
@SuppressWarnings("WeakerAccess")
public class EpipolarDistanceFundamentalMatrixComparator extends 
        FundamentalMatrixComparator {
     
    /**
     * Defines default minimum horizontal coordinate when generating random 
     * samples.
     */
    public static final double DEFAULT_MIN_X = 0.0;
    
    /**
     * Defines default maximum horizontal coordinate when generating random 
     * samples.
     */
    public static final double DEFAULT_MAX_X = 640.0;
    
    /**
     * Defines default minimum vertical coordinate when generating random 
     * samples.
     */
    public static final double DEFAULT_MIN_Y = 0.0;
    
    /**
     * Defines default maximum vertical coordinate when generating random
     * samples.
     */
    public static final double DEFAULT_MAX_Y = 480.0;
    
    /**
     * Default number of random samples to generate to compare fundamental
     * matrices.
     */
    public static final int DEFAULT_N_SAMPLES = 100;
    
    /**
     * Minimum number of samples that must be generated to compare fundamental
     * matrices.
     */
    public static final int MIN_N_SAMPLES = 1;
    
    /**
     * Minimum disparity factor respect to retinal plane size defined by
     * minimum and maximum samples coordinates. When computing residuals, 
     * matched samples are created along the corresponding epipolar lines with a
     * random disparity within provided range, and then the epipolar line for
     * the randomly generated matched sample is generated on the original view
     * to determine the distance to such line and the original sample.
     */
    public static final double DEFAULT_MIN_DISPARITY_FACTOR = -0.1;

    /**
     * Maximum disparity factor respect to retinal plane size defined by
     * minimum and maximum samples coordinates. When computing residuals, 
     * matched samples are created along the corresponding epipolar lines with a
     * random disparity within provided range, and then the epipolar line for
     * the randomly generated matched sample is generated on the original view
     * to determine the distance to such line and the original sample.
     */
    public static final double DEFAULT_MAX_DISPARITY_FACTOR = 0.1;
    
    /**
     * Default factor to determine maximum number of iterations respect to the 
     * number of samples to compute comparison.
     */
    public static final double DEFAULT_MAX_ITERATIONS_FACTOR = 10.0;
    
    /**
     * Minimum value for the factor to determine maximum number of iterations
     * respect to the number of samples to compute comparison.
     */
    public static final double MIN_MAX_ITERATIONS_FACTOR = 1.0;
    
    /**
     * Default amount of progress variation before notifying a change in 
     * comparison progress. By default this is set to 5%.
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
     * Minimum horizontal coordinate when generating random samples.
     */
    private double mMinX;
    
    /**
     * Maximum horizontal coordinate when generating random samples.
     */
    private double mMaxX;
    
    /**
     * Minimum vertical coordinate when generating random samples.
     */
    private double mMinY;
    
    /**
     * Maximum vertical coordinate when generating random samples.
     */
    private double mMaxY;
    
    /**
     * Number of random samples to generate to compare fundamental matrices.
     */
    private int mNSamples;
    
    /**
     * Minimum horizontal disparity factor respect to retinal plane size defined
     * by minimum and maximum samples coordinates. When computing residuals, 
     * matched samples are created along the corresponding epipolar lines with a
     * random disparity within provided range of disparities, and then the 
     * epipolar line for the randomly generated matched sample is generated on 
     * the original view to determine the distance to such line and the original 
     * sample.
     */
    private double mMinHorizontalDisparityFactor;

    /**
     * Maximum horizontal disparity factor respect to retinal plane size defined
     * by minimum and maximum samples coordinates. When computing residuals, 
     * matched samples are created along the corresponding epipolar lines with a
     * random disparity within provided range of disparities, and then the 
     * epipolar line for the randomly generated matched sample is generated on 
     * the original view to determine the distance to such line and the original 
     * sample.
     */    
    private double mMaxHorizontalDisparityFactor;
    
    /**
     * Minimum vertical disparity factor respect to retinal plane size defined
     * by minimum and maximum samples coordinates. When computing residuals, 
     * matched samples are created along the corresponding epipolar lines with a
     * random disparity within provided range of disparities, and then the 
     * epipolar line for the randomly generated matched sample is generated on 
     * the original view to determine the distance to such line and the original 
     * sample.
     */    
    private double mMinVerticalDisparityFactor;
    
    /**
     * Maximum vertical disparity factor respect to retinal plane size defined
     * by minimum and maximum samples coordinates. When computing residuals, 
     * matched samples are created along the corresponding epipolar lines with a
     * random disparity within provided range of disparities, and then the 
     * epipolar line for the randomly generated matched sample is generated on 
     * the original view to determine the distance to such line and the original 
     * sample.
     */    
    private double mMaxVerticalDisparityFactor;
    
    /**
     * Factor to determine maximum number of iterations respect to the 
     * number of samples to compute comparison.
     */    
    private double mMaxIterationsFactor;
    
    /**
     * Amount of progress variation before notifying a progress change during
     * comparison.
     */
    protected float mProgressDelta;        
        
    /**
     * Constructor.
     */
    public EpipolarDistanceFundamentalMatrixComparator() {
        super();
        init();
    }
        
    /**
     * Constructor.
     * @param groundTruthFundamentalMatrix fundamental matrix to be considered
     * as ground truth to compare against.
     * @param otherFundamentalMatrix other fundamental matrix being compared.
     */
    public EpipolarDistanceFundamentalMatrixComparator(
            FundamentalMatrix groundTruthFundamentalMatrix,
            FundamentalMatrix otherFundamentalMatrix) {
        super(groundTruthFundamentalMatrix, otherFundamentalMatrix);
        init();
    }
    
    /**
     * Constructor.
     * @param listener listener to handle events generated by this class.
     */
    public EpipolarDistanceFundamentalMatrixComparator(
            FundamentalMatrixComparatorListener listener) {
        super(listener);
        init();
    }
    
    /**
     * Constructor.
     * @param groundTruthFundamentalMatrix fundamental matrix to be considered
     * as ground truth to compare against.
     * @param otherFundamentalMatrix other fundamental matrix being compared.
     * @param listener listener to handle events generated by this class.
     */
    public EpipolarDistanceFundamentalMatrixComparator(
            FundamentalMatrix groundTruthFundamentalMatrix,
            FundamentalMatrix otherFundamentalMatrix,
            FundamentalMatrixComparatorListener listener) {
        super(groundTruthFundamentalMatrix, otherFundamentalMatrix, listener);
        init();
    }
    
    /**
     * Returns minimum horizontal coordinate when generating random samples.
     * @return minimum horizontal coordinate when generating random samples.
     */
    public double getMinX() {
        return mMinX;
    }
    
    /**
     * Returns maximum horizontal coordinate when generating random samples.
     * @return maximum horizontal coordinate when generating random samples.
     */
    public double getMaxX() {
        return mMaxX;
    }
    
    /**
     * Sets minimum and maximum horizontal coordinates when generating random 
     * samples.
     * @param minX minimum horizontal coordinate when generating random samples.
     * @param maxX maximum horizontal coordinate when generating random samples.
     * @throws LockedException if this instance is locked.
     * @throws IllegalArgumentException if minimum value is larger or equal
     * than maximum one.
     */
    public void setMinMaxX(double minX, double maxX) throws LockedException,
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (minX >= maxX) {
            throw new IllegalArgumentException();
        }
        
        mMinX = minX;
        mMaxX = maxX;
    }

    /**
     * Returns minimum vertical coordinate when generating random samples.
     * @return minimum vertical coordinate when generating random samples.
     */
    public double getMinY() {
        return mMinY;
    }
    
    /**
     * Returns maximum vertical coordinate when generating random samples.
     * @return maximum vertical coordinate when generating random samples.
     */
    public double getMaxY() {
        return mMaxY;
    }
    
    /**
     * Sets minimum and maximum vertical coordinates when generating random 
     * samples.
     * @param minY minimum vertical coordinate when generating random samples.
     * @param maxY maximum vertical coordinate when generating random samples.
     * @throws LockedException if this instance is locked.
     * @throws IllegalArgumentException if minimum value is larger or equal
     * than maximum one.
     */
    public void setMinMaxY(double minY, double maxY) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (minY >= maxY) {
            throw new IllegalArgumentException();
        }
        
        mMinY = minY;
        mMaxY = maxY;
    }
    
    /**
     * Returns number of random samples to generate to compare fundamental
     * matrices.
     * @return number of random samples to generate to compare fundamental
     * matrices.
     */
    public int getNSamples() {
        return mNSamples;
    }
    
    /**
     * Sets number of random samples to generate to compare fundamental matrices.
     * @param nSamples number of random samples to generate to compare 
     * fundamental matrices.
     * @throws LockedException if this instance is locked.
     * @throws IllegalArgumentException if provided value is less than 1.
     */
    public void setNSamples(int nSamples) throws LockedException, 
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (nSamples < MIN_N_SAMPLES) {
            throw new IllegalArgumentException();
        }
        
        mNSamples = nSamples;
    }
    
    /**
     * Returns minimum horizontal disparity factor respect to retinal plane size
     * defined by minimum and maximum samples coordinates. When computing 
     * residuals, matched samples are created along the corresponding epipolar
     * lines with a random disparity within provided range of disparities, and
     * then the epipolar line for the randomly generated matched sample is
     * generated on the original view to determine the distance to such line and
     * the original sample.
     * @return minimum horizontal disparity factor.
     */
    public double getMinHorizontalDisparityFactor() {
        return mMinHorizontalDisparityFactor;
    }

    /**
     * Returns maximum horizontal disparity factor respect to retinal plane size
     * defined by minimum and maximum samples coordinates. When computing 
     * residuals, matched samples are created along the corresponding epipolar
     * lines with a random disparity within provided range of disparities, and
     * then the epipolar line for the randomly generated matched sample is
     * generated on the original view to determine the distance to such line and
     * the original sample.
     * @return maximum horizontal disparity factor.
     */    
    public double getMaxHorizontalDisparityFactor() {
        return mMaxHorizontalDisparityFactor;
    }
    
    /**
     * Sets minimum and maximum horizontal disparity factor respect to retinal
     * plane size defined by minimum and maximum samples coordinates. When
     * computing residuals, matched samples are created along the corresponding
     * epipolar lines with a random disparity within provided range of 
     * disparites, and then the epipolar line for the randomly generated matched
     * sample is generated on the original view to determine the distance to
     * such line and the original sample.
     * @param minHorizontalDisparityFactor minimum horizontal disparity factor.
     * @param maxHorizontalDisparityFactor maximum horizontal disparity factor.
     * @throws LockedException if this instance is locked.
     * @throws IllegalArgumentException if minimum value is larger or equal
     * than maximum one.
     */
    public void setMinMaxHorizontalDisparityFactor(
            double minHorizontalDisparityFactor, 
            double maxHorizontalDisparityFactor) throws LockedException, 
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (minHorizontalDisparityFactor >= maxHorizontalDisparityFactor) {
            throw new IllegalArgumentException();
        }
        
        mMinHorizontalDisparityFactor = minHorizontalDisparityFactor;
        mMaxHorizontalDisparityFactor = maxHorizontalDisparityFactor;
    }

    /**
     * Returns minimum vertical disparity factor respect to retinal plane size
     * defined by minimum and maximum samples coordinates. When computing 
     * residuals, matched samples are created along the corresponding epipolar
     * lines with a random disparity within provided range of disparities, and
     * then the epipolar line for the randomly generated matched sample is
     * generated on the original view to determine the distance to such line and
     * the original sample.
     * @return minimum vertical disparity factor.
     */
    public double getMinVerticalDisparityFactor() {
        return mMinVerticalDisparityFactor;
    }

    /**
     * Returns maximum vertical disparity factor respect to retinal plane size
     * defined by minimum and maximum samples coordinates. When computing 
     * residuals, matched samples are created along the corresponding epipolar
     * lines with a random disparity within provided range of disparities, and
     * then the epipolar line for the randomly generated matched sample is
     * generated on the original view to determine the distance to such line and
     * the original sample.
     * @return maximum vertical disparity factor.
     */    
    public double getMaxVerticalDisparityFactor() {
        return mMaxVerticalDisparityFactor;
    }
    
    /**
     * Sets minimum and maximum vertical disparity factor respect to retinal
     * plane size defined by minimum and maximum samples coordinates. When
     * computing residuals, matched samples are created along the corresponding
     * epipolar lines with a random disparity within provided range of 
     * disparites, and then the epipolar line for the randomly generated matched
     * sample is generated on the original view to determine the distance to
     * such line and the original sample.
     * @param minVerticalDisparityFactor minimum vertical disparity factor.
     * @param maxVerticalDisparityFactor maximum vertical disparity factor.
     * @throws LockedException if this instance is locked.
     * @throws IllegalArgumentException if minimum value is larger or equal
     * than maximum one.
     */
    public void setMinMaxVerticalDisparityFactor(
            double minVerticalDisparityFactor, 
            double maxVerticalDisparityFactor) throws LockedException, 
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (minVerticalDisparityFactor >= maxVerticalDisparityFactor) {
            throw new IllegalArgumentException();
        }
        
        mMinVerticalDisparityFactor = minVerticalDisparityFactor;
        mMaxVerticalDisparityFactor = maxVerticalDisparityFactor;
    }        

    /**
     * Returns factor to determine maximum number of iterations respect to the
     * number of samples to compute comparison.
     * @return factor to determine maximum number of iterations respect to the
     * number of samples to compute comparison.
     */
    public double getMaxIterationsFactor() {
        return mMaxIterationsFactor;
    }
    
    /**
     * Sets factor to determine maximum number of iterations respect to the
     * number of samples to compute comparison.
     * @param maxIterationsFactor maximum number of iterations respect to the
     * number of samples to compute comparison.
     * @throws LockedException if this instance is locked.
     * @throws IllegalArgumentException if provided value is less than 1.0.
     */
    public void setMaxIterationsFactor(double maxIterationsFactor) 
            throws LockedException, IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxIterationsFactor < MIN_MAX_ITERATIONS_FACTOR) {
            throw new IllegalArgumentException();
        }
        
        mMaxIterationsFactor = maxIterationsFactor;
    }
    
    /**
     * Returns amount of progress variation before notifying a progress change
     * during comparison computation.
     * @return amount of progress variation before notifying a progress change
     * during comparison computation.
     */
    public float getProgressDelta() {
        return mProgressDelta;
    }
    
    /**
     * Sets amount of progress variation before notifying a progress change 
     * during comparison computation.
     * @param progressDelta amount of progress variation before notifying a
     * progress change during comparison computation.
     * @throws IllegalArgumentException if progress delta is less than zero or
     * greater than 1.
     * @throws LockedException if this estimator is locked.
     */
    public void setProgressDelta(float progressDelta)
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA ||
                progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        mProgressDelta = progressDelta;
    }
    
    /**
     * Compares two fundamental matrices and returns the comparison value.
     * Comparison value will depend on the method implemented to compare both
     * fundamental matrices.
     * @return comparison value. Typically the smaller the absolute value the
     * more similar the fundamental matrices are.
     * @throws NotReadyException if this comparator is not  yet ready to start
     * the comparison.
     * @throws LockedException if this instance is locked.
     * @throws FundamentalMatrixComparatorException if comparison fails due to
     * some other reason.
     */    
    @Override
    public double compare() throws NotReadyException, LockedException, 
            FundamentalMatrixComparatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
                
        try {
            mLocked = true;            
            
            if (mListener != null) {
                mListener.onCompareStart(this);            
            }                        
            
            double minHorizontalDisparity = mMinHorizontalDisparityFactor * 
                    (mMaxX - mMinX);
            double maxHorizontalDisparity = mMaxHorizontalDisparityFactor *
                    (mMaxX - mMinX);
            double minVerticalDisparity = mMinVerticalDisparityFactor *
                    (mMaxY - mMinY);
            double maxVerticalDisparity = mMaxVerticalDisparityFactor *
                    (mMaxY - mMinY);
            
            double d1, d1prime, d2, d2prime;
            double inhomX, inhomY;
            boolean repeat;
            int counter;
            Point2D m1 = Point2D.create(
                    CoordinatesType.HOMOGENEOUS_COORDINATES);
            Point2D m2 = Point2D.create(
                    CoordinatesType.HOMOGENEOUS_COORDINATES);
            Line2D l1real = new Line2D();
            Line2D l1est = new Line2D();
            Line2D l2real = new Line2D();
            Line2D l2est = new Line2D();
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double avgDist = 0.0;
            int maxIterations = (int)(((double)mNSamples) * mMaxIterationsFactor);
            int currentIter = 0;            
            float progress, previousProgress = 0.0f;
            for (int i = 0; i < mNSamples; i++) {
                repeat = true;
                counter = 0;
                while (repeat && (counter < mNSamples)) {
                    //set x value
                    inhomX = randomizer.nextDouble(mMinX, mMaxX);
                    
                    //set y value
                    inhomY = randomizer.nextDouble(mMinY, mMaxY);
                    m1.setInhomogeneousCoordinates(inhomX, inhomY);
                    m1.normalize();
                    
                    //real epipolar line for random point m1
                    mGroundTruthFundamentalMatrix.rightEpipolarLine(m1, l2real);
                    l2real.normalize();
                    
                    //check that epipolar line lies within retinal plane size
                    //taking into account the following equation
                    //x*l2.getA() + y*l2.getB() + l2.getC() = 0
                    double yMinX, yMaxX;
                    if (Math.abs(l2real.getB()) > Double.MIN_VALUE) {
                        //for x = mMinX
                        yMinX = -(mMinX * l2real.getA() + l2real.getC()) /
                                l2real.getB();
                        
                        //for x = mMaxX
                        yMaxX = -(mMaxX * l2real.getA() + l2real.getC()) /
                                l2real.getB();
                    } else {
                        yMinX = l2real.getA() >= 0.0 ?
                                -Double.MAX_VALUE : Double.MAX_VALUE;
                        yMaxX = -yMinX;
                    }
                    
                    //for y = mMinY
                    double xMinY = -(mMinY * l2real.getB() + l2real.getC()) /
                            l2real.getA();
                    
                    //for y = mMaxY
                    double xMaxY = -(mMaxY * l2real.getB() + l2real.getC()) /
                            l2real.getA();
                    
                    //if epipolar line does not intersect second image, we need
                    //to repeat with a different sample
                    repeat = (((yMinX < mMinY) && (yMaxX < mMinY)) || ((yMinX > mMaxY) &&
                        (yMaxX > mMaxY)) || ((xMinY < mMinX) && (xMaxY < mMinX)) ||
                        ((xMinY > mMaxX) && (xMaxY > mMaxX)));
                    counter++;
                    currentIter++;
                }
                
                if (counter >= mNSamples) {
                    continue;
                }
                
                //choose point lying on epipolar line l2real :
                //m2.getX() * l2real.getA() + m2.getY() * l2real.getB() +
                //l2real.getC() = 0
                //choose random horizontal component within provided disparity:
                inhomX = m1.getInhomX() + randomizer.nextDouble(
                        minHorizontalDisparity, maxHorizontalDisparity);
                if (Math.abs(l2real.getB()) > Double.MIN_VALUE) {
                    inhomY = -(inhomX * l2real.getA() + l2real.getC()) /
                            l2real.getB();
                } else {
                    inhomY = Double.MAX_VALUE;
                }
                
                //if point lies outside retinal plane limits, try setting random
                //vertical component within provided disparity
                if ((inhomY < mMinY) || (inhomY > mMaxY)) {
                    inhomY = m1.getInhomY() + randomizer.nextDouble(
                            minVerticalDisparity, maxVerticalDisparity);
                    inhomX = -(inhomY * l2real.getB() + l2real.getC()) /
                            l2real.getA();
                }
                
                m2.setInhomogeneousCoordinates(inhomX, inhomY);
                m2.normalize();
                
                //estimated epipolar line for some random point m1
                mOtherFundamentalMatrix.rightEpipolarLine(m1, l2est);
                l2est.normalize();
                
                //compute distance from l2est to m2 (distance from estimated to
                //real)
                d1prime = Math.abs(l2est.signedDistance(m2));
                
                mOtherFundamentalMatrix.leftEpipolarLine(m2, l1est);
                l1est.normalize();
                d1 = Math.abs(l1est.signedDistance(m1));
                
                //repeat reversing roles of ground truth and other fundamental
                //matrix
                repeat = true;
                counter = 0;
                while (repeat && (counter < mNSamples)) {
                    //set x value
                    inhomX = randomizer.nextDouble(mMinX, mMaxX);
                    
                    //set y value
                    inhomY = randomizer.nextDouble(mMinY, mMaxY);
                    m1.setInhomogeneousCoordinates(inhomX, inhomY);
                    m1.normalize();
                    
                    //real epipolar line for random point m1
                    mOtherFundamentalMatrix.rightEpipolarLine(m1, l2est);
                    l2est.normalize();
                    
                    //check that epipolar line lies within retinal plane size
                    //taking into account the following equation
                    //x*l2.getA() + y*l2.getB() + l2.getC() = 0
                    double yMinX, yMaxX;
                    if (Math.abs(l2est.getB()) > Double.MIN_VALUE) {
                        //for x = mMinX
                        yMinX = -(mMinX * l2est.getA() + l2est.getC()) /
                                l2est.getB();
                        
                        //for x = mMaxX
                        yMaxX = -(mMaxX * l2est.getA() + l2est.getC()) /
                                l2est.getB();
                    } else {
                        yMinX = l2est.getA() >= 0.0 ? -Double.MAX_VALUE : 
                                Double.MAX_VALUE;
                        yMaxX = -yMinX;
                    }
                    
                    //for y = mMinY
                    double xMinY = -(mMinY * l2est.getB() + l2est.getC()) /
                            l2est.getA();
                    
                    //for y = mMaxY
                    double xMaxY = -(mMaxY * l2est.getB() + l2est.getC()) /
                            l2est.getA();
                    
                    //if epipolar line does not intersect second image, we need
                    //to repeat with a different sample
                    repeat = (((yMinX < mMinY) && (yMaxX < mMinY)) || ((yMinX > mMaxY) &&
                        (yMaxX > mMaxY)) || ((xMinY < mMinX) && (xMaxY < mMinX)) ||
                    ((xMinY > mMaxX) && (xMaxY > mMaxX)));
                    counter++;
                    currentIter++;
                }
                
                if (counter >= mNSamples) {
                    continue;
                }
                
                //choose point lying on epipolar line l2est :
                //m2.getX() * l2est.getA() + m2.getY() * l2est.getB() +
                //l2est.getC() = 0
                //choose random horizontal component within provided disparity:
                inhomX = m1.getInhomX() + randomizer.nextDouble(
                        minHorizontalDisparity, maxHorizontalDisparity);
                if (Math.abs(l2real.getB()) > Double.MIN_VALUE) {
                    inhomY = -(inhomX * l2est.getA() + l2est.getC()) /
                            l2est.getB();
                } else {
                    inhomY = Double.MAX_VALUE;
                }
                
                //if point lies outside retinal plane limits, try setting random
                //Vertical component within provided disparity
                if ((inhomY < mMinY) || (inhomY > mMaxY)) {
                    inhomY = m1.getInhomY() + randomizer.nextDouble(
                            minVerticalDisparity, maxVerticalDisparity);
                    inhomX = -(inhomY * l2est.getB() + l2est.getC()) /
                            l2est.getA();
                }
                
                m2.setInhomogeneousCoordinates(inhomX, inhomY);
                m2.normalize();
                
                //estimated epipolar line for some random point m1
                mOtherFundamentalMatrix.rightEpipolarLine(m1, l2real);
                l2real.normalize();
                
                //compute distance from l2real to m2 (distance from estimated to
                //real)
                d2prime = Math.abs(l2real.signedDistance(m2));
                
                mOtherFundamentalMatrix.leftEpipolarLine(m2, l1real);
                l1real.normalize();
                d2 = Math.abs(l1real.signedDistance(m1));
                
                avgDist += (d1 + d1prime + d2 + d2prime) / 4.0;
                
                if (currentIter > maxIterations) {
                    throw new FundamentalMatrixComparatorException();
                }
                
                progress = (float)currentIter / (float)maxIterations;
                
                if (mListener != null) {
                    if (progress - previousProgress > mProgressDelta) {
                        previousProgress = progress;
                        mListener.onCompareProgressChange(this, progress);
                    }
                }
            }
            
            
            if (mListener != null) {
                mListener.onCompareEnd(this);
            }
        
            return avgDist / (double)mNSamples;
            
        } finally {
            mLocked = false;
        }                  
    }

    /**
     * Returns type of comparator.
     * @return type of comparator.
     */    
    @Override
    public FundamentalMatrixComparatorType getType() {
        return FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR;
    }
 
    /**
     * Initializes default settings.
     */
    private void init() {
        mMinX = DEFAULT_MIN_X;
        mMaxX = DEFAULT_MAX_X;
        mMinY = DEFAULT_MIN_Y;
        mMaxY = DEFAULT_MAX_Y;
        mNSamples = DEFAULT_N_SAMPLES;
        mMinHorizontalDisparityFactor = mMinVerticalDisparityFactor =
                DEFAULT_MIN_DISPARITY_FACTOR;
        mMaxHorizontalDisparityFactor = mMaxVerticalDisparityFactor =
                DEFAULT_MAX_DISPARITY_FACTOR;      
        mMaxIterationsFactor = DEFAULT_MAX_ITERATIONS_FACTOR;
        mProgressDelta = DEFAULT_PROGRESS_DELTA;
    }    
}
