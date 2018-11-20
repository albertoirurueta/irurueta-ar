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
import com.irurueta.algebra.Utils;
import com.irurueta.ar.calibration.RadialDistortion;
import com.irurueta.ar.calibration.RadialDistortionException;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.util.Iterator;
import java.util.List;

/**
 * This class defines an LMSE (Least Mean Square Error) estimator of radial 
 * distorsion.
 * Equations to determine a RadialDistortion instance for a single point are 
 * linear dependent, for that reason, at least 2 points are required for the 
 * estimation.
 * Even though x and y equations are linear dependent, both equations are taken
 * into account in case that sampled data contains errors, so that an LMSE error
 * can be obtained.
 */
public class LMSERadialDistortionEstimator extends RadialDistortionEstimator {
    /**
     * Indicates if by default an LMSE (Least Mean Square Error) solution is
     * allowed if more correspondences than the minimum are provided.
     */
    public static final boolean DEFAULT_ALLOW_LMSE_SOLUTION = false;
            
    /**
     * Indicates if an LMSE (Least Mean Square Error) solution is allowed if
     * more correspondences than the minimum are provided. If false, the 
     * exceeding correspondences will be ignored and only the 6 first 
     * correspondences will be used.
     */
    private boolean mAllowLMSESolution;
        
    /**
     * Constructor.
     */
    public LMSERadialDistortionEstimator() {
        super();
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public LMSERadialDistortionEstimator(
            RadialDistortionEstimatorListener listener) {
        super(listener);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */
    public LMSERadialDistortionEstimator(List<Point2D> distortedPoints, 
            List<Point2D> undistortedPoints) throws IllegalArgumentException {
        super(distortedPoints, undistortedPoints);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
   
    /**
     * Constructor.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */
    
    public LMSERadialDistortionEstimator(List<Point2D> distortedPoints, 
            List<Point2D> undistortedPoints, 
            RadialDistortionEstimatorListener listener) 
            throws IllegalArgumentException {
        super(distortedPoints, undistortedPoints, listener);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }

    /**
     * Constructor with distortion center.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     */
    public LMSERadialDistortionEstimator(Point2D distortionCenter) {
        super(distortionCenter);
    }
    
    /**
     * Constructor with listener and distortion center.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public LMSERadialDistortionEstimator(Point2D distortionCenter,
            RadialDistortionEstimatorListener listener) {
        super(distortionCenter, listener);
    }
    
    /**
     * Constructor with points and distortion center.
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */
    public LMSERadialDistortionEstimator(List<Point2D> distortedPoints, 
            List<Point2D> undistortedPoints, Point2D distortionCenter)
            throws IllegalArgumentException {
        super(distortedPoints, undistortedPoints, distortionCenter);
    }
   
    /**
     * Constructor
     * @param distortedPoints list of distorted points. Distorted points are
     * obtained after radial distorsion is applied to an undistorted point.
     * @param undistortedPoints list of undistorted points.
     * @param distortionCenter Distortion center. This is usually equal to the 
     * principal point of an estimated camera. If not set it is assumed to be at
     * the origin of coordinates (0,0).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size.
     */ 
    public LMSERadialDistortionEstimator(List<Point2D> distortedPoints, 
            List<Point2D> undistortedPoints, Point2D distortionCenter,
            RadialDistortionEstimatorListener listener) 
            throws IllegalArgumentException {
        super(distortedPoints, undistortedPoints, distortionCenter, listener);
    }
        
    /**
     * Indicates if an LMSE (Least Mean Square Error) solution is allowed if 
     * more correspondences than the minimum are provided. If false, the 
     * exceeding correspondences will be ignored and only the 6 first 
     * correspondences will be used.
     * @return true if LMSE solution is allowed, false otherwise.
     */
    public boolean isLMSESolutionAllowed() {
        return mAllowLMSESolution;
    }
    
    /**
     * Specifies if an LMSE (Least Mean Square Error) solution is allowed if
     * more correspondences than the minimum are provided. If false, the
     * exceeding correspondences will be ignored and only the 6 first 
     * correspondences will be used.
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
     * Estimates a radial distortion.
     * @return estimated radial distortion.
     * @throws LockedException if estimator is loked.
     * @throws NotReadyException if input has not yet been provided.
     * @throws RadialDistortionEstimatorException if an error occurs during
     * estimation, usually because input data is not valid.
     */    
    @Override
    public RadialDistortion estimate() throws LockedException, 
            NotReadyException, RadialDistortionEstimatorException {
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
            
            int nPoints = mDistortedPoints.size();
            
            int numRows;
            if (isLMSESolutionAllowed()) {
                //initialize new matrix having two rows per point
                numRows = 2 * nPoints;
            } else {
                //when LMSE is not allowed, restrict matrix to two rows (minimum
                //value required for a solution)
                numRows = 2 * getMinNumberOfMatchedPoints();
            }
            
            Matrix A = new Matrix(numRows, mNumKParams);
            double[] b = new double[numRows];
            
            
            Iterator<Point2D> iteratorDistorted = mDistortedPoints.iterator();
            Iterator<Point2D> iteratorUndistorted = 
                    mUndistortedPoints.iterator();
            
            Point2D distorted, undistorted;
            int counter = 0;
            
            //undistorted normalized homogeneous coordinates
            double uNormHomX, uNormHomY, uNormHomW; 
            //undistorted normalized inhomogeneous coordinates
            double uNormInhomX, uNormInhomY;
            //undistorted denormalized homogeneous coordinates
            double uDenormHomX, uDenormHomY, uDenormHomW; 
            //undistorted denormalized inhomogeneous coordinates
            double uDenormInhomX, uDenormInhomY;
            //distorted inhomogeneous coordinates
            double dInhomX, dInhomY;
            double rowNormX, rowNormY;
            
            //radial distortion center
            double centerX = 0.0, centerY = 0.0;
            if (mDistortionCenter != null) {
                centerX = mDistortionCenter.getInhomX();
                centerY = mDistortionCenter.getInhomY();
            }
            
            //radial distance of undistorted normalized (calibration independent) 
            //coordinates
            double r2; 
            double a, value;
            
            while (iteratorDistorted.hasNext() && iteratorUndistorted.hasNext()) {
                distorted = iteratorDistorted.next();
                undistorted = iteratorUndistorted.next();
                
                undistorted.normalize();
                
                uDenormHomX = undistorted.getHomX();
                uDenormHomY = undistorted.getHomY();
                uDenormHomW = undistorted.getHomW();
                
                uDenormInhomX = uDenormHomX / uDenormHomW;
                uDenormInhomY = uDenormHomY / uDenormHomW;
                
                //multiply intrinsic parameters by undistorted point
                uNormHomX = mKinv.getElementAt(0, 0) * uDenormHomX +
                        mKinv.getElementAt(0, 1) * uDenormHomY +
                        mKinv.getElementAt(0, 2) * uDenormHomW;
                uNormHomY = mKinv.getElementAt(1, 0) * uDenormHomX +
                        mKinv.getElementAt(1, 1) * uDenormHomY +
                        mKinv.getElementAt(1, 2) * uDenormHomW;
                uNormHomW = mKinv.getElementAt(2, 0) * uDenormHomX +
                        mKinv.getElementAt(2, 1) * uDenormHomY +
                        mKinv.getElementAt(2, 2) * uDenormHomW;
                
                uNormInhomX = uNormHomX / uNormHomW;
                uNormInhomY = uNormHomY / uNormHomW;
                
                r2 = uNormInhomX * uNormInhomX + uNormInhomY * uNormInhomY;
                
                dInhomX = distorted.getInhomX();
                dInhomY = distorted.getInhomY();
                
                a = 1.0;
                rowNormX = rowNormY = 0.0;
                for (int i = 0; i < mNumKParams; i++) {
                    a *= r2;
                    
                    //x and y coordinates generate linear dependent equations, for 
                    //that reason we need more than one point
                    
                    //x coordinates
                    value = (uDenormInhomX - centerX) * a;
                    A.setElementAt(2 * counter, i, value);
                    
                    rowNormX += Math.pow(value, 2.0);
                    
                    //y coordinates
                    value = (uDenormInhomY - centerY) * a;
                    A.setElementAt(2 * counter + 1, i, value);
                    
                    rowNormY += Math.pow(value, 2.0);
                }
                
                //x coordinates
                value = dInhomX - uDenormInhomX;
                b[2 * counter] = value;
                
                rowNormX += Math.pow(value, 2.0);
                
                //y coordinates
                value = dInhomY - uDenormInhomY;
                b[2 * counter + 1] = value;
                
                rowNormY += Math.pow(value, 2.0);
                
                //normalize rows to increase accuracy
                for (int i = 0; i < mNumKParams; i++) {
                    A.setElementAt(2 * counter, i, 
                            A.getElementAt(2 * counter, i) / rowNormX);
                    A.setElementAt(2 * counter + 1, i, 
                            A.getElementAt(2 * counter + 1, i) / rowNormY);
                }
                
                b[2 * counter] /= rowNormX;
                b[2 * counter + 1] /= rowNormY;
                                
                counter++;
                
                if (!isLMSESolutionAllowed() && (counter >= getMinNumberOfMatchedPoints())) {
                    break;
                }
            }
            
            double[] params = Utils.solve(A, b);
            
            RadialDistortion distortion = 
                    new RadialDistortion(params, mDistortionCenter, 
                    mHorizontalFocalLength, mVerticalFocalLength, mSkew);
            
            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
            
            return distortion;
        } catch (AlgebraException | RadialDistortionException e) {
            throw new RadialDistortionEstimatorException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Returns type of radial distortion estimator.
     * @return type of radial distortion estimator.
     */    
    @Override
    public RadialDistortionEstimatorType getType() {
        return RadialDistortionEstimatorType.LMSE_RADIAL_DISTORTION_ESTIMATOR;
    }
}
