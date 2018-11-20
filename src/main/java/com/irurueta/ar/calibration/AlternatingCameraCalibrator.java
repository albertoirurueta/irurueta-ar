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

import com.irurueta.ar.calibration.estimators.*;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
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
 * 
 * This class is based on technique described at:
 * Zhengyou Zhang. A Flexible New Technique for Camera Calibration. Technical 
 * Report. MSR-TR-98-71. December 2, 1998
 */
@SuppressWarnings("WeakerAccess")
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
    public static final RobustEstimatorMethod DEFAULT_RADIAL_DISTORTION_METHOD = 
            RobustEstimatorMethod.PROSAC;
    
    /**
     * Maximum number of times to do an alternating iteration to refine the
     * results.
     */
    private int mMaxIterations;
    
    /**
     * Default threshold to determine that convergence of the result has been
     * reached.
     */
    private double mConvergenceThreshold;
    
    /**
     * Robust estimator method to be used for radial distortion estimation.
     */
    private RobustEstimatorMethod mDistortionMethod;
    
    /**
     * Robust estimator of radial distortion.
     */
    private RadialDistortionRobustEstimator mDistortionEstimator;
    
    /**
     * Listener for robust estimator of radial distortion.
     */
    private RadialDistortionRobustEstimatorListener
            mDistortionEstimatorListener;
        
    /**
     * Indicates progress of radial distortion estimation.
     */
    private float mRadialDistortionProgress;
    
    /**
     * Overall progress taking into account current number of iteration.
     */
    private float mIterProgress;    
    
    /**
     * Previously notified progress.
     */
    private float mPreviousNotifiedProgress;
    
    /**
     * Constructor.
     */
    public AlternatingCameraCalibrator() {
        super();
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        mConvergenceThreshold = DEFAULT_CONVERGENCE_THRESHOLD;
        
        internalSetDistortionMethod(DEFAULT_RADIAL_DISTORTION_METHOD);
    }
    
    /**
     * Constructor.
     * @param pattern 2D pattern to use for calibration.
     * @param samples samples of the pattern taken with the camera to calibrate.
     * @throws IllegalArgumentException if not enough samples are provided.
     */
    public AlternatingCameraCalibrator(Pattern2D pattern,
                                       List<CameraCalibratorSample> samples)
            throws IllegalArgumentException {
        super(pattern, samples);
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        mConvergenceThreshold = DEFAULT_CONVERGENCE_THRESHOLD;        
        
        internalSetDistortionMethod(DEFAULT_RADIAL_DISTORTION_METHOD);
    }
    
    /**
     * Constructor.
     * @param pattern 2D pattern to use for calibration.
     * @param samples samples of the pattern taken with the camera to calibrate.
     * @param samplesQualityScores quality scores for each sample.
     * @throws IllegalArgumentException if not enough samples are provided or if
     * both samples and quality scores do not have the same size.
     */
    public AlternatingCameraCalibrator(Pattern2D pattern,
            List<CameraCalibratorSample> samples,
            double[] samplesQualityScores) throws IllegalArgumentException {
        super(pattern, samples, samplesQualityScores);
        mMaxIterations = DEFAULT_MAX_ITERATIONS;
        mConvergenceThreshold = DEFAULT_CONVERGENCE_THRESHOLD;                
        
        internalSetDistortionMethod(DEFAULT_RADIAL_DISTORTION_METHOD);
    }
    
    /**
     * Returns maximum number of times to do an alternating iteration to refine
     * the results.
     * @return maximum number of times to do an alternating iteration.
     */
    public int getMaxIterations() {
        return mMaxIterations;
    }
    
    /**
     * Sets maximum number of times to do an alternating iteration to refine the
     * results.
     * @param maxIterations maximum number of times to do an alternating 
     * iteration.
     * @throws LockedException if this instance is locked.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setMaxIterations(int maxIterations) 
            throws LockedException, IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxIterations < MIN_MAX_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        
        mMaxIterations = maxIterations;
    }

    /**
     * Returns threshold to determine that convergence of the result has been
     * reached.
     * @return threshold to determine that convergence of the result has been
     * reached.
     */
    public double getConvergenceThreshold() {
        return mConvergenceThreshold;
    }
    
    /**
     * Sets threshold to determine that convergence of the result has been 
     * reached.
     * @param convergenceThreshold threshold to determine that convergence of
     * the result has been reached.
     * @throws LockedException if this instance is locked.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setConvergenceThreshold(double convergenceThreshold)
            throws LockedException, IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (convergenceThreshold < MIN_CONVERGENCE_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        
        mConvergenceThreshold = convergenceThreshold;
    }

    /**
     * Returns robust estimator method to be used for radial distortion 
     * estimation.
     * @return robust estimator method to be used for radial distortion
     * estimation.
     */
    public RobustEstimatorMethod getDistortionMethod() {
        return mDistortionMethod;
    }
    
    /**
     * Sets robust estimator method to be used for radial distortion
     * estimation.
     * @param distortionMethod robust estimator method to be used for
     * radial distortion estimation.
     * @throws LockedException if this instance is locked.
     */
    public void setDistortionMethod(
            RobustEstimatorMethod distortionMethod) 
            throws LockedException {
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
     * @return radial distortion estimator.
     */
    public RadialDistortionRobustEstimator getDistortionEstimator() {
        return mDistortionEstimator;
    }
    
    /**
     * Returns threshold to robustly estimate radial distortion.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     * @return threshold to robustly estimate radial distortion.
     */
    public double getDistortionEstimatorThreshold() {
        switch (mDistortionEstimator.getMethod()) {
            case LMedS:
                return ((LMedSRadialDistortionRobustEstimator)mDistortionEstimator).
                        getStopThreshold();
            case MSAC:
                return ((MSACRadialDistortionRobustEstimator)mDistortionEstimator).
                        getThreshold();                
            case PROSAC:
                return ((PROSACRadialDistortionRobustEstimator)mDistortionEstimator).
                        getThreshold();                
            case PROMedS:
                return ((PROMedSRadialDistortionRobustEstimator)mDistortionEstimator).
                        getStopThreshold();                
            case RANSAC:
            default:
                return ((RANSACRadialDistortionRobustEstimator)mDistortionEstimator).
                        getThreshold();
        }
    }
    
    /**
     * Sets threshold to robustly estimate radial distortion.
     * Usually the default value is good enough for most situations, but this 
     * setting can be changed for finder adjustments.
     * @param distortionEstimatorThreshold threshold to robustly estimate
     * radial distortion .
     * @throws LockedException if this instance is locked.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setDistortionEstimatorThreshold(
            double distortionEstimatorThreshold) throws LockedException,
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        switch (mDistortionEstimator.getMethod()) {
            case LMedS:
                ((LMedSRadialDistortionRobustEstimator)mDistortionEstimator).
                        setStopThreshold(distortionEstimatorThreshold);
                break;
            case MSAC:
                ((MSACRadialDistortionRobustEstimator)mDistortionEstimator).
                        setThreshold(distortionEstimatorThreshold);
                break;                
            case PROSAC:
                ((PROSACRadialDistortionRobustEstimator)mDistortionEstimator).
                        setThreshold(distortionEstimatorThreshold);
                break;                
            case PROMedS:
                ((PROMedSRadialDistortionRobustEstimator)mDistortionEstimator).
                        setStopThreshold(distortionEstimatorThreshold);
                break;                
            case RANSAC:
            default:
                ((RANSACRadialDistortionRobustEstimator)mDistortionEstimator).
                        setThreshold(distortionEstimatorThreshold);
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
     * Typically this value will be close to 1.0, but not exactly 1.0, because
     * a 100% confidence would require an infinite number of iterations.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     * @return confidence to robustly estimate homographies.
     */
    public double getDistortionEstimatorConfidence() {
        return mDistortionEstimator.getConfidence();
    }
    
    /**
     * Sets confidence to robustly estimate radial distortion.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     * Confidence is expressed as a value between 0.0 (0%) and 1.0 (100%). The
     * amount of confidence indicates the probability that the estimated
     * homography is correct (i.e. no outliers were used for the estimation,
     * because they were successfully discarded).
     * Typically this value will be close to 1.0, but not exactly 1.0, because
     * a 100% confidence would require an infinite number of iterations.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     * @param distortionEstimatorConfidence confidence to robustly estimate
     * radial distortion.
     * @throws LockedException if this instance is locked.
     * @throws IllegalArgumentException if provided value is not between 0.0 and
     * 1.0.
     */
    public void setDistortionEstimatorConfidence(
            double distortionEstimatorConfidence) throws LockedException,
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mDistortionEstimator.setConfidence(distortionEstimatorConfidence);
    }
    
    /**
     * Returns the maximum number of iterations to be done when estimating
     * the radial distortion.
     * If the maximum allowed number of iterations is reached, resulting 
     * estimation might not have desired confidence.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     * @return maximum number of iterations to be done when estimating the
     * homographies.
     */
    public int getDistortionEstimatorMaxIterations() {
        return mDistortionEstimator.getMaxIterations();
    }
    
    /**
     * Sets the maximum number of iterations to be done when estimating the
     * radial distortion.
     * If the maximum allowed number of iterations is reached, resulting 
     * estimation might not have desired confidence.
     * Usually the default value is good enough for most situations, but this
     * setting can be changed for finer adjustments.
     * @param distortionEstimatorMaxIterations maximum number of iterations to
     * be done when estimating radial distortion.
     * @throws LockedException if this instance is locked.
     * @throws IllegalArgumentException if provided value is negative or zero.
     */
    public void setDistortionEstimatorMaxIterations(
            int distortionEstimatorMaxIterations) throws LockedException,
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        mDistortionEstimator.setMaxIterations(distortionEstimatorMaxIterations);
    }
    
    /**
     * Starts the calibration process.
     * Depending on the settings the following will be estimated:
     * intrinsic pinhole camera parameters, radial distortion of lens,
     * camera pose (rotation and translation) for each sample, and the 
     * associated homobraphy of sampled points respect to the ideal pattern 
     * samples.
     * @throws CalibrationException if calibration fails for some reason.
     * @throws LockedException if this instance is locked because calibration is
     * already in progress.
     * @throws NotReadyException if this instance does not have enough data to
     * start camera calibration.
     */    
    @Override
    public void calibrate() throws CalibrationException, LockedException, 
            NotReadyException {
        
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        mLocked = true;
        
        mHomographyQualityScoresRequired = 
                (mDistortionEstimator.getMethod() == RobustEstimatorMethod.PROSAC ||
                mDistortionEstimator.getMethod() == RobustEstimatorMethod.PROMedS);
        
        if (mListener != null) {
            mListener.onCalibrateStart(this);
        }
            
        reset();
        mRadialDistortionProgress = mIterProgress = mPreviousNotifiedProgress = 
                0.0f;        
        
        List<Point2D> idealFallbackPatternMarkers = mPattern.getIdealPoints();
                
        try {
            double errorDiff, previousError, currentError = Double.MAX_VALUE;
            double bestError = Double.MAX_VALUE;
            PinholeCameraIntrinsicParameters bestIntrinsic = null;
            RadialDistortion bestDistortion = null;
            
            //iterate until error converges
            int iter = 0;
            do {
                previousError = currentError;
                
                //estimate intrinsic parameters
                estimateIntrinsicParameters(idealFallbackPatternMarkers);
                                
                if (!mEstimateRadialDistortion) {
                    break;
                }
                        
                //estimate radial distortion using estimated intrinsic 
                //parameters and camera poses and obtain average reprojection 
                //error                
                currentError = estimateRadialDistortion(
                        idealFallbackPatternMarkers);
                
                if (currentError < bestError) {
                    bestError=currentError;
                    bestIntrinsic = mIntrinsic;
                    bestDistortion = mDistortion;
                }
                                
                errorDiff = Math.abs(previousError - currentError);  
                iter++;
                mIterProgress = (float)iter / (float)mMaxIterations;
                notifyProgress();
                
            } while (errorDiff > mConvergenceThreshold && iter < mMaxIterations);
            
            if (bestIntrinsic != null) {
                mIntrinsic = bestIntrinsic;
            }
            if (bestDistortion != null) {
                mDistortion = bestDistortion;
            }
            
            if (mListener != null) {
                mListener.onCalibrateEnd(this);
            }            
        } finally {
            mLocked = false;
        }                
    }
    
    /**
     * Estimates radial distortion using estimated intrinsic parameters among
     * all samples to estimate their camera poses to find non distorted points
     * and compare them with the sampled ones.
     * @param idealFallbackPatternMarkers ideal pattern markers coordinates 
     * These coordinates are used as fallback when a given sample does not have 
     * an associated pattern.
     * @return average reprojection error, obtained after projecting ideal 
     * pattern markers using estimated camera poses and then doing a comparison 
     * with sampled points taking into account estimated distortion to undo 
     * their corresponding distortion.
     * @throws CalibrationException if anything fails.
     */
    protected double estimateRadialDistortion(
            List<Point2D> idealFallbackPatternMarkers)            
            throws CalibrationException {
        
        mRadialDistortionProgress = 0.0f;
        
        if (mListener != null) {
            mListener.onRadialDistortionEstimationStarts(this);
        }
        
        List<Point2D> distortedPoints = new ArrayList<>();
        List<Point2D> undistortedPoints = new ArrayList<>();
        //compute total points for samples where homography could be estimated
        int totalPoints = 0;
        for (CameraCalibratorSample sample : mSamples) {
            if (sample.getHomography() != null) {
                totalPoints += sample.getSampledMarkers().size();
            }
        }
        
        double[] qualityScores = null;
        if (mDistortionMethod == RobustEstimatorMethod.PROSAC ||
                mDistortionMethod == RobustEstimatorMethod.PROMedS) {
            qualityScores = new double[totalPoints];
        }
        
        //estimate camera pose for each sample        
        int pointCounter = 0, sampleCounter = 0;
        for (CameraCalibratorSample sample : mSamples) {
            if (sample.getHomography() == null) {
                //homography computation failed, so we cannot compute camera
                //pose for this sample, or use this sample for radial distortion
                //estimation
                continue;
            }
            sample.computeCameraPose(mIntrinsic);

            //transform ideal pattern markers using estimated homography
            List<Point2D> idealPatternMarkers;
            if (sample.getPattern() != null) {
                //use points generated by pattern in sample
                idealPatternMarkers = sample.getPattern().
                        getIdealPoints();                
            } else {
                //use fallback pattern points
                idealPatternMarkers = idealFallbackPatternMarkers;
            }
            
            List<Point2D> transformedIdealPatternMarkers = 
                    sample.getHomography().transformPointsAndReturnNew(idealPatternMarkers);
            
            //transformedIdealPatternMarkers are considered the undistorted 
            //points, because camera follows a pure pinhole model without 
            //distortion and we have transformed the ideal points using a 
            //pure projective homography without distortion. 
            //sample.getSampledMarkers() contains the sampled coordinates using
            //the actual camera, which will be distorted
            
            //the sampled markers are the ones considered to be distorted for
            //radial distortion estimation purposes, because they are obtained
            //directly from the camera
            
            //stack together all distorted and undistorted points from all 
            //samples    
            
            distortedPoints.addAll(sample.getSampledMarkers());
            undistortedPoints.addAll(transformedIdealPatternMarkers);            
            
            int markersSize = transformedIdealPatternMarkers.size();
            
            //if distortion estimator requires quality scores, set them
            if (qualityScores != null &&
                    (mDistortionMethod == RobustEstimatorMethod.PROSAC ||
                    mDistortionMethod == RobustEstimatorMethod.PROMedS)) {

                double sampleQuality = mHomographyQualityScores[sampleCounter];
                
                //assign to all points (markers) in the sample the same sample
                //quality
                for (int i = pointCounter; i < pointCounter + markersSize; i++) {
                    qualityScores[i] = sampleQuality;
                }
                
                pointCounter += markersSize;                        
                sampleCounter++;
            }
        }
        
        //estimate radial distortion
        double avgError = 0.0;
        try {
            mDistortionEstimator.setIntrinsic(mIntrinsic);
            mDistortionEstimator.setPoints(distortedPoints, undistortedPoints);
            mDistortionEstimator.setQualityScores(qualityScores);
        
            RadialDistortion distortion = mDistortionEstimator.estimate();
            
            //add distortion to undistorted points (which are ideal pattern 
            //markers with homography applied)
            List<Point2D> distortedPoints2 = distortion.distort(
                    undistortedPoints);
            
            //set undistorted points obtained after undistorting sampled points
            //to refine homography on next iteration
            for (CameraCalibratorSample sample : mSamples) {
                if (sample.getHomography() == null) {
                    continue;
                }

                //undo distortion of distorted (sampled) points using estimated 
                //distortion

                List<Point2D> undistortedPoints2 = distortion.undistort(
                    sample.getSampledMarkers());
                
                sample.setUndistortedMarkers(undistortedPoints2);
            }
            
            //compare distortedPoints (obtained by using sampled data)
            //with distortedPoints2 (obtained after applying homography to
            //ideal marker points and applying distortion with estimated 
            //distortion)
            Point2D distortedPoint1, distortedPoint2;
            totalPoints = distortedPoints.size();
            int inlierCount = 0;
            for (int i = 0; i < totalPoints; i++) {
                distortedPoint1 = distortedPoints.get(i);
                distortedPoint2 = distortedPoints2.get(i);

                double distance = distortedPoint1.distanceTo(
                        distortedPoint2);
                if (distance < getDistortionEstimatorThreshold()) {
                    avgError += distance;
                    inlierCount++;
                }                
            }
            
            avgError /= (double)inlierCount;
            
            mDistortion = distortion;
            
        } catch(Exception e) {
            throw new CalibrationException(e);
        }
                
        if (mListener != null) {
            mListener.onRadialDistortionEstimationEnds(this, mDistortion);
        }
        
        return avgError;
    }

    /**
     * Returns the camera calibrator method used by this instance.
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
        float lambda = 1.0f / (float)mMaxIterations;
        float partial = 0.5f * mIntrinsicProgress +
                0.5f * mRadialDistortionProgress;
        
        float progress;
        if (!mEstimateRadialDistortion) {
            progress = partial; //we do not iterate if there is no need to 
                                //estimate radial distortion
        } else {
            progress = mIterProgress + lambda * partial;
        }
        
        if (mListener != null) {
            if ((progress - mPreviousNotifiedProgress) > mProgressDelta) {
                mListener.onCalibrateProgressChange(this, progress);
                mPreviousNotifiedProgress = progress;
            }
        }
    }
    
    /**
     * Refreshes listener of distortion estimator
     */
    protected void refreshDistortionEstimatorListener() {
        if (mDistortionEstimatorListener == null) {
            mDistortionEstimatorListener = new RadialDistortionRobustEstimatorListener() {

                @Override
                public void onEstimateStart(
                        RadialDistortionRobustEstimator estimator) {
                    mRadialDistortionProgress = 0.0f;
                    notifyProgress();
                }

                @Override
                public void onEstimateEnd(
                        RadialDistortionRobustEstimator estimator) {
                    mRadialDistortionProgress = 1.0f;
                    notifyProgress();
                }

                @Override
                public void onEstimateNextIteration(
                        RadialDistortionRobustEstimator estimator, 
                        int iteration) { }

                @Override
                public void onEstimateProgressChange(
                        RadialDistortionRobustEstimator estimator, 
                        float progress) {
                    mRadialDistortionProgress = progress;
                    notifyProgress();
                }
            };
        }
        
        try {
            mDistortionEstimator.setListener(mDistortionEstimatorListener);
        } catch (LockedException e) {
            Logger.getLogger(AlternatingCameraCalibrator.class.getName()).log(
                    Level.WARNING, "Could not set radial distortion estimator listener",
                    e);
        }
    }
    
    /**
     * Sets robust estimator method to be used for radial distortion estimation.
     * If method changes, then a new radial distortion estimator is created and
     * configured.
     * @param distortionMethod robust estimator method to be used for
     * radial distortion estimation.
     */
    private void internalSetDistortionMethod(
            RobustEstimatorMethod distortionMethod) {
        //im method changes, recreat estimator
        if (distortionMethod != mDistortionMethod) {
            boolean previousAvailable = mDistortionMethod != null;
            double threshold = 0.0, confidence = 0.0;
            int maxIterations = 0;
            if (previousAvailable) {
                threshold = getDistortionEstimatorThreshold();
                confidence = getDistortionEstimatorConfidence();
                maxIterations = getDistortionEstimatorMaxIterations();
            }
            
            mDistortionEstimator = RadialDistortionRobustEstimator.create(
                    distortionMethod);
            
            //configure new estimator
            refreshDistortionEstimatorListener();
            if (previousAvailable) {
                try {
                    setDistortionEstimatorThreshold(threshold);
                    setDistortionEstimatorConfidence(confidence);
                    setDistortionEstimatorMaxIterations(maxIterations);
                } catch (LockedException e) {
                    Logger.getLogger(
                            AlternatingCameraCalibrator.class.getName()).log(
                            Level.WARNING, "Could not reconfigure distortion estimator", 
                            e);
                }
            }                        
        }
        
        mDistortionMethod = distortionMethod;
    }   
}
