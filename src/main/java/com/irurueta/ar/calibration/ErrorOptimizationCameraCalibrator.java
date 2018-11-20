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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.calibration.estimators.*;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.JacobianEstimator;
import com.irurueta.numerical.MultiVariateFunctionEvaluatorListener;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFunctionEvaluator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Calibrates a camera in order to find its intrinsic parameters and radial
 * distortion by first estimating the intrinsic parameters without accounting
 * for radial distortion and then use an optimization algorithm to minimize
 * error and adjust estimated camera pose, intrinsic parameters and radial
 * distortion parameters.
 * 
 * This class is based on technique described at:
 * Zhengyou Zhang. A Flexible New Technique for Camera Calibration. Technical
 * Report. MSR-TR-98-71. December 2, 1998.
 */
@SuppressWarnings("WeakerAccess")
public class ErrorOptimizationCameraCalibrator extends CameraCalibrator {
    
    /**
     * Default robust estimator method to be used for radial distortion
     * estimation.
     */
    public static final RobustEstimatorMethod DEFAULT_RADIAL_DISTORTION_METHOD =
            RobustEstimatorMethod.PROSAC;
    
    /**
     * Indicates whether an initial radial distortion guess is estimated based
     * on sampled data and estimated camera poses before starting the actual
     * radial distortion optimization process.
     */
    public static final boolean DEFAULT_ESTIMATE_INITIAL_RADIAL_DISTORTION = 
            true;
    
    /**
     * Default maximum number of iterations to be used when adjusting parameters
     * using Levenberg-Marquardt algorithm.
     */
    public static final int DEFAULT_LEVENBERG_MARQUARDT_MAX_ITERS = 1000;
    
    /**
     * Default tolerance to assume that Levenberg-Marquardt algorithm has 
     * reached convergence when adjusting parameters.
     */
    public static final double DEFAULT_LEVENBERG_MARQUARDT_TOLERANCE = 1e-12;
    
    /**
     * Maximum number of iterations to be used when adjusting parameters using 
     * Levenberg-Marquardt algorithm.
     */
    private int mLevenbergMarquardtMaxIters;
    
    /**
     * Tolerance to assume that Levenberg-Marquardt algorithm has reached 
     * convergence when adjusting parameters.
     */
    private double mLevenbergMarquardtTolerance;
    
    /**
     * Robust estimator method to be used for radial distortion estimation.
     */
    private RobustEstimatorMethod mDistortionMethod;
    
    /**
     * Indicates whether an initial radial distortion guess is estimated based
     * on sampled data and estimated camera poses before starting the actual
     * radial distortion optimization process.
     */
    private boolean mEstimateInitialRadialDistortion;
    
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
     * Indicates progress of Levenberg-Marquardt fitting.
     */
    private float mFittingProgress;
    
    /**
     * Previously notified progress.
     */
    private float mPreviousNotifiedProgress;    
    
    /**
     * Array to keep a relation between each point at index i-th and the sample
     * (i.e. view) where it belongs to.
     */
    private int[] mIndexToView;

    
    /**
     * Constructor.
     */
    public ErrorOptimizationCameraCalibrator() {
        super();
        mLevenbergMarquardtMaxIters = DEFAULT_LEVENBERG_MARQUARDT_MAX_ITERS;
        mLevenbergMarquardtTolerance = DEFAULT_LEVENBERG_MARQUARDT_TOLERANCE;
        internalSetDistortionMethod(DEFAULT_RADIAL_DISTORTION_METHOD);
        mEstimateInitialRadialDistortion = 
                DEFAULT_ESTIMATE_INITIAL_RADIAL_DISTORTION;                
    }
    
    /**
     * Constructor.
     * @param pattern 2D pattern to use for calibration.
     * @param samples samples of the pattern taken with the camera to calibrate.
     * @throws IllegalArgumentException if not enough samples are provided.
     */
    public ErrorOptimizationCameraCalibrator(Pattern2D pattern,
            List<CameraCalibratorSample> samples)
            throws IllegalArgumentException {
        super(pattern, samples);
        mLevenbergMarquardtMaxIters = DEFAULT_LEVENBERG_MARQUARDT_MAX_ITERS;
        mLevenbergMarquardtTolerance = DEFAULT_LEVENBERG_MARQUARDT_TOLERANCE;        
        internalSetDistortionMethod(DEFAULT_RADIAL_DISTORTION_METHOD);
        mEstimateInitialRadialDistortion = 
                DEFAULT_ESTIMATE_INITIAL_RADIAL_DISTORTION;        
    }
    
    /**
     * Constructor.
     * @param pattern 2D pattern to use for calibration.
     * @param samples samples of the pattern taken with the camera to calibrate.
     * @param samplesQualityScores quality scores for each sample.
     * @throws IllegalArgumentException if not enough samples are provided or
     * both samples and quality scores do not have the same size.
     */
    public ErrorOptimizationCameraCalibrator(Pattern2D pattern,
            List<CameraCalibratorSample> samples,
            double[] samplesQualityScores) throws IllegalArgumentException {
        super(pattern, samples, samplesQualityScores);
        mLevenbergMarquardtMaxIters = DEFAULT_LEVENBERG_MARQUARDT_MAX_ITERS;
        mLevenbergMarquardtTolerance = DEFAULT_LEVENBERG_MARQUARDT_TOLERANCE;        
        internalSetDistortionMethod(DEFAULT_RADIAL_DISTORTION_METHOD);
        mEstimateInitialRadialDistortion = 
                DEFAULT_ESTIMATE_INITIAL_RADIAL_DISTORTION;        
    }
    
    /**
     * Returns maximum number of iterations to be used when adjusting parameters
     * using Levenberg-Marquardt algorithm.
     * @return maximum number of iterations to be used when adjusting parameters
     * using Levenberg-Marquardt algorithm.
     */
    public int getLevenbergMarquardtMaxIters() {
        return mLevenbergMarquardtMaxIters;
    }
    
    /**
     * Sets maximum number of iterations to be used when adjusting parameters
     * using Levenberg-Marquardt algorithm.
     * @param levenbergMarquardtMaxIters maximum number of iterations to be used
     * when adjusting parameters using Levenberg-Marquardt algorithm.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException if this instance is locked.
     */
    public void setLevenbergMarquardtMaxIters(int levenbergMarquardtMaxIters)
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (levenbergMarquardtMaxIters <= 0) {
            throw new IllegalArgumentException();
        }
        mLevenbergMarquardtMaxIters = levenbergMarquardtMaxIters;
    }
    
    /**
     * Returns tolerance to assume that Levenberg-Marquardt algorithm has
     * reached convergence when adjusting parameters.
     * @return tolerance to assume that Levenberg-Marquardt algorithm has
     * reached convergence when adjusting parameters.
     */
    public double getLevenbergMarquardtTolerance() {
        return mLevenbergMarquardtTolerance;
    }
    
    /**
     * Sets tolerance to assume that Levenberg-Marquardt algorithm has reached
     * convergence when adjusting parameters.
     * @param levenbergMarquardtTolerance tolerance to assume that 
     * Levenberg-Marquardt algorithm has reached convergence when adjusting 
     * parameter.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException if this instance is locked.
     */
    public void setLevenbergMarquardtTolerance(
            double levenbergMarquardtTolerance) throws IllegalArgumentException,
            LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (levenbergMarquardtTolerance <= 0.0) {
            throw new IllegalArgumentException();
        }
        mLevenbergMarquardtTolerance = levenbergMarquardtTolerance;
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
        mRadialDistortionProgress = mFittingProgress = 
                mPreviousNotifiedProgress = 0.0f;
        
        List<Point2D> idealFallbackPatternMarkers = mPattern.getIdealPoints();
        
        try {
            //estimate intrinsic parameters
            estimateIntrinsicParameters(idealFallbackPatternMarkers);

            if (mEstimateRadialDistortion) {
                //estimate radial distortion
                estimateRadialDistortion(idealFallbackPatternMarkers);
            }

            if (mListener != null) {
                mListener.onCalibrateEnd(this);
            }
        } finally {
            mLocked = false;
        }
    }

    /**
     * Returns the camera calibrator method used by this instance.
     * @return the camera calibrator method.
     */        
    @Override
    public CameraCalibratorMethod getMethod() {
        return CameraCalibratorMethod.ERROR_OPTIMIZATION;
    }    

    /**
     * Notifies progress to current listener, if needed.
     */        
    @Override
    protected void notifyProgress() {
        float progress;
        if (mEstimateInitialRadialDistortion) {
            progress = (mRadialDistortionProgress + mIntrinsicProgress + 
                    mFittingProgress) / 3.0f;
        } else {
            progress = 0.5f * mIntrinsicProgress + 0.5f * mFittingProgress;
        }
        
        if (mListener != null) {
            if ((progress - mPreviousNotifiedProgress) > mProgressDelta) {
                mListener.onCalibrateProgressChange(this, progress);
                mPreviousNotifiedProgress = progress;
            }
        }
    }
        
    /**
     * Estimates radial distortion by minimizing the reprojection error by
     * adjusting the camera pose and radial distortion parameters using an
     * optimization algorithm.
     * The initial solution for the optimization algorithm is the estimated
     * camera pose and intrinsic parameters without accounting for radial
     * distortion and radial distortion parameters equal to 0.0.
     * @param idealFallbackPatternMarkers ideal pattern markers coordinates. 
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

        //compute total points for samples where homography could be estimated
        int totalPoints = 0;
        int totalHomographies = 0;
        for (CameraCalibratorSample sample : mSamples) {
            if (sample.getHomography() != null) {
                totalPoints += sample.getSampledMarkers().size();
                totalHomographies++;
            }
        }
        
        mIndexToView = new int[totalPoints];
        
        if (mEstimateInitialRadialDistortion) {
            List<Point2D> distortedPoints = new ArrayList<>();
            List<Point2D> undistortedPoints = new ArrayList<>();
            
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
                    //pose for this sample
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
                        sample.getHomography().transformPointsAndReturnNew(
                        idealPatternMarkers);
                
                distortedPoints.addAll(sample.getSampledMarkers());
                undistortedPoints.addAll(transformedIdealPatternMarkers);
            
                int markersSize = transformedIdealPatternMarkers.size();
                
                //fills array indicanding to which sample (i.e. view) each point
                //belongs to
                Arrays.fill(mIndexToView, pointCounter, 
                        pointCounter + markersSize, sampleCounter);

                //if distortion estimator requires quality scores, set them
                if (qualityScores != null &&
                        (mDistortionMethod == RobustEstimatorMethod.PROSAC ||
                        mDistortionMethod == RobustEstimatorMethod.PROMedS)) {

                    double sampleQuality = mHomographyQualityScores[sampleCounter];

                    //assign to all points (markers) in the sample the same sample
                    //quality
                    for (int i = pointCounter; i < pointCounter + markersSize; 
                            i++) {
                        qualityScores[i] = sampleQuality;
                    }

                    pointCounter += markersSize;                        
                    sampleCounter++;
                }                         
            }                
            
            //estimate radial distortion
            try {
                mDistortionEstimator.setIntrinsic(mIntrinsic);
                mDistortionEstimator.setPoints(distortedPoints, 
                        undistortedPoints);
                mDistortionEstimator.setQualityScores(qualityScores);
        
                mDistortion = mDistortionEstimator.estimate();                
            } catch (Exception e) {
                throw new CalibrationException(e);
            }
        } else {
            
            //estimate camera pose for each sample
            for (CameraCalibratorSample sample : mSamples) {
                if (sample.getHomography() == null) {
                    //homography computation failed, so we cannot compute camera
                    //pose for this sample
                    continue;
                }
                sample.computeCameraPose(mIntrinsic);
            }
            
            //set initial radial distortion as if there was no distortion
            mDistortion = new RadialDistortion(0.0, 0.0);
        }
        
        //optimize cost function to refine camera poses and radial distortion  
            
        try {
            //compute initial parameters to fit a function using 
            //Levenberg-Marquardt
            final double[] initParams = 
                    new double[numParameters(totalHomographies)];
            paramsFromData(initParams);
            //compute x data (input points)
            Matrix x = dataXToMatrix(idealFallbackPatternMarkers);
            //compute y data (output points)
            Matrix y = dataYToMatrix();
            
            //Evaluator for Levenberg-Marquardt fitter. This is in charge of
            //evaluating function to be fitted using current parameters and
            //also in charge of estimating function Jacobian for each point
            //where the function is evaluated
            LevenbergMarquardtMultiVariateFunctionEvaluator evaluator =
                    new LevenbergMarquardtMultiVariateFunctionEvaluator() {
                
                //position of current point being evaluated
                private int mI;
                
                //current point being evaluated
                private double[] mPoint;
                
                //Instance in charge of estimating Jacobian of function being 
                //fitted at current point and for provided parameters. Jacobian
                //is computed by keeping point constant and computing the 
                //partial derivatives for each parameter
                private JacobianEstimator mJacobianEstimator =
                        new JacobianEstimator(
                                new MultiVariateFunctionEvaluatorListener() {

                    //We provide params so that jacobian is computed as the 
                    //partial derivatives respect parameters
                    @Override
                    public void evaluate(double[] params, double[] result) {
                        evaluateFunction(mI, mPoint, params, result);
                    }

                    //Function being fitted is multi variate returning 2D points
                    //(having horizontal and vertical inhomogeneous coordinates)
                    @Override
                    public int getNumberOfVariables() {
                        return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
                    }
                });
                   
                //Function being fitted has as input data 2D points (having
                //horizontal and vertical inhomogeneous coordinates)
                @Override
                public int getNumberOfDimensions() {
                    return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
                }

                //Function being fitted is multi variate returning 2D points
                //(having horizontal and vertical inhomogeneous coordinates)                
                @Override
                public int getNumberOfVariables() {
                    return Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH;
                }

                //Creates array where parameters are stored. This array is 
                //initialized with the parameter values initially used by the
                //Levenberg-Marquardt algorithm
                @Override
                public double[] createInitialParametersArray() {
                    return initParams;
                }

                //Evaluates function to be fitted and computes Jacobian
                @Override
                public void evaluate(int i, double[] point, double[] result, 
                        double[] params, Matrix jacobian) throws Throwable {
                    mI = i;
                    mPoint = point;
                    evaluateFunction(mI, mPoint, params, result);
                    mJacobianEstimator.jacobian(params, jacobian);
                }
            };
            
            //fits function
            double sigma = 1.0;
            LevenbergMarquardtMultiVariateFitter fitter =
                    new LevenbergMarquardtMultiVariateFitter(evaluator, x, y, 
                    sigma);
            fitter.setItmax(mLevenbergMarquardtMaxIters);
            fitter.setTol(mLevenbergMarquardtTolerance);
            
            double[] estimatedParams = fitter.getA();
            //updates camera poses from estimated parameters
            dataFromParams(estimatedParams);
            
            //computes reprojection errors between sampled and ideal data using
            //fitted parameters
            double error = computeReprojectionError(
                    idealFallbackPatternMarkers);
            
            if (mListener != null) {
                mListener.onRadialDistortionEstimationEnds(this, mDistortion);
            }
            
            return error;
            
        } catch (Exception e) {
            throw new CalibrationException(e);
        }
    }
    
    /**
     * Refreshes listener of distortion estimator when robust estimator method
     * is changed for the distortion estimator.
     */
    protected void refreshDistortionEstimatorListener() {
        if (mDistortionEstimatorListener == null) {
            mDistortionEstimatorListener = 
                    new RadialDistortionRobustEstimatorListener() {

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
     * Evaluates provided point and parameters to obtain the distorted points
     * that would be obtained. Levenberg-Marquardt algorithm will iteratively
     * change parameters until the obtained result approximates sampled Y data
     * @param i index of point among all provided data.
     * @param point input point to be evaluated.
     * @param params parameters to evaluate function.
     * @param result result of evaluation.
     */
    private void evaluateFunction(int i, double[] point, double[] params, 
            double[] result) {
        //set data from current params (updates camera poses for each sample - 
        //i.e. view)
        dataFromParams(params);
        
        int numView = mIndexToView[i];
        
        //obtain camera pose for numView
        CameraCalibratorSample sample = mSamples.get(numView);
        PinholeCamera camera = sample.getCamera();
        
        Point3D idealPoint3D = new HomogeneousPoint3D();
        //ideal 3D point is the marker point assumed to be at plane z = 0
        idealPoint3D.setInhomogeneousCoordinates(point[0], point[1], 0.0);
        
        //project ideal point using estimated camera
        Point2D undistortedPoint = camera.project(idealPoint3D);
        
        //add distortion
        Point2D distortedPoint = new HomogeneousPoint2D();
        mDistortion.distort(undistortedPoint, distortedPoint);
        
        result[0] = distortedPoint.getInhomX();
        result[1] = distortedPoint.getInhomY();
    }
    
    /**
     * Converts undistorted points corresponding to ideal pattern markers into
     * a matrix to be used by Levenberg-Marquardt as the input data to be used
     * by the function being fitted.
     * @param idealFallbackPatternMarkers ideal pattern markers coordinates. 
     * These coordinates are used as fallback when a given sample does not have 
     * an associated pattern.
     * @return a matrix.
     * @throws WrongSizeException if no undistorted points are available.
     */
    private Matrix dataXToMatrix(List<Point2D> idealFallbackPatternMarkers) 
            throws WrongSizeException {
        List<Point2D> idealPoints = new ArrayList<>();
        for (CameraCalibratorSample sample : mSamples) {
            List<Point2D> idealPatternMarkers;
            if (sample.getPattern() != null) {
                //use points generated by pattern in sample
                idealPatternMarkers = sample.getPattern().getIdealPoints();                
            } else {
                //use fallback pattern points
                idealPatternMarkers = idealFallbackPatternMarkers;
            }            
            
            idealPoints.addAll(idealPatternMarkers);
        }
        
        int nPoints = idealPoints.size();
        
        Matrix m = new Matrix(nPoints, 
                Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH);
        int i = 0;
        for (CameraCalibratorSample sample : mSamples) {
            List<Point2D> idealPatternMarkers;
            if (sample.getPattern() != null) {
                //use points generated by pattern in sample
                idealPatternMarkers = sample.getPattern().getIdealPoints();                
            } else {
                //use fallback pattern points
                idealPatternMarkers = idealFallbackPatternMarkers;
            }            

            for (Point2D point : idealPatternMarkers) {
                m.setElementAt(i, 0, point.getInhomX());
                m.setElementAt(i, 1, point.getInhomY());
                i++;
            }
        }
        
        return m;
    }
    
    /**
     * Converts sampled distorted points into a matrix to be used by 
     * Levenberg-Marquardt algorithm as the sampled function evaluations.
     * @return a matrix.
     * @throws WrongSizeException if no sampled points are available.
     */
    private Matrix dataYToMatrix() 
            throws WrongSizeException {
        int nPoints = 0;
        for (CameraCalibratorSample sample : mSamples) {
            nPoints += sample.getSampledMarkers().size();
        }
        
        Matrix m = new Matrix(nPoints, 
                Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH);
        int i = 0;
        for (CameraCalibratorSample sample : mSamples) {
            for (Point2D point : sample.getSampledMarkers()) {
                m.setElementAt(i, 0, point.getInhomX());
                m.setElementAt(i, 1, point.getInhomY());                
                i++;
            }
        }

        return m;
    }
    
    /**
     * Sets parameters of function to be fitted using Levenberg-Marquardt
     * algorithm.
     * These parameters will be used as an initial solution and on each 
     * iteration of the Levenberg-Marquardt algorithm.
     * @param params arrays where parameters will be set using current data.
     * @throws RotationException never happens.
     */
    private void paramsFromData(double[] params) throws RotationException {
        int pos = 0;  
        
        //common parameters (intrinsic camera parameters and radial distortion
        //parameters)
        
        //intrinsic parameters
        if (!isZeroSkewness()) {
            //aspect ratio is not known (2 different focal distances) and
            //skewness is not zero
            params[pos] = mIntrinsic.getHorizontalFocalLength();
            pos++;
            params[pos] = mIntrinsic.getVerticalFocalLength();
            pos++;
            params[pos] = mIntrinsic.getSkewness();
            pos++;
        } else {
            //skewness is always zero (so it is not stored in vector)
            params[pos] = mIntrinsic.getHorizontalFocalLength();
            pos++;

            if (!isFocalDistanceAspectRatioKnown()) {
                //focal distances are different, so we also store vertical
                //one
                params[pos] = mIntrinsic.getVerticalFocalLength();
                pos++;
            }
        }

        if (!isPrincipalPointAtOrigin()) {
            //principal point is not zero
            params[pos] = mIntrinsic.getHorizontalPrincipalPoint();
            pos++;
            params[pos] = mIntrinsic.getVerticalPrincipalPoint();
            pos++;
        }
        
        //radial distortion parameters
        double[] kParams = mDistortion.getKParams();
        for (double kParam : kParams) {
            params[pos] = kParam;
            pos++;
        }


        //parameters for each sample (camera rotation and translation)
        for (CameraCalibratorSample sample : mSamples) {
            if (sample.getHomography() == null) {
                continue;
            }
            
            //4 rotation parameters
            AxisRotation3D rot;
            if (sample.getRotation().getType() == 
                    Rotation3DType.AXIS_ROTATION3D) {
                rot = (AxisRotation3D)sample.getRotation();
            } else {
                rot = new AxisRotation3D(sample.getRotation());
            }
            
            params[pos] = rot.getRotationAngle();
            pos++;
            params[pos] = rot.getAxisX();
            pos++;
            params[pos] = rot.getAxisY();
            pos++;
            params[pos] = rot.getAxisZ();
            pos++;
                        
            //3 translation parameters (camera center)
            Point3D center = sample.getCameraCenter();
            params[pos] = center.getInhomX();
            pos++;
            params[pos] = center.getInhomY();
            pos++;
            params[pos] = center.getInhomZ();
            pos++;            
        }         
    }
    
    /**
     * Sets data in samples from parameters values fitted by the 
     * Levenberg-Marquardt algorithm.
     * @param params  vector containing estimated parameters .
     */
    private void dataFromParams(double[] params) {
        int pos = 0;
        
        //intrinsic parameters
        double horizontalFocalLength, verticalFocalLength, skewness,
                horizontalPrincipalPoint, verticalPrincipalPoint;
        if (!isZeroSkewness()) {
            //aspect ratio is not known (2 different focal distances) and
            //skewness is not zero
            horizontalFocalLength = params[pos];
            pos++;
            verticalFocalLength = params[pos];
            pos++;
            skewness = params[pos];
            pos++;
        } else {
            //skewness is always zero (so it is not stored in vector)
            skewness = 0.0;
            horizontalFocalLength = params[pos];
            pos++;

            if (!isFocalDistanceAspectRatioKnown()) {
                //focal distances are different
                verticalFocalLength = params[pos];
                pos++;
            } else {
                //vertical focal distance is related to horizontal one
                //through aspect ratio
                verticalFocalLength = horizontalFocalLength * 
                        getFocalDistanceAspectRatio();
            }
        }

        if (!isPrincipalPointAtOrigin()) {
            //principal point is not zero
            horizontalPrincipalPoint = params[pos];
            pos++;
            verticalPrincipalPoint = params[pos];
            pos++;
        } else {
            //principal point is zero
            horizontalPrincipalPoint = verticalPrincipalPoint = 0.0;
        }
        
        //update intrinsic parameters
        mIntrinsic.setHorizontalFocalLength(horizontalFocalLength);
        mIntrinsic.setVerticalFocalLength(verticalFocalLength);
        mIntrinsic.setSkewness(skewness);
        mIntrinsic.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
        mIntrinsic.setVerticalPrincipalPoint(verticalPrincipalPoint);
        
        //radial distortion parameters
        double[] kParams = mDistortion.getKParams();
        for (int i = 0; i < kParams.length; i++) {
            kParams[i] = params[pos];
            pos++;
        }
        
        //sample parameters
        for (CameraCalibratorSample sample : mSamples) {
            if (sample.getHomography() == null) {
                continue;
            }
            
            //4 rotation parameters
            AxisRotation3D rot;
            if (sample.getRotation().getType() == 
                    Rotation3DType.AXIS_ROTATION3D) {
                rot = (AxisRotation3D)sample.getRotation();
            } else {
                rot = new AxisRotation3D();
                //update sample
                sample.setRotation(rot);
            }
            
            double rotAngle = params[pos];
            pos++;
            double axisX = params[pos];
            pos++;
            double axisY = params[pos];
            pos++;
            double axisZ = params[pos];
            pos++;
            
            rot.setAxisAndRotation(axisX, axisY, axisZ, rotAngle);
            
                        
            //3 translation parameters (camera center)
            double inhomX = params[pos];
            pos++;
            double inhomY = params[pos];
            pos++;
            double inhomZ = params[pos];
            pos++;
            Point3D center = sample.getCameraCenter();
            center.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);            
            
            //update camera
            PinholeCamera camera = sample.getCamera();
            camera.setIntrinsicAndExtrinsicParameters(mIntrinsic, rot, center);
        }          
    }
            
    /**
     * Computes reprojection error taking into account ideal pattern marker
     * points, transforming them using estimated homography, adding to them
     * distortion and comparing them with sampled points.
     * @param idealFallbackPatternMarkers ideal 2D pattern marker points used
     * as fallback in case that a given sample does not have an associated 
     * pattern.
     * @return average reprojection error.
     */
    private double computeReprojectionError(
            List<Point2D> idealFallbackPatternMarkers) {
        //distorted points are the sampled points
        //undistorted points are the ideal pattern marker points projected
        //using current camera pose
        PinholeCamera camera;
        Point2D marker2D;
        Point3D marker3D = Point3D.create();
        Point2D undistortedPoint = Point2D.create();
        int totalPoints = 0;
        Point2D distortedPoint = Point2D.create();
        Point2D sampledPoint;
        double avgError = 0.0;
        for (CameraCalibratorSample sample : mSamples) {
            camera = sample.getCamera();
            if (camera == null) {
                continue;
            }
            
            List<Point2D> idealPatternMarkers;
            if (sample.getPattern() != null) {
                idealPatternMarkers = sample.getPattern().getIdealPoints();
            } else {
                idealPatternMarkers = idealFallbackPatternMarkers;
            }
            
            int pointsPerSample = idealPatternMarkers.size();
            for (int i = 0; i < pointsPerSample; i++) {
                marker2D = idealPatternMarkers.get(i);
                sampledPoint = sample.getSampledMarkers().get(i);
                
                //conver ideal marker point into a 3D point placed in plane
                //z = 0
                marker3D.setInhomogeneousCoordinates(marker2D.getInhomX(), 
                        marker2D.getInhomY(), 0.0);
                
                //project 3D marker point using estimated camera on current 
                //sample (i.e. view)
                camera.project(marker3D, undistortedPoint);
                
                //add distortion to ideal projected point
                mDistortion.distort(undistortedPoint, distortedPoint);
                
                //obtain distance between sampled point and ideal projected
                //point with added distortion
                avgError += sampledPoint.distanceTo(distortedPoint);
                totalPoints++;
            }
        }
        
        avgError /= (double)totalPoints;
        
        return avgError;
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
    
    /**
     * Returns number of parameters of cost function.
     * @param numHomographies number of valid estimated homographies.
     * @return number of parameters of cost function.
     */
    private int numParameters(int numHomographies) {
        //For each homography there are:
        //- 4 rotation parameters (angle and axis coordinates x, y, z)
        //- 3 translation parameters
        //- x intrinsic parameters (depending on settings)
        //- x radial distortion parameters (K params length)
        
        return 7 * numHomographies + numIntrinsicParameters() +
                mDistortion.getKParams().length;
    }
    
    /**
     * Returns number of intrinsic parameters to be taken into account in
     * cost function.
     * @return number of intrinsic parameters to be taken into account in
     * cost function.
     */
    private int numIntrinsicParameters() {
        //if no constraints, there are 5 intrinsic parameters (horizontal
        //focal length, verticla focal length, skewness, horizontal principal
        //point and vertical principal point
        int num = 5; 
        if (isZeroSkewness()) {
            if (isFocalDistanceAspectRatioKnown()) {
                num--;
            }
            num--;
        }
        if (isPrincipalPointAtOrigin()) {
            num -= 2;
        }
        
        return num;
    }    
}
