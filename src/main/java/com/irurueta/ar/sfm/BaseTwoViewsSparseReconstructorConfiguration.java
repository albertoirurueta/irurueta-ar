/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.ar.sfm;

import com.irurueta.ar.epipolar.CorrectorType;
import com.irurueta.ar.epipolar.estimators.FundamentalMatrixEstimatorMethod;
import com.irurueta.ar.epipolar.estimators.FundamentalMatrixRobustEstimator;
import com.irurueta.ar.epipolar.estimators.PROSACFundamentalMatrixRobustEstimator;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.io.Serializable;

/**
 * Base class containing configuration for a two view sparse re-constructor.
 *
 * @param <T> an actual implementation of a configuration class.
 */
public abstract class BaseTwoViewsSparseReconstructorConfiguration<
        T extends BaseTwoViewsSparseReconstructorConfiguration<T>> implements Serializable {
    /**
     * Default robust fundamental matrix estimator method.
     * This is only used when general scenes are allowed.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD =
            RobustEstimatorMethod.PROSAC;

    /**
     * Default non-robust fundamental matrix estimator method used internally
     * within a robust estimator.
     * This is only used when general scenes are allowed.
     */
    public static final FundamentalMatrixEstimatorMethod DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD =
            FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM;

    /**
     * Indicates that estimated fundamental matrix is refined by default using
     * all found inliers.
     * This is only used when general scenes are allowed.
     */
    public static final boolean DEFAULT_REFINE_FUNDAMENTAL_MATRIX = true;

    /**
     * Indicates that fundamental matrix covariance is kept by default after the
     * estimation.
     * This is only used when general scenes are allowed.
     */
    public static final boolean DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE = false;

    /**
     * Default confidence of robustly estimated fundamental matrix. By default,
     * this is 99%.
     * This is only used when general scenes are allowed.
     */
    public static final double DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE =
            FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE;

    /**
     * Default maximum number of iterations to make while robustly estimating
     * fundamental matrix. By default, this is 5000 iterations.
     * This is only used when general scenes are allowed.
     */
    public static final int DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS =
            FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS;

    /**
     * Default threshold to determine whether samples for robust fundamental
     * matrix estimation are inliers or not.
     * This is only used when general scenes are allowed.
     */
    public static final double DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD =
            PROSACFundamentalMatrixRobustEstimator.DEFAULT_THRESHOLD;

    /**
     * Default value indicating that inlier data is kept after robust
     * fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     */
    public static final boolean DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS = true;

    /**
     * Default value indicating that residual data is kept after robust
     * fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     */
    public static final boolean DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS = true;

    /**
     * Default method to use for initial estimation of cameras.
     */
    public static final InitialCamerasEstimatorMethod DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD =
            InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX;

    /**
     * Indicates whether an homogeneous point triangulator is used for point
     * triangulation when Dual Absolute Quadric (DAQ) camera initialization is
     * used.
     */
    public static final boolean DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR = true;

    /**
     * Default aspect ratio for initial cameras.
     */
    public static final double DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO = 1.0;

    /**
     * Default horizontal principal point value to use for initial cameras
     * estimation using Dual Image of Absolute Conic (DIAC) or Dual Absolute
     * Quadric (DAQ) methods.
     */
    public static final double DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_X = 0.0;

    /**
     * Default vertical principal point value to use for initial cameras
     * estimation using Dual Image of Absolute Conic (DIAC) or Dual Absolute
     * Quadric (DAQ) methods.
     */
    public static final double DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_Y = 0.0;

    /**
     * Default corrector type to use for point triangulation when initial
     * cameras are being estimated using either Dual Image of Absolute Conic
     * (DIAC), Dual Absolute Quadric (DAQ) or essential matrix methods.
     */
    public static final CorrectorType DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE = CorrectorType.SAMPSON_CORRECTOR;

    /**
     * Default value indicating whether valid triangulated points are marked
     * during initial cameras estimation using either Dual Image of Absolute
     * Conic (DIAC) or essential matrix methods.
     */
    public static final boolean DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS = true;

    /**
     * Indicates whether a general (points are laying in a general 3D position)
     * scene is allowed.
     * When true, an initial geometry estimation is attempted for general
     * points.
     */
    public static final boolean DEFAULT_ALLOW_GENERAL_SCENE = true;

    /**
     * Indicates whether a planar (points laying in a 3D plane) scene is
     * allowed.
     * When true, an initial geometry estimation is attempted for planar points.
     */
    public static final boolean DEFAULT_ALLOW_PLANAR_SCENE = true;

    /**
     * Default robust planar homography estimator method.
     * This is only used when planar scenes are allowed.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD =
            RobustEstimatorMethod.PROMEDS;

    /**
     * Indicates that planar homography is refined by default using all found
     * inliers.
     * This is only used when planar scenes are allowed.
     */
    public static final boolean DEFAULT_REFINE_PLANAR_HOMOGRAPHY = true;

    /**
     * Indicates that planar homography covariance is kept by default after
     * estimation.
     * This is only used when planar scenes are allowed.
     */
    public static final boolean DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE = false;

    /**
     * Default confidence of robustly estimated planar homography. By default,
     * this is 99%.
     * This is only used when planar scenes are allowed.
     */
    public static final double DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE =
            ProjectiveTransformation2DRobustEstimator.DEFAULT_CONFIDENCE;

    /**
     * Default maximum number of iterations to make while robustly estimating
     * planar homography. By default, this is 5000 iterations.
     * This is only used when planar scenes are allowed.
     */
    public static final int DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS =
            ProjectiveTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS;

    /**
     * Default threshold to determine whether samples for robust projective
     * 2D transformation estimation are inliers or not.
     * This is only used when planar scenes are allowed.
     */
    public static final double DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD = 1e-3;

    /**
     * Default value indicating that inlier data is kept after robust planar
     * homography estimation.
     * This is only used when planar scenes are allowed.
     */
    public static final boolean DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS = true;

    /**
     * Default value indicating that residual data is kept after robust planar
     * homography estimation.
     * This is only used when planar scenes are allowed.
     */
    public static final boolean DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS = true;

    /**
     * Method to use for non-robust fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     */
    private FundamentalMatrixEstimatorMethod mNonRobustFundamentalMatrixEstimatorMethod =
            DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD;

    /**
     * Method to use for robust fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     */
    private RobustEstimatorMethod mRobustFundamentalMatrixEstimatorMethod =
            DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD;

    /**
     * Indicates whether estimated fundamental matrix is refined among all found
     * inliers.
     * This is only used when general scenes are allowed.
     */
    private boolean refineFundamentalMatrix = DEFAULT_REFINE_FUNDAMENTAL_MATRIX;

    /**
     * Indicates whether covariance of estimated fundamental matrix is kept
     * after the estimation.
     * This is only used when general scenes are allowed.
     */
    private boolean keepFundamentalMatrixCovariance = DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE;

    /**
     * Confidence of robustly estimated fundamental matrix.
     * This is only used when general scenes are allowed.
     */
    private double fundamentalMatrixConfidence = DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE;

    /**
     * Maximum number of iterations to robustly estimate fundamental matrix.
     * This is only used when general scenes are allowed.
     */
    private int fundamentalMatrixMaxIterations = DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS;

    /**
     * Threshold to determine whether samples for robust fundamental matrix
     * estimation are inliers or not.
     * This is only used when general scenes are allowed.
     */
    private double fundamentalMatrixThreshold = DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD;

    /**
     * Indicates whether inliers must be kept during robust fundamental matrix
     * estimation.
     * This is only used when general scenes are allowed.
     */
    private boolean fundamentalMatrixComputeAndKeepInliers = DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS;

    /**
     * Indicates whether residuals must be computed and kept during robust
     * fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     */
    private boolean fundamentalMatrixComputeAndKeepResiduals = DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS;

    /**
     * Method to use for initial estimation of cameras.
     */
    private InitialCamerasEstimatorMethod initialCamerasEstimatorMethod = DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD;

    /**
     * Indicates whether an homogeneous point triangulator is used for point
     * triangulation when Dual Absolute Quadric (DAQ) camera initialization is
     * used.
     */
    private boolean daqUseHomogeneousPointTriangulator = DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR;

    /**
     * Aspect ratio for initial cameras.
     */
    private double initialCamerasAspectRatio = DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO;

    /**
     * Horizontal principal point value to use for initial cameras estimation
     * using Dual Image of Absolute Conic (DIAC) or Dual Absolute Quadric (DAQ)
     * methods.
     */
    private double principalPointX = DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_X;

    /**
     * Vertical principal point value to use for initial cameras estimation
     * using Dual Image of Absolute Conic (DIAC) or Dual Absolute Quadric (DAQ)
     * methods.
     */
    private double principalPointY = DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_Y;

    /**
     * Corrector type to use for point triangulation when initial cameras are
     * being estimated using either Dual Image of Absolute Conic (DIAC) or
     * essential matrix methods or null if no corrector is used.
     */
    private CorrectorType initialCamerasCorrectorType = DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE;

    /**
     * Value indicating whether valid triangulated points are marked during
     * initial cameras estimation using either Dual Image of Absolute Conic
     * (DIAC) or essential matrix methods.
     */
    private boolean initialCamerasMarkValidTriangulatedPoints = DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS;

    /**
     * Intrinsic parameters of first camera estimated using the essential matrix
     * method.
     */
    private PinholeCameraIntrinsicParameters initialIntrinsic1;

    /**
     * Intrinsic parameters of second camera estimated using the essential
     * matrix method.
     */
    private PinholeCameraIntrinsicParameters initialIntrinsic2;

    /**
     * Indicates whether a general scene (points laying in a general 3D
     * position) is allowed.
     * When true, an initial geometry estimation is attempted for general
     * points.
     */
    private boolean allowGeneralScene = DEFAULT_ALLOW_GENERAL_SCENE;

    /**
     * Indicates whether a planar scene (points laying in a 3D plane) is allowed.
     * When true, an initial geometry estimation is attempted for planar points.
     */
    private boolean allowPlanarScene = DEFAULT_ALLOW_PLANAR_SCENE;

    /**
     * Robust method to use for planar homography estimation.
     * This is only used when planar scenes are allowed.
     */
    private RobustEstimatorMethod robustPlanarHomographyEstimatorMethod =
            DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD;

    /**
     * Indicates whether planar homography is refined using all found inliers or
     * not.
     * This is only used when planar scenes are allowed.
     */
    private boolean refinePlanarHomography = DEFAULT_REFINE_PLANAR_HOMOGRAPHY;

    /**
     * Indicates whether planar homography covariance is kept after estimation.
     * This is only used when planar scenes are allowed.
     */
    private boolean keepPlanarHomographyCovariance = DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE;

    /**
     * Confidence of robustly estimated planar homography. By default, this is
     * 99%.
     * This is only used when planar scenes are allowed.
     */
    private double planarHomographyConfidence = DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE;

    /**
     * Maximum number of iterations to make while robustly estimating planar
     * homography. By default, this is 5000.
     * This is only used when planar scenes are allowed.
     */
    private int planarHomographyMaxIterations = DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS;

    /**
     * Threshold to determine whether samples for robust projective 2D
     * transformation estimation are inliers or not.
     */
    private double planarHomographyThreshold = DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD;

    /**
     * Value indicating that inlier data is kept after robust planar homography
     * estimation.
     * This is only used when planar scenes are allowed.
     */
    private boolean planarHomographyComputeAndKeepInliers = DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS;

    /**
     * Value indicating that residual data is kept after robust planar
     * homography estimation.
     * This is only used when planar scenes are allowed.
     */
    private boolean planarHomographyComputeAndKeepResiduals = DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS;

    /**
     * Constructor.
     */
    protected BaseTwoViewsSparseReconstructorConfiguration() {
    }

    /**
     * Gets method to use for non-robust fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     *
     * @return method to use for non-robust fundamental matrix estimation.
     */
    public FundamentalMatrixEstimatorMethod getNonRobustFundamentalMatrixEstimatorMethod() {
        return mNonRobustFundamentalMatrixEstimatorMethod;
    }

    /**
     * Sets method to use for non-robust fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     *
     * @param method method to use for non-robust fundamental matrix estimation.
     * @return this instance so that method can be easily chained.
     */
    public T setNonRobustFundamentalMatrixEstimatorMethod(final FundamentalMatrixEstimatorMethod method) {
        mNonRobustFundamentalMatrixEstimatorMethod = method;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets method to use for robust fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     *
     * @return method to use for robust fundamental matrix estimation.
     */
    public RobustEstimatorMethod getRobustFundamentalMatrixEstimatorMethod() {
        return mRobustFundamentalMatrixEstimatorMethod;
    }

    /**
     * Sets method to use for robust fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     *
     * @param method method to use for robust fundamental matrix estimation.
     * @return this instance so that method can be easily chained.
     */
    public T setRobustFundamentalMatrixEstimatorMethod(final RobustEstimatorMethod method) {
        mRobustFundamentalMatrixEstimatorMethod = method;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether estimated fundamental matrix is refined among all found
     * inliers.
     * This is only used when general scenes are allowed.
     *
     * @return true if fundamental matrix is refined, false otherwise.
     */
    public boolean isFundamentalMatrixRefined() {
        return refineFundamentalMatrix;
    }

    /**
     * Specifies whether estimated fundamental matrix is refined among all found
     * inliers.
     * This is only used when general scenes are allowed.
     *
     * @param refineFundamentalMatrix true if fundamental matrix is refined,
     *                                false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixRefined(final boolean refineFundamentalMatrix) {
        this.refineFundamentalMatrix = refineFundamentalMatrix;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether covariance of estimated fundamental matrix is kept
     * after the estimation.
     * This is only used when general scenes are allowed.
     *
     * @return true if covariance is kept, false otherwise.
     */
    public boolean isFundamentalMatrixCovarianceKept() {
        return keepFundamentalMatrixCovariance;
    }

    /**
     * Specifies whether covariance of estimated fundamental matrix is kept
     * after the estimation.
     * This is only used when general scenes are allowed.
     *
     * @param keepFundamentalMatrixCovariance true if covariance is kept, false
     *                                        otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixCovarianceKept(final boolean keepFundamentalMatrixCovariance) {
        this.keepFundamentalMatrixCovariance = keepFundamentalMatrixCovariance;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets confidence of robustly estimated fundamental matrix.
     * This is only used when general scenes are allowed.
     *
     * @return confidence of robustly estimated fundamental matrix.
     */
    public double getFundamentalMatrixConfidence() {
        return fundamentalMatrixConfidence;
    }

    /**
     * Sets confidence of robustly estimated fundamental matrix.
     * This is only used when general scenes are allowed.
     *
     * @param fundamentalMatrixConfidence confidence of robustly estimated
     *                                    fundamental matrix.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixConfidence(final double fundamentalMatrixConfidence) {
        this.fundamentalMatrixConfidence = fundamentalMatrixConfidence;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets maximum number of iterations to robustly estimate fundamental
     * matrix.
     * This is only used when general scenes are allowed.
     *
     * @return maximum number of iterations to robustly estimate fundamental
     * matrix.
     */
    public int getFundamentalMatrixMaxIterations() {
        return fundamentalMatrixMaxIterations;
    }

    /**
     * Sets maximum number of iterations to robustly estimate fundamental
     * matrix.
     * This is only used when general scenes are allowed.
     *
     * @param fundamentalMatrixMaxIterations maximum number of iterations to
     *                                       robustly estimate fundamental matrix.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixMaxIterations(final int fundamentalMatrixMaxIterations) {
        this.fundamentalMatrixMaxIterations = fundamentalMatrixMaxIterations;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets threshold to determine whether samples for robust fundamental matrix
     * estimation are inliers or not.
     * This is only used when general scenes are allowed.
     *
     * @return threshold to determine whether samples for robust fundamental
     * matrix estimation are inliers or not.
     */
    public double getFundamentalMatrixThreshold() {
        return fundamentalMatrixThreshold;
    }

    /**
     * Sets threshold to determine whether samples for robust fundamental matrix
     * estimation are inliers or not.
     * This is only used when general scenes are allowed.
     *
     * @param fundamentalMatrixThreshold threshold to determine whether samples
     *                                   for robust fundamental matrix estimation are inliers or not.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixThreshold(final double fundamentalMatrixThreshold) {
        this.fundamentalMatrixThreshold = fundamentalMatrixThreshold;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether inliers must be kept during robust fundamental matrix
     * estimation.
     * This is only used when general scenes are allowed.
     *
     * @return true if inliers must be kept during robust fundamental matrix
     * estimation, false otherwise.
     */
    public boolean getFundamentalMatrixComputeAndKeepInliers() {
        return fundamentalMatrixComputeAndKeepInliers;
    }

    /**
     * Specifies whether inliers must be kept during robust fundamental matrix
     * estimation.
     * This is only used when general scenes are allowed.
     *
     * @param fundamentalMatrixComputeAndKeepInliers true if inliers must be
     *                                               kept during robust fundamental matrix estimation,
     *                                               false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixComputeAndKeepInliers(final boolean fundamentalMatrixComputeAndKeepInliers) {
        this.fundamentalMatrixComputeAndKeepInliers = fundamentalMatrixComputeAndKeepInliers;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether residuals must be computed and kept during robust
     * fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     *
     * @return true if residuals must be computed and kept, false otherwise.
     */
    public boolean getFundamentalMatrixComputeAndKeepResiduals() {
        return fundamentalMatrixComputeAndKeepResiduals;
    }

    /**
     * Specifies whether residuals must be computed and kept during robust
     * fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     *
     * @param fundamentalMatrixComputeAndKeepResiduals true if residuals must be
     *                                                 computed and kept, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixComputeAndKeepResiduals(final boolean fundamentalMatrixComputeAndKeepResiduals) {
        this.fundamentalMatrixComputeAndKeepResiduals = fundamentalMatrixComputeAndKeepResiduals;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets method to use for initial estimation of cameras.
     *
     * @return method to use for initial estimation of cameras.
     */
    public InitialCamerasEstimatorMethod getInitialCamerasEstimatorMethod() {
        return initialCamerasEstimatorMethod;
    }

    /**
     * Sets method to use for initial estimation of cameras.
     *
     * @param method method to use for initial estimation of cameras.
     * @return this instance so that method can be easily chained.
     */
    public T setInitialCamerasEstimatorMethod(final InitialCamerasEstimatorMethod method) {
        initialCamerasEstimatorMethod = method;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether an homogeneous point triangulator is used for point
     * triangulation when Dual Absolute Quadric (DAQ) camera initialization is
     * used.
     *
     * @return true if homogeneous point triangulator is used, false if an
     * inhomogeneous point triangulator is used instead.
     */
    public boolean getDaqUseHomogeneousPointTriangulator() {
        return daqUseHomogeneousPointTriangulator;
    }

    /**
     * Specifies whether an homogeneous point triangulator is used for point
     * triangulation when Dual Absolute Quadric (DAQ) camera initialization is
     * used.
     *
     * @param daqUseHomogeneousPointTriangulator true if homogeneous point
     *                                           triangulator is used, false if an inhomogeneous point
     *                                           triangulator is used instead.
     * @return this instance so that method can be easily chained.
     */
    public T setDaqUseHomogeneousPointTriangulator(final boolean daqUseHomogeneousPointTriangulator) {
        this.daqUseHomogeneousPointTriangulator = daqUseHomogeneousPointTriangulator;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets aspect ratio for initial cameras estimation using DAQ or DIAC
     * methods.
     *
     * @return aspect ratio for initial cameras using DAQ or DIAC methods.
     */
    public double getInitialCamerasAspectRatio() {
        return initialCamerasAspectRatio;
    }

    /**
     * Sets aspect ratio for initial cameras using DAQ or DIAC methods.
     *
     * @param initialCamerasAspectRatio aspect ratio for initial cameras using
     *                                  DAQ or DIAC methods.
     * @return this instance so that method can be easily chained.
     */
    public T setInitialCamerasAspectRatio(final double initialCamerasAspectRatio) {
        this.initialCamerasAspectRatio = initialCamerasAspectRatio;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets horizontal principal point value to use for initial cameras
     * estimation using DIAC or DAQ methods.
     *
     * @return horizontal principal point value to use for initial cameras
     * estimation using DIAC or DAQ methods.
     */
    public double getPrincipalPointX() {
        return principalPointX;
    }

    /**
     * Sets horizontal principal point value to use for initial cameras
     * estimation using DIAC or DAQ methods.
     *
     * @param principalPointX horizontal principal point value to use for
     *                        initial cameras estimation using DIAC or DAQ methods.
     * @return this instance so that method can be easily chained.
     */
    public T setPrincipalPointX(final double principalPointX) {
        this.principalPointX = principalPointX;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets vertical principal point value to use for initial cameras estimation
     * using DIAC or DAQ methods.
     *
     * @return vertical principal point value to use for initial cameras
     * estimation using DIAC or DAQ methods.
     */
    public double getPrincipalPointY() {
        return principalPointY;
    }

    /**
     * Sets vertical principal point value to use for initial cameras estimation
     * using DIAC or DAQ methods.
     *
     * @param principalPointY vertical principal point value to use for
     *                        initial cameras estimation using DIAC or DAQ methods.
     * @return this instance so that method can be easily chained.
     */
    public T setPrincipalPointY(final double principalPointY) {
        this.principalPointY = principalPointY;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets corrector type to use for point triangulation when initial cameras
     * are being estimated using either DIAC or essential matrix methods or null
     * if no corrector is used.
     *
     * @return corrector type to use for point triangulation when initial
     * cameras are being estimated using either DIAC or essential matrix methods
     * or null if no corrector is used.
     */
    public CorrectorType getInitialCamerasCorrectorType() {
        return initialCamerasCorrectorType;
    }

    /**
     * Sets corrector type to use for point triangulation when initial cameras
     * are being estimated using either DIAC or essential matrix methods or null
     * if no corrector is used.
     *
     * @param type corrector type to use for point triangulation when initial
     *             cameras are being estimated using either DIAC or essential matrix methods
     *             or null if no corrector is used.
     * @return this instance so that method can be easily chained.
     */
    public T setInitialCamerasCorrectorType(final CorrectorType type) {
        initialCamerasCorrectorType = type;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value indicating whether valid triangulated points are marked during
     * initial cameras estimation using either DIAC or essential matrix methods.
     *
     * @return value indicating whether valid triangulated points are marked
     * during initial cameras estimation using either DIAC or essential matrix
     * methods.
     */
    public boolean getInitialCamerasMarkValidTriangulatedPoints() {
        return initialCamerasMarkValidTriangulatedPoints;
    }

    /**
     * Sets value indicating whether valid triangulated points are marked during
     * initial cameras estimation using either DIAC or essential matrix methods.
     *
     * @param initialCamerasMarkValidTriangulatedPoints value indicating whether
     *                                                  valid triangulated points are marked during
     *                                                  initial cameras estimation using either DIAC or
     *                                                  essential matrix methods.
     * @return this instance so that method can be easily chained.
     */
    public T setInitialCamerasMarkValidTriangulatedPoints(final boolean initialCamerasMarkValidTriangulatedPoints) {
        this.initialCamerasMarkValidTriangulatedPoints = initialCamerasMarkValidTriangulatedPoints;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets intrinsic parameters of first camera estimated using the essential
     * matrix method.
     *
     * @return intrinsic parameters of first camera estimated using the
     * essential matrix method.
     */
    public PinholeCameraIntrinsicParameters getInitialIntrinsic1() {
        return initialIntrinsic1;
    }

    /**
     * Sets intrinsic parameters of first camera estimated using the essential
     * matrix method.
     *
     * @param initialIntrinsic1 intrinsic parameters of first camera estimated
     *                          using the essential matrix method.
     * @return this instance so that method can be easily chained.
     */
    public T setInitialIntrinsic1(final PinholeCameraIntrinsicParameters initialIntrinsic1) {
        this.initialIntrinsic1 = initialIntrinsic1;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets intrinsic parameters of second camera estimated using the essential
     * matrix method.
     *
     * @return intrinsic parameters of second camera estimated using the
     * essential matrix method.
     */
    public PinholeCameraIntrinsicParameters getInitialIntrinsic2() {
        return initialIntrinsic2;
    }

    /**
     * Sets intrinsic parameters of second camera estimated using the essential
     * matrix method.
     *
     * @param initialIntrinsic2 intrinsic parameters of second camera estimated
     *                          using the essential matrix method.
     * @return this instance so that method can be easily chained.
     */
    public T setInitialIntrinsic2(final PinholeCameraIntrinsicParameters initialIntrinsic2) {
        this.initialIntrinsic2 = initialIntrinsic2;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether a general scene (points laying in a general 3D
     * position) is allowed.
     * When true, an initial geometry estimation is attempted for general
     * points.
     *
     * @return true if general scene is allowed, false otherwise.
     */
    public boolean isGeneralSceneAllowed() {
        return allowGeneralScene;
    }

    /**
     * Specifies whether a general scene (points laying in a general 3D
     * position) is allowed.
     * When true, an initial geometry estimation is attempted for general
     * points.
     *
     * @param allowGeneralScene true if general scene is allowed, false
     *                          otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setGeneralSceneAllowed(final boolean allowGeneralScene) {
        this.allowGeneralScene = allowGeneralScene;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether a planar scene (points laying in a 3D plane) is allowed
     * or not.
     * When true, an initial geometry estimation is attempted for planar points.
     *
     * @return true if planar scene is allowed, false otherwise.
     */
    public boolean isPlanarSceneAllowed() {
        return allowPlanarScene;
    }

    /**
     * Specifies whether a planar scene (points laying in a 3D plane) is allowed
     * or not.
     * When true, an initial geometry estimation is attempted for planar points.
     *
     * @param allowPlanarScene true if planar scene is allowed, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarSceneAllowed(final boolean allowPlanarScene) {
        this.allowPlanarScene = allowPlanarScene;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets robust method to use for planar homography estimation.
     * This is only used when planar scenes are allowed.
     *
     * @return robust method to use for planar homography estimation.
     */
    public RobustEstimatorMethod getRobustPlanarHomographyEstimatorMethod() {
        return robustPlanarHomographyEstimatorMethod;
    }

    /**
     * Sets robust method to use for planar homography estimation.
     * This is only used when planar scenes are allowed.
     *
     * @param robustPlanarHomographyEstimatorMethod robust method to use for
     *                                              planar homography estimation.
     * @return this instance so that method can be easily chained.
     */
    public T setRobustPlanarHomographyEstimatorMethod(
            final RobustEstimatorMethod robustPlanarHomographyEstimatorMethod) {
        this.robustPlanarHomographyEstimatorMethod = robustPlanarHomographyEstimatorMethod;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether planar homography is refined using all found inliers or
     * not.
     * This is only used when planar scenes are allowed.
     *
     * @return true if planar homography is refined, false otherwise.
     */
    public boolean isPlanarHomographyRefined() {
        return refinePlanarHomography;
    }

    /**
     * Specifies whether planar homography is refined using all found inliers or
     * not.
     * This is only used when planar scenes are allowed.
     *
     * @param refinePlanarHomography true if planar homography must be refined,
     *                               false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyRefined(final boolean refinePlanarHomography) {
        this.refinePlanarHomography = refinePlanarHomography;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether planar homography covariance is kept after estimation.
     * This is only used when planar scenes are allowed.
     *
     * @return true if planar homography covariance is kept, false otherwise.
     */
    public boolean isPlanarHomographyCovarianceKept() {
        return keepPlanarHomographyCovariance;
    }

    /**
     * Specifies whether planar homography covariance is kept after estimation.
     * This is only used when planar scenes are allowed.
     *
     * @param keepPlanarHomographyCovariance true if planar homography
     *                                       covariance is kept, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyCovarianceKept(final boolean keepPlanarHomographyCovariance) {
        this.keepPlanarHomographyCovariance = keepPlanarHomographyCovariance;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets confidence of robustly estimated planar homography. By default, this
     * is 99%.
     * This is only used when planar scenes are allowed.
     *
     * @return confidence of robustly estimated planar homography.
     */
    public double getPlanarHomographyConfidence() {
        return planarHomographyConfidence;
    }

    /**
     * Sets confidence of robustly estimated planar homography. By default, this
     * is 99%.
     * This is only used when planar scenes are allowed.
     *
     * @param planarHomographyConfidence confidence of robustly estimated planar
     *                                   homography.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyConfidence(final double planarHomographyConfidence) {
        this.planarHomographyConfidence = planarHomographyConfidence;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets maximum number of iterations to make while robustly estimating
     * planar homography. By default, this is 5000.
     * This is only used when planar scenes are allowed.
     *
     * @return maximum number of iterations to make while robustly estimating
     * planar homography.
     */
    public int getPlanarHomographyMaxIterations() {
        return planarHomographyMaxIterations;
    }

    /**
     * Sets maximum number of iterations to make while robustly estimating
     * planar homography. By default, this is 5000.
     * This is only used when planar scenes are allowed.
     *
     * @param planarHomographyMaxIterations maximum number of iterations to make
     *                                      while robustly estimating planar homography.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyMaxIterations(final int planarHomographyMaxIterations) {
        this.planarHomographyMaxIterations = planarHomographyMaxIterations;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets threshold to determine whether samples for robust projective 2D
     * transformation estimation are inliers or not.
     * This is only used when planar scenes are allowed.
     *
     * @return threshold to robustly estimate projective 2D transformation.
     */
    public double getPlanarHomographyThreshold() {
        return planarHomographyThreshold;
    }

    /**
     * Sets threshold to determine whether samples for robust projective 2D
     * transformation estimation are inliers or not.
     * This is only used when planar scenes are allowed.
     *
     * @param planarHomographyThreshold threshold to robustly estimate
     *                                  projective 2D transformation.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyThreshold(final double planarHomographyThreshold) {
        this.planarHomographyThreshold = planarHomographyThreshold;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value indicating that inlier data is kept after robust planar
     * homography estimation.
     * This is only used when planar scenes are allowed.
     *
     * @return true if inlier data is kept, false otherwise.
     */
    public boolean getPlanarHomographyComputeAndKeepInliers() {
        return planarHomographyComputeAndKeepInliers;
    }

    /**
     * Specifies whether inlier data is kept after robust planar homography
     * estimation.
     * This is only used when planar scenes are allowed.
     *
     * @param planarHomographyComputeAndKeepInliers true if inlier data is kept,
     *                                              false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyComputeAndKeepInliers(final boolean planarHomographyComputeAndKeepInliers) {
        this.planarHomographyComputeAndKeepInliers = planarHomographyComputeAndKeepInliers;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value indicating that residual data is kept after robust planar
     * homography estimation.
     * This is only used when planar scenes are allowed.
     *
     * @return true if residual data is kept, false otherwise.
     */
    public boolean getPlanarHomographyComputeAndKeepResiduals() {
        return planarHomographyComputeAndKeepResiduals;
    }

    /**
     * Sets value indicating that residual data is kept after robust planar
     * homography estimation.
     * This is only used when planar scenes are allowed.
     *
     * @param planarHomographyComputeAndKeepResiduals true if residual data is
     *                                                kept, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyComputeAndKeepResiduals(final boolean planarHomographyComputeAndKeepResiduals) {
        this.planarHomographyComputeAndKeepResiduals = planarHomographyComputeAndKeepResiduals;
        //noinspection unchecked
        return (T) this;
    }
}
