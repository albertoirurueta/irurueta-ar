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
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.estimators.EPnPPointCorrespondencePinholeCameraEstimator;
import com.irurueta.geometry.estimators.PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator;
import com.irurueta.geometry.estimators.PinholeCameraRobustEstimator;
import com.irurueta.geometry.estimators.ProjectiveTransformation2DRobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.io.Serializable;

/**
 * Base class containing configuration for a sparse re-constructor supporting multiple views.
 *
 * @param <T> an actual implementation of a configuration class.
 */
public abstract class BaseSparseReconstructorConfiguration<T extends BaseSparseReconstructorConfiguration<T>> implements
        Serializable {

    /**
     * Default robust fundamental matrix estimator method.
     * This is only used when general scenes are allowed.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD =
            RobustEstimatorMethod.PROSAC;

    /**
     * Default non robust fundamental matrix estimator method used internally within a robust estimator.
     * This is only used when general scenes are allowed.
     */
    public static final FundamentalMatrixEstimatorMethod DEFAULT_NON_ROBUST_FUNDAMENTAL_MATRIX_ESTIMATOR_METHOD =
            FundamentalMatrixEstimatorMethod.SEVEN_POINTS_ALGORITHM;

    /**
     * Indicates that estimated fundamental matrix is refined by default using all found inliers.
     * This is only used when general scenes are allowed.
     */
    public static final boolean DEFAULT_REFINE_FUNDAMENTAL_MATRIX = true;

    /**
     * Indicates that fundamental matrix covariance is kept by default after the estimation.
     * This is only used when general scenes are allowed.
     */
    public static final boolean DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE = false;

    /**
     * Default confidence of robustly estimated fundamental matrix. By default this is 99%.
     * This is only used when general scenes are allowed.
     */
    public static final double DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE =
            FundamentalMatrixRobustEstimator.DEFAULT_CONFIDENCE;

    /**
     * Default maximum number of iterations to make while robustly estimating fundamental matrix.
     * By default this is 5000 iterations. This is only used when general scenes are allowed.
     */
    public static final int DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS =
            FundamentalMatrixRobustEstimator.DEFAULT_MAX_ITERATIONS;

    /**
     * Default threshold to determine whether samples for robust fundamental matrix estimation are inliers or not.
     * This is only used when general scenes are allowed.
     */
    public static final double DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD =
            PROSACFundamentalMatrixRobustEstimator.DEFAULT_THRESHOLD;

    /**
     * Default value indicating that inlier data is kept after robust fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     */
    public static final boolean DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS = true;

    /**
     * Default value indicating that residual data is kept after robust fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     */
    public static final boolean DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS = true;

    /**
     * Default method to use for initial cameras estimation.
     */
    public static final InitialCamerasEstimatorMethod DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD =
            InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC_AND_ESSENTIAL_MATRIX;

    /**
     * Indicates whether an homogeneous point triangulator is used for point triangulation when Dual Absolute Quadric
     * (DAQ) camera initialization is used.
     */
    public static final boolean DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR = true;

    /**
     * Default aspect ratio for initial cameras.
     */
    public static final double DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO = 1.0;

    /**
     * Default horizontal principal point value to use for initial cameras estimation using
     * Dual Image of Absolute Conic (DIAC) or Dual Absolute Quadric (DAQ) methods.
     */
    public static final double DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_X = 0.0;

    /**
     * Default vertical principal point value to use for initial cameras estimation using
     * Dual Image of Absolute Conic (DIAC) or Dual Absolute Quadric (DAQ) methods.
     */
    public static final double DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_Y = 0.0;

    /**
     * Default corrector type to use for point triangulation when initial cameras are
     * being estimated using either Dual Image of Absolute Conic (DIAC), Dual Absolute Quadric
     * (DAQ) or essential matrix methods.
     */
    public static final CorrectorType DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE =
            CorrectorType.SAMPSON_CORRECTOR;

    /**
     * Default value indicating whether valid triangulated points are marked during initial
     * cameras estimation using either Dual Image of Absolute Conic (DIAC) or essential matrix
     * methods.
     */
    public static final boolean DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS = true;

    /**
     * Indicates whether a general (points are laying in a general 3D position) scene is
     * allowed.
     * When true, an initial geometry estimation is attempted for general points.
     */
    public static final boolean DEFAULT_ALLOW_GENERAL_SCENE = true;

    /**
     * Indicates whether a planar (points laying in a 3D plane) scene is allowed.
     * When true, an initial geometry estimation is attempted for planar points.
     */
    public static final boolean DEFAULT_ALLOW_PLANAR_SCENE = true;

    /**
     * Default robust planar homography estimator method.
     * This is only used when planar scenes are allowed.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD =
            RobustEstimatorMethod.PROMedS;

    /**
     * Indicates that planar homography is refined by default using all found inliers.
     * This is only used when planar scenes are allowed.
     */
    public static final boolean DEFAULT_REFINE_PLANAR_HOMOGRAPHY = true;

    /**
     * Indicates that planar homography covariance is kept by default after estimation.
     * This is only used when planar scenes are allowed.
     */
    public static final boolean DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE = false;

    /**
     * Default confidence of robustly estimated planar homography. By default this is 99%.
     * This is only used when planar scenes are allowed.
     */
    public static final double DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE =
            ProjectiveTransformation2DRobustEstimator.DEFAULT_CONFIDENCE;

    /**
     * Default maximum number of iterations to make while robustly estimating planar
     * homography. By default this is 5000 iterations.
     * This is only used when planar scenes are allowed.
     */
    public static final int DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS =
            ProjectiveTransformation2DRobustEstimator.DEFAULT_MAX_ITERATIONS;

    /**
     * Default threshold to determine whether samples for robust projective 2D transformation
     * estimation are inliers or not.
     * This is only used when planar scenes are allowed.
     */
    public static final double DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD = 1e-3;

    /**
     * Default value indicating that inlier data is kept after robust planar homography estimation.
     * This is only used when planar scenes are allowed.
     */
    public static final boolean DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS = true;

    /**
     * Default value indicating that residual data is kept after robust planar homography
     * estimation.
     * This is only used when planar scenes are allowed.
     */
    public static final boolean DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS = true;

    /**
     * Default value indicating that additional cameras intrinsics are estimated using the
     * Dual Absolute Quadric (DAQ).
     */
    public static final boolean DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS = true;

    /**
     * Default value indicating that additional cameras intrinsics are estimated using
     * the Dual Image of Absolute Conic (DIAC).
     */
    public static final boolean DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS = false;

    /**
     * Default skewness for additional cameras when UPnP (Uncalibrated Perspective-n-Point)
     * method is used for additional cameras estimation and neither Dual Image of Absolute
     * Conic (DIAC) or Dual Absolute Quadric (DAQ) are estimated to find intrinsic
     * parameters when adding new cameras.
     */
    public static final double DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS = 0.0;

    /**
     * Default horizontal coordinate of principal point for additional cameras when
     * UPnP (Uncalibrated Perspective-n-Point) method is used for additional cameras
     * estimation and neither Dual Image of Absolute Conic (DIAC) or Dual Absolute
     * Quadric (DAQ) are estimated to find intrinsic parameters when adding new
     * cameras.
     */
    public static final double DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT =
            0.0;

    /**
     * Default vertical coordinate of principal point for additional cameras when
     * UPnP (Uncalibrated Perspective-n-Point) method is used for additional cameras
     * estimation and neither Dual Image of Absolute Conic (DIAC) or Dual Absolute
     * Quadric (DAQ) are estimated to find intrinsic parameters when adding new
     * cameras.
     */
    public static final double DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT =
            0.0;

    /**
     * Default aspect ratio for additional cameras.
     */
    public static final double DEFAULT_ADDITIONAL_CAMERAS_ASPECT_RATIO = 1.0;


    /**
     * Indicates that by default EPnP (Efficient Perspective-n-Point) method is NOT
     * used for additional cameras estimation.
     */
    public static final boolean DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION =
            false;

    /**
     * Indicates that by default UPnP (Uncalibrated Perspective-n-Point) method is
     * used for additional cameras estimation.
     */
    public static final boolean DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION =
            true;

    /**
     * Default robust method to estimate additional cameras.
     */
    public static final RobustEstimatorMethod
            DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD =
            RobustEstimatorMethod.PROSAC;

    /**
     * Default value indicating that planar configuration is allowed for additional
     * cameras estimation using either EPnP or UPnP.
     */
    public static final boolean DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION =
            EPnPPointCorrespondencePinholeCameraEstimator.
                    DEFAULT_PLANAR_CONFIGURATION_ALLOWED;

    /**
     * Default value indicating that dimension 2 null-space is allowed while estimating
     * additional cameras using either EPnP or UPnP.
     */
    public static final boolean DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2 =
            EPnPPointCorrespondencePinholeCameraEstimator.
                    DEFAULT_NULLSPACE_DIMENSION2_ALLOWED;

    /**
     * Default value indicating that dimension 3 null-space is allowed while estimating
     * additional cameras using EPnP.
     */
    public static final boolean DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3 =
            EPnPPointCorrespondencePinholeCameraEstimator.
                    DEFAULT_NULLSPACE_DIMENSION3_ALLOWED;

    /**
     * Default threshold to determine whether 3D matched points to estimate additional
     * cameras are in a planar configuration.
     */
    public static final double DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD =
            EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD;

    /**
     * Default value indicating that additional cameras are refined to minimize overall
     * projection error among all found inliers.
     */
    public static final boolean DEFAULT_REFINE_ADDITIONAL_CAMERAS =
            PinholeCameraRobustEstimator.DEFAULT_REFINE_RESULT;

    /**
     * Default value indicating that covariance is kept after refining results of
     * additional cameras estimation.
     */
    public static final boolean DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS = true;

    /**
     * Default value indicating that fast refinement is used for additional cameras
     * estimation.
     */
    public static final boolean DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT = true;

    /**
     * Default confidence of estimated additional cameras, which is 99%. This means
     * that with a probability of 99% estimation will be accurate because chosen
     * sub-samples will be inliers.
     */
    public static final double DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE =
            PinholeCameraRobustEstimator.DEFAULT_CONFIDENCE;

    /**
     * Default maximum allowed number of iterations for additional cameras estimation.
     */
    public static final int DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS =
            PinholeCameraRobustEstimator.DEFAULT_MAX_ITERATIONS;

    /**
     * Default threshold to determine whether samples for robust pinhole camera estimation are
     * inliers or not.
     */
    public static final double DEFAULT_ADDITIONAL_CAMERAS_THRESHOLD =
            PROSACEPnPPointCorrespondencePinholeCameraRobustEstimator.DEFAULT_THRESHOLD;

    /**
     * Default value indicating that inlier data is kept after additional camera estimation.
     */
    public static final boolean DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS = true;

    /**
     * Default value indicating that residual data is kept after additional camera estimation.
     */
    public static final boolean DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS = true;

    /**
     * Default value indicating that skewness is not suggested during additional cameras
     * estimation.
     */
    public static final boolean DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED =
            PinholeCameraRobustEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED;

    /**
     * Default value of skewness to be suggested when suggestion is enabled during
     * additional cameras estimation.
     * By default suggested skewness is zero.
     */
    public static final double DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE =
            PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE;

    /**
     * Default value indicating whether horizontal focal length value is suggested or not
     * during additional cameras estimation. By default this is disabled.
     */
    public static final boolean DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED =
            PinholeCameraRobustEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED;

    /**
     * Default value indicating whether vertical focal length value is suggested or not
     * during additional cameras estimation. By default this is disabled.
     */
    public static final boolean DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED =
            PinholeCameraRobustEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED;

    /**
     * Default value indicating whether aspect ratio is suggested or not. By default this is
     * disabled.
     */
    public static final boolean DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED =
            PinholeCameraRobustEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED;

    /**
     * Default value of aspect ratio to be suggested when suggestion is enabled.
     * By default suggested aspect ratio is 1.0, although also -1.0 is a typical value
     * when vertical coordinates increase downwards.
     */
    public static final double DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE =
            PinholeCameraRobustEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE;

    /**
     * Default value indicating whether principal point is suggested or not. By default
     * this is disabled.
     */
    public static final boolean DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED =
            PinholeCameraRobustEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED;

    /**
     * Indicates that an homogeneous point triangulator will be used by default.
     */
    public static final boolean DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR =
            RobustSinglePoint3DTriangulator.DEFAULT_USE_HOMOGENEOUS_SOLUTION;

    /**
     * Default robust point triangulator method.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD =
            RobustSinglePoint3DTriangulator.DEFAULT_ROBUST_METHOD;

    /**
     * Default confidence of robustly triangulated points. By default this is 99%.
     */
    public static final double DEFAULT_POINT_TRIANGULATOR_CONFIDENCE =
            RobustSinglePoint3DTriangulator.DEFAULT_CONFIDENCE;

    /**
     * Default maximum number of iterations to make while robustly estimating
     * triangulated points. By default this is 5000 iterations.
     */
    public static final int DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS =
            RobustSinglePoint3DTriangulator.DEFAULT_MAX_ITERATIONS;

    /**
     * Default threshold to determine whether samples for robust point triangulator are
     * inliers or not.
     */
    public static final double DEFAULT_POINT_TRIANGULATOR_THRESHOLD =
            PROSACRobustSinglePoint3DTriangulator.DEFAULT_THRESHOLD;

    /**
     * Method to use for non robust fundamental matrix estimation.
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
     * Indicates whether estimated fundamental matrix is refined among all found inliers.
     * This is only used when general scenes are allowed.
     */
    private boolean mRefineFundamentalMatrix = DEFAULT_REFINE_FUNDAMENTAL_MATRIX;

    /**
     * Indicates whether covariance of estimated fundamental matrix is kept after the
     * estimation.
     * This is only used when general scenes are allowed.
     */
    private boolean mKeepFundamentalMatrixCovariance = DEFAULT_KEEP_FUNDAMENTAL_MATRIX_COVARIANCE;

    /**
     * Confidence of robustly estimated fundamental matrix.
     * This is only used when general scenes are allowed.
     */
    private double mFundamentalMatrixConfidence = DEFAULT_FUNDAMENTAL_MATRIX_CONFIDENCE;

    /**
     * Maximum number of iterations to robustly estimate fundamental matrix.
     * This is only used when general scenes are allowed.
     */
    private int mFundamentalMatrixMaxIterations = DEFAULT_FUNDAMENTAL_MATRIX_MAX_ITERATIONS;

    /**
     * Threshold to determine whether samples for robust fundamental matrix estimation are
     * inliers or not.
     * This is only used when general scenes are allowed.
     */
    private double mFundamentalMatrixThreshold = DEFAULT_FUNDAMENTAL_MATRIX_THRESHOLD;

    /**
     * Indicates whether inliers must be kept during robust fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     */
    private boolean mFundamentalMatrixComputeAndKeepInliers =
            DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_INLIERS;

    /**
     * Indicates whether residuals must be computed and kept during robust fundamental matrix
     * estimation.
     * This is only used when general scenes are allowed.
     */
    private boolean mFundamentalMatrixComputeAndKeepResiduals =
            DEFAULT_FUNDAMENTAL_MATRIX_COMPUTE_AND_KEEP_RESIDUALS;

    /**
     * Method to use for initial cameras estimation.
     */
    private InitialCamerasEstimatorMethod mInitialCamerasEstimatorMethod =
            DEFAULT_INITIAL_CAMERAS_ESTIMATOR_METHOD;

    /**
     * Indicates whether an homogeneous point triangulator is used for point triangulation
     * when Dual Absolute Quadric (DAQ) camera initialization is used.
     */
    private boolean mDaqUseHomogeneousPointTriangulator =
            DEFAULT_DAQ_USE_HOMOGENEOUS_POINT_TRIANGULATOR;

    /**
     * Aspect ratio for initial cameras.
     */
    private double mInitialCamerasAspectRatio = DEFAULT_INITIAL_CAMERAS_ASPECT_RATIO;

    /**
     * Horizontal principal point value to use for initial cameras estimation using
     * Dual Image of Absolute Conic (DIAC) or Dual Absolute Quadric (DAQ) methods.
     */
    private double mPrincipalPointX = DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_X;

    /**
     * Vertical principal point value to use for initial cameras estimation using
     * Dual Image of Absolute Conic (DIAC) or Dual Absolute Quadric (DAC) methods.
     */
    private double mPrincipalPointY = DEFAULT_INITIAL_CAMERAS_PRINCIPAL_POINT_Y;

    /**
     * Corrector type to use for point triangulation when initial cameras are being estimated
     * using either Dual Image of Absolute Conic (DIAC) or essential matrix methods or null
     * if no corrector is used.
     */
    private CorrectorType mInitialCamerasCorrectorType =
            DEFAULT_INITIAL_CAMERAS_CORRECTOR_TYPE;

    /**
     * Value indicating whether valid triangulated points are marked during initial
     * cameras estimation using either Dual Image of Absolute Conic (DIAC) or essential
     * matrix methods.
     */
    private boolean mInitialCamerasMarkValidTriangulatedPoints =
            DEFAULT_INITIAL_CAMERAS_MARK_VALID_TRIANGULATED_POINTS;

    /**
     * Intrinsic parameters of first camera estimated using the essential matrix
     * method.
     */
    private PinholeCameraIntrinsicParameters mInitialIntrinsic1;

    /**
     * Intrinsic parameters of second camera estimated using the essential matrix
     * method.
     */
    private PinholeCameraIntrinsicParameters mInitialIntrinsic2;

    /**
     * Indicates whether a general scene (points laying in a general 3D position) is
     * allowed.
     * When true, an initial geometry estimation is attempted for general points.
     */
    private boolean mAllowGeneralScene = DEFAULT_ALLOW_GENERAL_SCENE;

    /**
     * Indicates whether a planar scene (points laying in a 3D plane) is allowed.
     * When true, an initial geometry estimation is attempted for planar points.
     */
    private boolean mAllowPlanarScene = DEFAULT_ALLOW_PLANAR_SCENE;

    /**
     * Robust method to use for planar homography estimation.
     * This is only used when planar scenes are allowed.
     */
    private RobustEstimatorMethod mRobustPlanarHomographyEstimatorMethod =
            DEFAULT_ROBUST_PLANAR_HOMOGRAPHY_ESTIMATOR_METHOD;

    /**
     * Indicates whether planar homography is refined using all found inliers or not.
     * This is only used when planar scenes are allowed.
     */
    private boolean mRefinePlanarHomography = DEFAULT_REFINE_PLANAR_HOMOGRAPHY;

    /**
     * Indicates whether planar homography covariance is kept after estimation.
     * This is only used when planar scenes are allowed.
     */
    private boolean mKeepPlanarHomographyCovariance = DEFAULT_KEEP_PLANAR_HOMOGRAPHY_COVARIANCE;

    /**
     * Confidence of robustly estimated planar homography. By default this is 99%.
     * This is only used when planar scenes are allowed.
     */
    private double mPlanarHomographyConfidence = DEFAULT_PLANAR_HOMOGRAPHY_CONFIDENCE;

    /**
     * Maximum number of iterations to make while robustly estimating planar homography.
     * By default this is 5000.
     * This is only used when planar scenes are allowed.
     */
    private int mPlanarHomographyMaxIterations = DEFAULT_PLANAR_HOMOGRAPHY_MAX_ITERATIONS;

    /**
     * Threshold to determine whether samples for robust projective 2D transformation estimation
     * are inliers or not.
     */
    private double mPlanarHomographyThreshold = DEFAULT_PLANAR_HOMOGRAPHY_THRESHOLD;

    /**
     * Value indicating that inlier data is kept after robust planar homography estimation.
     * This is only used when planar scenes are allowed.
     */
    private boolean mPlanarHomographyComputeAndKeepInliers =
            DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_INLIERS;

    /**
     * Value indicating that residual data is kept after robust planar homography estimation.
     * This is only used when planar scenes are allowed.
     */
    private boolean mPlanarHomographyComputeAndKeepResiduals =
            DEFAULT_PLANAR_HOMOGRAPHY_COMPUTE_AND_KEEP_RESIDUALS;

    /**
     * Indicates that additional cameras intrinsics are estimated using the Dual Absolute
     * Quadric (DAQ).
     */
    private boolean mUseDAQForAdditionalCamerasIntrinsics =
            DEFAULT_USE_DAQ_FOR_ADDITIONAL_CAMERAS_INTRINSICS;

    /**
     * Indicates that additional cameras intrinsics are estimated using the Dual Image of Absolute
     * Conic (DIAC).
     */
    private boolean mUseDIACForAdditionalCamerasIntrinsics =
            DEFAULT_USE_DIAC_FOR_ADDITIONAL_CAMERAS_INTRINSICS;

    /**
     * Intrinsic parameters to use for additional cameras estimation when neither
     * Dual Image of Absolute Conic (DIAC) nor Dual Absolute Quadric (DAQ) are used
     * for intrinsic parameters estimation.
     */
    private PinholeCameraIntrinsicParameters mAdditionalCamerasIntrinsics;

    /**
     * Skewness for additional cameras when UPnP (Uncalibrated Perspective-n-Point) method
     * is used for additional cameras estimation.
     */
    private double mAdditionalCamerasSkewness = DEFAULT_ADDITIONAL_CAMERAS_SKEWNESS;

    /**
     * Horizontal coordinate of principal point for additional cameras when UPnP
     * (Uncalibrated Perspective-n-Point) method is used for additional cameras estimation
     * and neither Dual Image of Absolute Conic (DIAC) or Dual Absolute Quadric (DAQ) are
     * estimated to find intrinsic parameters when adding new cameras.
     */
    private double mAdditionalCamerasHorizontalPrincipalPoint =
            DEFAULT_ADDITIONAL_CAMERAS_HORIZONTAL_PRINCIPAL_POINT;

    /**
     * Vertical coordinate of principal point for additional cameras when UPnP (Uncalibrated
     * Perspective-n-Point) method is used for additional cameras estimation and neither Dual
     * Image of Absolute Conic (DIAC) nor Dual Absolute Quadric (DAQ) are estimated to find
     * intrinsic parameters when adding new cameras.
     */
    private double mAdditionalCamerasVerticalPrincipalPoint =
            DEFAULT_ADDITIONAL_CAMERAS_VERTICAL_PRINCIPAL_POINT;

    /**
     * Aspect ratio for additional cameras.
     */
    private double mAdditionalCamerasAspectRatio = DEFAULT_ADDITIONAL_CAMERAS_ASPECT_RATIO;

    /**
     * Indicates whether EPnP (Efficient Perspective-n-Point) method is used for additional
     * cameras estimation. Either EPnP or UPnP must be used for additional cameras
     * estimation.
     */
    private boolean mUseEPnPForAdditionalCamerasEstimation =
            DEFAULT_USE_EPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION;

    /**
     * Indicates whether UPnP (Uncalibrated Perspective-n-Point) method is used for
     * additional cameras estimation. Either EPnP or UPnP must be used for additional
     * cameras estimation.
     */
    private boolean mUseUPnPForAdditionalCamerasEstimation =
            DEFAULT_USE_UPNP_FOR_ADDITIONAL_CAMERAS_ESTIMATION;

    /**
     * Robust method to estimate additional cameras.
     */
    private RobustEstimatorMethod mAdditionalCamerasRobustEstimationMethod =
            DEFAULT_ADDITIONAL_CAMERAS_ROBUST_ESTIMATION_METHOD;

    /**
     * Indicates whether planar configuration is allowed for additional cameras
     * estimation using either EPnP or UPnP.
     */
    private boolean mAdditionalCamerasAllowPlanarConfiguration =
            DEFAULT_ADDITIONAL_CAMERAS_ALLOW_PLANAR_CONFIGURATION;

    /**
     * Indicates whether dimension 2 null-space is allowed while estimating additional
     * cameras using either EPnP or UPnP.
     */
    private boolean mAdditionalCamerasAllowNullspaceDimension2 =
            DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION2;

    /**
     * Indicates whether dimension 3 null-space is allowed while estimating additional
     * cameras using EPnP.
     */
    private boolean mAdditionalCamerasAllowNullspaceDimension3 =
            DEFAULT_ADDITIONAL_CAMERAS_ALLOW_NULLSPACE_DIMENSION3;

    /**
     * Threshold to determine whether 3D matched points to estimate additional cameras
     * are in a planar configuration.
     */
    private double mAdditionalCamerasPlanarThreshold =
            DEFAULT_ADDITIONAL_CAMERAS_PLANAR_THRESHOLD;

    /**
     * Indicates whether additional cameras are refined to minimize overall projection
     * error among all found inliers.
     */
    private boolean mRefineAdditionalCameras = DEFAULT_REFINE_ADDITIONAL_CAMERAS;

    /**
     * Indicates whether covariance is kept after refining result of additional
     * cameras estimation.
     */
    private boolean mKeepCovarianceAdditionalCameras =
            DEFAULT_KEEP_COVARIANCE_ADDITIONAL_CAMERAS;

    /**
     * Value indicating whether fast refinement is used for additional cameras estimation.
     */
    private boolean mAdditionalCamerasUseFastRefinement =
            DEFAULT_ADDITIONAL_CAMERAS_USE_FAST_REFINEMENT;

    /**
     * Confidence of estimated additional cameras.
     */
    private double mAdditionalCamerasConfidence = DEFAULT_ADDITIONAL_CAMERAS_CONFIDENCE;

    /**
     * Maximum allowed number of iterations for additional cameras estimation.
     */
    private int mAdditionalCamerasMaxIterations =
            DEFAULT_ADDITIONAL_CAMERAS_MAX_ITERATIONS;

    /**
     * Threshold to determine whether samples for robust pinhole camera estimation are inliers or not.
     */
    private double mAdditionalCamerasThreshold = DEFAULT_ADDITIONAL_CAMERAS_THRESHOLD;

    /**
     * Indicates whether inliers must be kept during additional camera estimation.
     */
    private boolean mAdditionalCamerasComputeAndKeepInliers = DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_INLIERS;

    /**
     * Indicates whether residuals must be computed and kept during additional camera estimation.
     */
    private boolean mAdditionalCamerasComputeAndKeepResiduals = DEFAULT_ADDITIONAL_CAMERAS_COMPUTE_AND_KEEP_RESIDUALS;

    /**
     * Value indicating whether skewness is not suggested during additional cameras
     * estimation.
     */
    private boolean mAdditionalCamerasSuggestSkewnessValueEnabled =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_SKEWNESS_VALUE_ENABLED;

    /**
     * Value of skewness to be suggested when suggestion is enabled during additional
     * cameras estimation.
     */
    private double mAdditionalCamerasSuggestedSkewnessValue =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_SKEWNESS_VALUE;

    /**
     * Value indicating whether horizontal focal length value is suggested or not
     * during additional cameras estimation.
     */
    private boolean mAdditionalCamerasSuggestHorizontalFocalLengthEnabled =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED;

    /**
     * Value of suggested horizontal focal length during additional cameras estimation.
     */
    private double mAdditionalCamerasSuggestedHorizontalFocalLengthValue;

    /**
     * Value indicating whether vertical focal length value is suggested or not
     * during additional cameras estimation.
     */
    private boolean mAdditionalCamerasSuggestVerticalFocalLengthEnabled =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED;

    /**
     * Value of suggested vertical focal length during additional cameras estimation.
     */
    private double mAdditionalCamerasSuggestedVerticalFocalLengthValue;

    /**
     * Value indicating whether aspect ratio is suggested or not during
     * additional cameras estimation.
     */
    private boolean mAdditionalCamerasSuggestAspectRatioEnabled =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_ASPECT_RATIO_ENABLED;

    /**
     * Value of aspect ratio to be suggested when suggestion is enabled during
     * additional cameras estimation.
     */
    private double mAdditionalCamerasSuggestedAspectRatioValue =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGESTED_ASPECT_RATIO_VALUE;

    /**
     * Value indicating whether principal point is suggested or not during
     * additional cameras estimation.
     */
    private boolean mAdditionalCamerasSuggestPrincipalPointEnabled =
            DEFAULT_ADDITIONAL_CAMERAS_SUGGEST_PRINCIPAL_POINT_ENABLED;

    /**
     * Value of principal point to be suggested when suggestion is enabled
     * during additional cameras estimation.
     */
    private InhomogeneousPoint2D mAdditionalCamerasSuggestedPrincipalPointValue;

    /**
     * Indicates whether homogeneous point triangulator must be used or not to
     * estimate 3D points when only two matches are available.
     */
    private boolean mUseHomogeneousPointTriangulator =
            DEFAULT_USE_HOMOGENEOUS_POINT_TRIANGULATOR;

    /**
     * Robust method for point triangulation when points are matched in more
     * than two views.
     */
    private RobustEstimatorMethod mRobustPointTriangulatorMethod =
            DEFAULT_ROBUST_POINT_TRIANGULATOR_METHOD;

    /**
     * Confidence of robustly triangulated points. By default this is 99%.
     */
    private double mPointTriangulatorConfidence =
            DEFAULT_POINT_TRIANGULATOR_CONFIDENCE;

    /**
     * Maximum number of iterations to make while robustly estimating
     * triangulated points. By default this is 5000 iterations.
     */
    private int mPointTriangulatorMaxIterations =
            DEFAULT_POINT_TRIANGULATOR_MAX_ITERATIONS;

    /**
     * Threshold to determine whether samples for robust point triangulator are
     * inliers or not.
     */
    private double mPointTriangulatorThreshold =
            DEFAULT_POINT_TRIANGULATOR_THRESHOLD;

    /**
     * Constructor.
     */
    protected BaseSparseReconstructorConfiguration() {
    }

    /**
     * Gets method to use for non robust fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     *
     * @return method to use for non robust fundamental matrix estimation.
     */
    public FundamentalMatrixEstimatorMethod getNonRobustFundamentalMatrixEstimatorMethod() {
        return mNonRobustFundamentalMatrixEstimatorMethod;
    }

    /**
     * Sets method to use for non robust fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     *
     * @param method method to use for non robust fundamental matrix estimation.
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
     * Indicates whether estimated fundamental matrix is refined among all found inliers.
     * This is only used when general scenes are allowed.
     *
     * @return true if fundamental matrix is refined, false otherwise.
     */
    public boolean isFundamentalMatrixRefined() {
        return mRefineFundamentalMatrix;
    }

    /**
     * Specifies whether estimated fundamental matrix is refined among all found inliers.
     * This is only used when general scenes are allowed.
     *
     * @param refineFundamentalMatrix true if fundamental matrix is refined, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixRefined(final boolean refineFundamentalMatrix) {
        mRefineFundamentalMatrix = refineFundamentalMatrix;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether covariance of estimated fundamental matrix is kept after the estimation.
     * This is only used when general scenes are allowed.
     *
     * @return true if covariance is kept, false otherwise.
     */
    public boolean isFundamentalMatrixCovarianceKept() {
        return mKeepFundamentalMatrixCovariance;
    }

    /**
     * Specifies whether covariance of estimated fundamental matrix is kept after the
     * estimation.
     * This is only used when general scenes are allowed.
     *
     * @param keepFundamentalMatrixCovariance true if covariance is kept, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixCovarianceKept(final boolean keepFundamentalMatrixCovariance) {
        mKeepFundamentalMatrixCovariance = keepFundamentalMatrixCovariance;
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
        return mFundamentalMatrixConfidence;
    }

    /**
     * Sets confidence of robustly estimated fundamental matrix.
     * This is only used when general scenes are allowed.
     *
     * @param fundamentalMatrixConfidence confidence of robustly estimated fundamental matrix.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixConfidence(final double fundamentalMatrixConfidence) {
        mFundamentalMatrixConfidence = fundamentalMatrixConfidence;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets maximum number of iterations to robustly estimate fundamental matrix.
     * This is only used when general scenes are allowed.
     *
     * @return maximum number of iterations to robustly estimate fundamental matrix.
     */
    public int getFundamentalMatrixMaxIterations() {
        return mFundamentalMatrixMaxIterations;
    }

    /**
     * Sets maximum number of iterations to robustly estimate fundamental matrix.
     * This is only used when general scenes are allowed.
     *
     * @param fundamentalMatrixMaxIterations maximum number of iterations to robustly estimate
     *                                       fundamental matrix.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixMaxIterations(final int fundamentalMatrixMaxIterations) {
        mFundamentalMatrixMaxIterations = fundamentalMatrixMaxIterations;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets threshold to determine whether samples for robust fundamental matrix estimation
     * are inliers or not.
     * This is only used when general scenes are allowed.
     *
     * @return threshold to determine whether samples for robust fundamental matrix
     * estimation are inliers or not.
     */
    public double getFundamentalMatrixThreshold() {
        return mFundamentalMatrixThreshold;
    }

    /**
     * Sets threshold to determine whether samples for robust fundamental matrix
     * estimation are inliers or not.
     * This is only used when general scenes are allowed.
     *
     * @param fundamentalMatrixThreshold threshold to determine whether samples for
     *                                   robust fundamental matrix estimation are inliers
     *                                   or not.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixThreshold(final double fundamentalMatrixThreshold) {
        mFundamentalMatrixThreshold = fundamentalMatrixThreshold;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether inliers must be kept during robust fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     *
     * @return true if inliers must be kept during robust fundamental matrix estimation,
     * false otherwise.
     */
    public boolean getFundamentalMatrixComputeAndKeepInliers() {
        return mFundamentalMatrixComputeAndKeepInliers;
    }

    /**
     * Specifies whether inliers must be kept during robust fundamental matrix estimation.
     * This is only used when general scenes are allowed.
     *
     * @param fundamentalMatrixComputeAndKeepInliers true if inliers must be kept during
     *                                               robust fundamental matrix estimation, false
     *                                               otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixComputeAndKeepInliers(
            final boolean fundamentalMatrixComputeAndKeepInliers) {
        mFundamentalMatrixComputeAndKeepInliers = fundamentalMatrixComputeAndKeepInliers;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether residuals must be computed and kept during robust fundamental
     * matrix estimation.
     * This is only used when general scenes are allowed.
     *
     * @return true if residuals must be computed and kept, false otherwise.
     */
    public boolean getFundamentalMatrixComputeAndKeepResiduals() {
        return mFundamentalMatrixComputeAndKeepResiduals;
    }

    /**
     * Specifies whether residuals must be computed and kept during robust fundamental
     * matrix estimation.
     * This is only used when general scenes are allowed.
     *
     * @param fundamentalMatrixComputeAndKeepResiduals true if residuals must be
     *                                                 computed and kept, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setFundamentalMatrixComputeAndKeepResiduals(
            final boolean fundamentalMatrixComputeAndKeepResiduals) {
        mFundamentalMatrixComputeAndKeepResiduals =
                fundamentalMatrixComputeAndKeepResiduals;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets method to use for initial cameras estimation.
     *
     * @return method to use for initial cameras estimation.
     */
    public InitialCamerasEstimatorMethod getInitialCamerasEstimatorMethod() {
        return mInitialCamerasEstimatorMethod;
    }

    /**
     * Sets method to use for initial cameras estimation.
     *
     * @param method method to use for initial cameras estimation.
     * @return this instance so that method can be easily chained.
     */
    public T setInitialCamerasEstimatorMethod(final InitialCamerasEstimatorMethod method) {
        mInitialCamerasEstimatorMethod = method;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether an homogeneous point triangulator is used for point triangulation
     * when Dual Absolute Quadric (DAQ) camera initialization is used.
     *
     * @return true if homogeneous point triangulator is used, false if an inhomogeneous
     * point triangulator is used instead.
     */
    public boolean getDaqUseHomogeneousPointTriangulator() {
        return mDaqUseHomogeneousPointTriangulator;
    }

    /**
     * Specifies whether an homogeneous point triangulator is used for point
     * triangulation when Dual Absolute Quadric (DAQ) camera initialization is used.
     *
     * @param daqUseHomogeneousPointTriangulator true if homogeneous point triangulator
     *                                           is used, false if inhomogeneous point
     *                                           triangulator is used instead.
     * @return this instance so that method can be easily chained.
     */
    public T setDaqUseHomogeneousPointTriangulator(
            final boolean daqUseHomogeneousPointTriangulator) {
        mDaqUseHomogeneousPointTriangulator = daqUseHomogeneousPointTriangulator;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets aspect ratio for initial cameras estimation using DAQ or DIAC methods.
     *
     * @return aspect ratio for initial cameras using DAQ or DIAC methods.
     */
    public double getInitialCamerasAspectRatio() {
        return mInitialCamerasAspectRatio;
    }

    /**
     * Sets aspect ratio for initial cameras using DAQ or DIAC methods.
     *
     * @param initialCamerasAspectRatio aspect ratio for initial cameras using DAQ or DIAC
     *                                  methods.
     * @return this instance so that method can be easily chained.
     */
    public T setInitialCamerasAspectRatio(final double initialCamerasAspectRatio) {
        mInitialCamerasAspectRatio = initialCamerasAspectRatio;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets horizontal principal point value to use for initial cameras estimation
     * using DIAC or DAQ methods.
     *
     * @return horizontal principal point value to use for initial cameras estimation
     * using DIAC or DAQ methods.
     */
    public double getPrincipalPointX() {
        return mPrincipalPointX;
    }

    /**
     * Sets horizontal principal point value to use for initial cameras estimation
     * using DIAC or DAQ methods.
     *
     * @param principalPointX horizontal principal point value to use for initial
     *                        cameras estimation using DIAC or DAQ methods.
     * @return this instance so that method can be easily chained.
     */
    public T setPrincipalPointX(final double principalPointX) {
        mPrincipalPointX = principalPointX;
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
        return mPrincipalPointY;
    }

    /**
     * Sets vertical principal point value to use for initial cameras estimation using
     * DIAC or DAQ methods.
     *
     * @param principalPointY vertical principal point value to use for initial cameras
     *                        estimation using DIAC or DAQ methods.
     * @return this instance so that method can be easily chained.
     */
    public T setPrincipalPointY(final double principalPointY) {
        mPrincipalPointY = principalPointY;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets corrector type to use for point triangulation when initial cameras are being
     * estimated using either DIAC or essential matrix methods or null if no corrector is
     * used.
     *
     * @return corrector type to use for point triangulation when initial cameras are
     * being estimated using either DIAC or essential matrix methods or null if no
     * corrector is used.
     */
    public CorrectorType getInitialCamerasCorrectorType() {
        return mInitialCamerasCorrectorType;
    }

    /**
     * Sets corrector type to use for point triangulation when initial cameras are being
     * estimated using either DIAC or essential matrix methods or null if no corrector
     * is used.
     *
     * @param type corrector type to use for point triangulation when initial cameras
     *             are being estimated using either DIAC or essential matrix methods
     *             or null if no corrector is used.
     * @return this instance so that method can be easily chained.
     */
    public T setInitialCamerasCorrectorType(final CorrectorType type) {
        mInitialCamerasCorrectorType = type;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value indicating whether valid triangulated points are marked during initial
     * cameras estimation using either DIAC or essential matrix methods.
     *
     * @return value indicating whether valid triangulated points are marked during
     * initial cameras estimation using either DIAC or essential matrix methods.
     */
    public boolean getInitialCamerasMarkValidTriangulatedPoints() {
        return mInitialCamerasMarkValidTriangulatedPoints;
    }

    /**
     * Sets value indicating whether valid triangulated points are marked during initial
     * cameras estimation using either DIAC or essential matrix methods.
     *
     * @param initialCamerasMarkValidTriangulatedPoints value indicating whether valid
     *                                                  triangulated points are marked during
     *                                                  initial cameras estimation using
     *                                                  either DIAC or essential matrix
     *                                                  methods.
     * @return this instance so that method can be easily chained.
     */
    public T setInitialCamerasMarkValidTriangulatedPoints(
            final boolean initialCamerasMarkValidTriangulatedPoints) {
        mInitialCamerasMarkValidTriangulatedPoints =
                initialCamerasMarkValidTriangulatedPoints;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Intrinsic parameters of first camera estimated using the essential matrix method.
     *
     * @return parameters of first camera estimated using the essential matrix method.
     */
    public PinholeCameraIntrinsicParameters getInitialIntrinsic1() {
        return mInitialIntrinsic1;
    }

    /**
     * Sets intrinsic parameters of first camera estimated using the essential matrix
     * method.
     *
     * @param initialIntrinsic1 parameters of first camera estimated using the essential
     *                          matrix method.
     * @return this instance so that method can be easily chained.
     */
    public T setInitialIntrinsic1(final PinholeCameraIntrinsicParameters initialIntrinsic1) {
        mInitialIntrinsic1 = initialIntrinsic1;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Intrinsic parameters of second camera estimated using the essential matrix method.
     *
     * @return parameters of second camera estimated using the essential matrix method.
     */
    public PinholeCameraIntrinsicParameters getInitialIntrinsic2() {
        return mInitialIntrinsic2;
    }

    /**
     * Sets intrinsic parameters of second camera estimated using the essential matrix
     * method.
     *
     * @param initialIntrinsic2 parameters of second camera estimated using the essential
     *                          matrix method.
     * @return this instance so that method can be easily chained.
     */
    public T setInitialIntrinsic2(final PinholeCameraIntrinsicParameters initialIntrinsic2) {
        mInitialIntrinsic2 = initialIntrinsic2;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether a general scene (points laying in a general 3D position) is
     * allowed.
     * When true, an initial geometry estimation is attempted for general points.
     *
     * @return true if general scene is allowed, false otherwise.
     */
    public boolean isGeneralSceneAllowed() {
        return mAllowGeneralScene;
    }

    /**
     * Specifies whether a general scene (points laying in a general 3D position) is
     * allowed.
     * When true, an initial geometry estimation is attempted for general points.
     *
     * @param allowGeneralScene true if general scene is allowed, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setGeneralSceneAllowed(final boolean allowGeneralScene) {
        mAllowGeneralScene = allowGeneralScene;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether a planar scene (points laying in a 3D plane) is allowed or not.
     * When true, an initial geometry estimation is attempted for planar points.
     *
     * @return true if planar scene is allowed, false otherwise.
     */
    public boolean isPlanarSceneAllowed() {
        return mAllowPlanarScene;
    }

    /**
     * Specifies whether a planar scene (points laying in a 3D plane) is allowed or not.
     * When true, an initial geometry estimation is attempted for planar points.
     *
     * @param allowPlanarScene true if planar scene is allowed, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarSceneAllowed(final boolean allowPlanarScene) {
        mAllowPlanarScene = allowPlanarScene;
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
        return mRobustPlanarHomographyEstimatorMethod;
    }

    /**
     * Sets robust method to use for planar homography estimation.
     * This is only used when planar scenes are allowed.
     *
     * @param robustPlanarHomographyEstimatorMethod robust method to use for planar
     *                                              homography estimation.
     * @return this instance so that method can be easily chained.
     */
    public T setRobustPlanarHomographyEstimatorMethod(
            final RobustEstimatorMethod robustPlanarHomographyEstimatorMethod) {
        mRobustPlanarHomographyEstimatorMethod = robustPlanarHomographyEstimatorMethod;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether planar homography is refined using all found inliers or not.
     * This is only used when planar scenes are allowed.
     *
     * @return true if planar homography is refined, false otherwise.
     */
    public boolean isPlanarHomographyRefined() {
        return mRefinePlanarHomography;
    }

    /**
     * Specifies whether planar homography is refined using all found inliers or not.
     * This is only used when planar scenes are allowed.
     *
     * @param refinePlanarHomography true if planar homography must be refined, false
     *                               otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyRefined(final boolean refinePlanarHomography) {
        mRefinePlanarHomography = refinePlanarHomography;
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
        return mKeepPlanarHomographyCovariance;
    }

    /**
     * Specifies whether planar homography covariance is kept after estimation.
     * This is only used when planar scenes are allowed.
     *
     * @param keepPlanarHomographyCovariance true if planar homography covariance is
     *                                       kept, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyCovarianceKept(final boolean keepPlanarHomographyCovariance) {
        mKeepPlanarHomographyCovariance = keepPlanarHomographyCovariance;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets confidence of robustly estimated planar homography. By default this is 99%.
     * This is only used when planar scenes are allowed.
     *
     * @return confidence of robustly estimated planar homography.
     */
    public double getPlanarHomographyConfidence() {
        return mPlanarHomographyConfidence;
    }

    /**
     * Sets confidence of robustly estimated planar homography. By default this is 99%.
     * This is only used when planar scenes are allowed.
     *
     * @param planarHomographyConfidence confidence of robustly estimated planar
     *                                   homography.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyConfidence(final double planarHomographyConfidence) {
        mPlanarHomographyConfidence = planarHomographyConfidence;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets maximum number of iterations to make while robustly estimating planar
     * homography. By default this is 5000.
     * This is only used when planar scenes are allowed.
     *
     * @return maximum number of iterations to make while robustly estimating planar
     * homography.
     */
    public int getPlanarHomographyMaxIterations() {
        return mPlanarHomographyMaxIterations;
    }

    /**
     * Sets maximum number of iterations to make while robustly estimating planar
     * homography. By default this is 5000.
     * This is only used when planar scenes are allowed.
     *
     * @param planarHomographyMaxIterations maximum number of iterations to make while
     *                                      robustly estimating planar homography.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyMaxIterations(final int planarHomographyMaxIterations) {
        mPlanarHomographyMaxIterations = planarHomographyMaxIterations;
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
        return mPlanarHomographyThreshold;
    }

    /**
     * Sets threshold to determine whether samples for robust projective 2D
     * transformation estimation are inliers or not.
     * This is only used when planar scenes are allowed.
     *
     * @param planarHomographyThreshold threshold to robustly estimate projective 2D
     *                                  transformation.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyThreshold(final double planarHomographyThreshold) {
        mPlanarHomographyThreshold = planarHomographyThreshold;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value indicating that inlier data is kept after robust planar homography
     * estimation.
     * This is only used when planar scenes are allowed.
     *
     * @return true if inlier data is kept, false otherwise.
     */
    public boolean getPlanarHomographyComputeAndKeepInliers() {
        return mPlanarHomographyComputeAndKeepInliers;
    }

    /**
     * Specifies whether inlier data is kept after robust planar homography estimation.
     * This is only used when planar scenes are allowed.
     *
     * @param planarHomographyComputeAndKeepInliers true if inlier data is kept, false
     *                                              otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyComputeAndKeepInliers(
            final boolean planarHomographyComputeAndKeepInliers) {
        mPlanarHomographyComputeAndKeepInliers = planarHomographyComputeAndKeepInliers;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value indicating that residual data is kept after robust planar homography
     * estimation.
     * This is only used when planar scenes are allowed.
     *
     * @return true if residual data is kept, false otherwise.
     */
    public boolean getPlanarHomographyComputeAndKeepResiduals() {
        return mPlanarHomographyComputeAndKeepResiduals;
    }

    /**
     * Sets value indicating that residual data is kept after robust planar homography
     * estimation.
     * This is only used when planar scenes are allowed.
     *
     * @param planarHomographyComputeAndKeepResiduals true if residual data is kept,
     *                                                false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setPlanarHomographyComputeAndKeepResiduals(
            final boolean planarHomographyComputeAndKeepResiduals) {
        mPlanarHomographyComputeAndKeepResiduals =
                planarHomographyComputeAndKeepResiduals;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates that additional cameras intrinsics are estimated using the Dual Absolute
     * Quadric (DAQ).
     *
     * @return true if additional cameras intrinsics are estimated using the Dual Absolute
     * Quadric (DAQ), false otherwise.
     */
    public boolean getUseDAQForAdditionalCamerasIntrinsics() {
        return mUseDAQForAdditionalCamerasIntrinsics;
    }

    /**
     * Specifies whether additional cameras intrinsics are estimated using the Dual Absolute
     * Quadric (DAQ).
     *
     * @param useDAQForAdditionalCamerasIntrinics true if additional cameras intrinsics
     *                                            are estimated using the Dual Absolute
     *                                            Quadric (DAQ), false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setUseDAQForAdditionalCamerasIntrinics(
            final boolean useDAQForAdditionalCamerasIntrinics) {
        mUseDAQForAdditionalCamerasIntrinsics = useDAQForAdditionalCamerasIntrinics;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates that additional cameras intrinsics are estimated using the Dual Image of Absolute
     * Conic (DIAC).
     *
     * @return true if additional cameras intrinsics are estimated using the Dual Image of Absolute Conic
     * (DIAC), false otherwise.
     */
    public boolean getUseDIACForAdditionalCamerasIntrinsics() {
        return mUseDIACForAdditionalCamerasIntrinsics;
    }

    /**
     * Specifies whether additional cameras intrinsics are estimated using the Dual Image of
     * Absolute Conic (DIAC).
     * It is not recommended to enable this setting as it has low accuracy.
     *
     * @param useDIACForAdditionalCamerasIntrinsics true if additional cameras intrinsics are
     *                                              estimated using the Dual Image of Absolute
     *                                              Conic (DIAC), false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setUseDIACForAdditionalCamerasIntrinsics(
            final boolean useDIACForAdditionalCamerasIntrinsics) {
        mUseDIACForAdditionalCamerasIntrinsics = useDIACForAdditionalCamerasIntrinsics;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets intrinsic parameters to use for additional cameras estimation when neither
     * Dual Image of Absolute Conic (DIAC) nor Dual Absolute Quadric (DAQ) are used for
     * intrinsic parameters estimation.
     *
     * @return intrinsic parameters to use for additional cameras estimation.
     */
    public PinholeCameraIntrinsicParameters getAdditionalCamerasIntrinsics() {
        return mAdditionalCamerasIntrinsics;
    }

    /**
     * Sets intrinsic parameters to use for additional cameras estimation when neither
     * Dual Image of Absolute Conic (DIAC) nor Dual Absolute Quadric (DAQ) are used for
     * intrinsic parameters estimation.
     *
     * @param additionalCamerasIntrinsics intrinsic parameters to use for additional
     *                                    cameras estimation.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasIntrinsics(
            final PinholeCameraIntrinsicParameters additionalCamerasIntrinsics) {
        mAdditionalCamerasIntrinsics = additionalCamerasIntrinsics;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets skewness for additional cameras when UPnP (Uncalibrated Perspective-n-Point)
     * method is used for additional cameras estimation.
     *
     * @return skewness for additional cameras when UPnP method is used for additional
     * cameras estimation.
     */
    public double getAdditionalCamerasSkewness() {
        return mAdditionalCamerasSkewness;
    }

    /**
     * Sets skewness for additional cameras when UPnP (Uncalibrated Perspective-n-Point)
     * method is used for additional cameras estimation.
     *
     * @param additionalCamerasSkewness skewness for additional cameras when UPnP method is
     *                                  used for additional cameras estimation.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasSkewness(final double additionalCamerasSkewness) {
        mAdditionalCamerasSkewness = additionalCamerasSkewness;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets horizontal coordinate of principal point for additional cameras when UPnP
     * (Uncalibrated Perspective-n-Point) method is used for additional cameras
     * estimation and neither Dual Image of Absolute Conic (DIAC) or Dual Absolute
     * Quadric (DAQ) are estimated to find intrinsic parameters when adding new cameras.
     *
     * @return horizontal coordinate of principal point for additional cameras when
     * UPnP method is used for additional cameras estimation and neither DIAC or DAQ
     * are estimated to find intrinsic parameters when adding new cameras.
     */
    public double getAdditionalCamerasHorizontalPrincipalPoint() {
        return mAdditionalCamerasHorizontalPrincipalPoint;
    }

    /**
     * Sets horizontal coordinate of principal point for additional cameras when UPnP
     * (Uncalibrated Perspective-n-Point) method is used for additional cameras estimation
     * and neither Dual Image of Absolute Conic (DIAC) or Dual Absolute Quadric (DAQ) are
     * estimated to find intrinsic parameters when adding new cameras.
     *
     * @param additionalCamerasHorizontalPrincipalPoint horizontal coordinate of principal point
     *                                                  for additional cameras when UPnP method is
     *                                                  used for additional cameras estimation and
     *                                                  neither DIAC or DAQ are estimated to find
     *                                                  intrinsic parameters when adding new cameras.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasHorizontalPrincipalPoint(
            final double additionalCamerasHorizontalPrincipalPoint) {
        mAdditionalCamerasHorizontalPrincipalPoint = additionalCamerasHorizontalPrincipalPoint;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets vertical coordinate of principal point for additional cameras when UPnP
     * (Uncalibrated Perspective-n-Point) method is used for additional cameras estimation
     * and neither Dual Image of Absolute Conic (DIAC) or Dual Absolute Quadric (DAQ) are
     * estimated to find intrinsic parameters when adding new cameras.
     *
     * @return vertical coordinate of principal point for additional cameras when UPnP
     * method is used for additional cameras estimation and neither DIAC or DAQ are
     * estimated to find intrinsic parameters when adding new cameras.
     */
    public double getAdditionalCamerasVerticalPrincipalPoint() {
        return mAdditionalCamerasVerticalPrincipalPoint;
    }

    /**
     * Sets vertical coordinate of principal point for additional cameras when UPnP
     * (Uncalibrated Perspective-n-Point) method is used for additional cameras
     * estimation and neither Dual Image of Absolute Conic (DIAC) or Dual Absolute Quadric
     * (DAQ) are estimated to find intrinsic parameters when adding new cameras.
     *
     * @param additionalCamerasVerticalPrincipalPoint vertical coordinate of principal
     *                                                point for additional cameras when UPnP
     *                                                method is used for additional cameras
     *                                                estimation and neither DIAC or DAQ
     *                                                are estimated to find intrinsic
     *                                                parameters when adding new cameras.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasVerticalPrincipalPoint(
            final double additionalCamerasVerticalPrincipalPoint) {
        mAdditionalCamerasVerticalPrincipalPoint = additionalCamerasVerticalPrincipalPoint;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets aspect ratio for additional cameras estimation using DAQ or DIAC methods.
     *
     * @return aspect ratio for additional cameras using DAQ or DIAC methods.
     */
    public double getAdditionalCamerasAspectRatio() {
        return mAdditionalCamerasAspectRatio;
    }

    /**
     * Sets aspect ratio for additional cameras using DAQ or DIAC methods.
     *
     * @param additionalCamerasAspectRatio aspect ratio for additional cameras using DAQ or DIAC
     *                                     methods.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasAspectRatio(final double additionalCamerasAspectRatio) {
        mAdditionalCamerasAspectRatio = additionalCamerasAspectRatio;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether EPnP (Efficient Perspective-n-Point) method is used for additional
     * cameras estimation. Either EPnP or UPnP must be used for additional cameras
     * estimation.
     *
     * @return true if EPnP method is used for additional cameras estimation, false
     * otherwise.
     */
    public boolean getUseEPnPForAdditionalCamerasEstimation() {
        return mUseEPnPForAdditionalCamerasEstimation;
    }

    /**
     * Specifies whether EPnP (Efficient Perspective-n-Point) method is used for additional
     * cameras estimation. Either EPnP or UPnP must be used for additional cameras estimation.
     *
     * @param useEPnPForAdditionalCamerasEstimation true if EPnP method is used for additional
     *                                              cameras estimation, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setUseEPnPForAdditionalCamerasEstimation(
            final boolean useEPnPForAdditionalCamerasEstimation) {
        mUseEPnPForAdditionalCamerasEstimation = useEPnPForAdditionalCamerasEstimation;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether UPnP (Uncalibrated Perspective-n-Point) method is used for
     * additional cameras estimation. Either EPnP or UPnP must be used for additional
     * cameras estimation.
     *
     * @return true if UPnP method is used for additional cameras estimation, false
     * otherwise.
     */
    public boolean getUseUPnPForAdditionalCamerasEstimation() {
        return mUseUPnPForAdditionalCamerasEstimation;
    }

    /**
     * Specifies whether UPnP (Uncalibrated Perspective-n-Point) method is used for
     * additional cameras estimation. Either EPnP or UPnP must be used for additional
     * cameras estimation.
     *
     * @param useUPnPForAdditionalCamerasEstimation true if UPnP method is used for
     *                                              additional cameras estimation, false
     *                                              otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setUseUPnPForAdditionalCamerasEstimation(
            final boolean useUPnPForAdditionalCamerasEstimation) {
        mUseUPnPForAdditionalCamerasEstimation = useUPnPForAdditionalCamerasEstimation;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets robust method to estimate additional cameras.
     *
     * @return robust method to estimate additional cameras.
     */
    public RobustEstimatorMethod getAdditionalCamerasRobustEstimationMethod() {
        return mAdditionalCamerasRobustEstimationMethod;
    }

    /**
     * Sets robust method to estimate additional cameras.
     *
     * @param method robust method to estimate additional cameras.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasRobustEstimationMethod(final RobustEstimatorMethod method) {
        mAdditionalCamerasRobustEstimationMethod = method;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether planar configuration is allowed for additional cameras
     * estimation using either EPnP or UPnP.
     *
     * @return true if planar configuration is allowed, false otherwise.
     */
    public boolean getAdditionalCamerasAllowPlanarConfiguration() {
        return mAdditionalCamerasAllowPlanarConfiguration;
    }

    /**
     * Specifies whether planar configuration is allowed for additional cameras
     * estimation using either EPnP or UPnP.
     *
     * @param allowPlanarConfiguration true if planar configuration is allowed, false
     *                                 otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasAllowPlanarConfiguration(final boolean allowPlanarConfiguration) {
        mAdditionalCamerasAllowPlanarConfiguration = allowPlanarConfiguration;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether dimension 2 null-space is allowed while estimating additional
     * cameras using either EPnP or UPnP.
     *
     * @return true if dimension 2 null-space is allowed while estimating additional
     * cameras, false otherwise.
     */
    public boolean getAdditionalCamerasAllowNullspaceDimension2() {
        return mAdditionalCamerasAllowNullspaceDimension2;
    }

    /**
     * Specifies whether dimension 2 null-space is allowed while estimating additional
     * cameras using either EPnP or UPnP.
     *
     * @param allowNullspaceDimension2 true if dimension 2 null-space is allowed while
     *                                 estimating additional cameras, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasAllowNullspaceDimension2(
            final boolean allowNullspaceDimension2) {
        mAdditionalCamerasAllowNullspaceDimension2 = allowNullspaceDimension2;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether dimension 3 null-space is allowed while estimating additional
     * cameras using EPnP.
     *
     * @return true if dimension 3 null-space is allowed while estimating additional
     * cameras, false otherwise.
     */
    public boolean getAdditionalCamerasAllowNullspaceDimension3() {
        return mAdditionalCamerasAllowNullspaceDimension3;
    }

    /**
     * Specifies whether dimension 3 null-space is allowed while estimating additional
     * cameras using either EPnP or UPnP.
     *
     * @param allowNullspaceDimension3 true if dimension 3 null-space is allowed while
     *                                 estimating additional cameras, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasAllowNullspaceDimension3(
            final boolean allowNullspaceDimension3) {
        mAdditionalCamerasAllowNullspaceDimension3 = allowNullspaceDimension3;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets threshold to determine whether 3D matched points to estimate additional
     * cameras are in a planar configuration.
     *
     * @return threshold to determine whether 3D matched points to estimate additional
     * cameras are in a planar configuration.
     */
    public double getAdditionalCamerasPlanarThreshold() {
        return mAdditionalCamerasPlanarThreshold;
    }

    /**
     * Specifies threshold to determine whether 3D matched points to estimate additional
     * cameras are in a planar configuration.
     *
     * @param additionalCamerasPlanarThreshold threshold to determine whether 3D matched
     *                                         points to estimate additional cameras are
     *                                         in a planar configuration.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasPlanarThreshold(
            final double additionalCamerasPlanarThreshold) {
        mAdditionalCamerasPlanarThreshold = additionalCamerasPlanarThreshold;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether additional cameras are refined to minimize overall projection
     * error among all found inliers.
     *
     * @return true if additional cameras are refined, false otherwise.
     */
    public boolean areAdditionalCamerasRefined() {
        return mRefineAdditionalCameras;
    }

    /**
     * Specifies whether additional cameras are refined to minimize overall projection
     * error among all found inliers.
     *
     * @param refineAdditionalCameras true if additional cameras are refined, false
     *                                otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasRefined(final boolean refineAdditionalCameras) {
        mRefineAdditionalCameras = refineAdditionalCameras;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether covariance is kept after refining result of additional
     * cameras estimation.
     *
     * @return true if covariance is kept, false otherwise.
     */
    public boolean isAdditionalCamerasCovarianceKept() {
        return mKeepCovarianceAdditionalCameras;
    }

    /**
     * Specifies whether covariance is kept after refining result of additional cameras
     * estimation.
     *
     * @param keepCovarianceAdditionalCameras true if covariance is kept, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasCovarianceKept(final boolean keepCovarianceAdditionalCameras) {
        mKeepCovarianceAdditionalCameras = keepCovarianceAdditionalCameras;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value indicating whether fast refinement is used for additional cameras
     * estimation.
     *
     * @return true if fast refinement is used for additional cameras estimation,
     * false otherwise.
     */
    public boolean getAdditionalCamerasUseFastRefinement() {
        return mAdditionalCamerasUseFastRefinement;
    }

    /**
     * Sets value indicating whether fast refinement is used for additional cameras
     * estimation.
     *
     * @param additionalCamerasUseFastRefinement true if fast refinement is used for
     *                                           additional cameras estimation, false
     *                                           otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasUseFastRefinement(final boolean additionalCamerasUseFastRefinement) {
        mAdditionalCamerasUseFastRefinement = additionalCamerasUseFastRefinement;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets confidence of estimated additional cameras.
     *
     * @return confidence of estimated additional cameras.
     */
    public double getAdditionalCamerasConfidence() {
        return mAdditionalCamerasConfidence;
    }

    /**
     * Sets confidence of estimated additional cameras.
     *
     * @param additionalCamerasConfidence confidence of estimated additional cameras.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasConfidence(final double additionalCamerasConfidence) {
        mAdditionalCamerasConfidence = additionalCamerasConfidence;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets maximum allowed number of iterations for additional cameras estimation.
     *
     * @return maximum allowed number of iterations for additional cameras estimation.
     */
    public int getAdditionalCamerasMaxIterations() {
        return mAdditionalCamerasMaxIterations;
    }

    /**
     * Sets maximum allowed number of iterations for additional cameras estimation.
     *
     * @param additionalCamerasMaxIterations maximum allowed number of iterations for
     *                                       additional cameras estimation.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasMaxIterations(final int additionalCamerasMaxIterations) {
        mAdditionalCamerasMaxIterations = additionalCamerasMaxIterations;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets threshold to determine whether samples for robust pinhole camera estimation are inliers or not.
     *
     * @return threshold to determine whether samples for robust pinhole camera estimation are inliers or not.
     */
    public double getAdditionalCamerasThreshold() {
        return mAdditionalCamerasThreshold;
    }

    /**
     * Sets threshold to determine whether samples for robust pinhole camera estimation are inliers or not.
     *
     * @param additionalCamerasThreshold threshold to determine whether samples for robust pinhole camera estimation
     *                                   are inliers or not.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasThreshold(final double additionalCamerasThreshold) {
        mAdditionalCamerasThreshold = additionalCamerasThreshold;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether inliers must be kept during additional camera estimation.
     *
     * @return true if inliers must be kept during additional camera estimation, false
     * otherwise.
     */
    public boolean getAdditionalCamerasComputeAndKeepInliers() {
        return mAdditionalCamerasComputeAndKeepInliers;
    }

    /**
     * Specifies whether inliers must be kept during additional camera estimation.
     *
     * @param additionalCamerasComputeAndKeepInliers true if inliers must be kept during additional camera estimation,
     *                                               false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasComputeAndKeepInliers(final boolean additionalCamerasComputeAndKeepInliers) {
        mAdditionalCamerasComputeAndKeepInliers = additionalCamerasComputeAndKeepInliers;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether residuals must be computed and kept during additional camera estimation.
     *
     * @return true if residuals must be computed and kept, false otherwise.
     */
    public boolean getAdditionalCamerasComputeAndKeepResiduals() {
        return mAdditionalCamerasComputeAndKeepResiduals;
    }

    /**
     * Specifies whether residuals must be computed and kept during additional camera estimation.
     *
     * @param additionalCamerasComputeAndKeepResiduals true if residuals must be computed and kept, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasComputeAndKeepResiduals(final boolean additionalCamerasComputeAndKeepResiduals) {
        mAdditionalCamerasComputeAndKeepResiduals = additionalCamerasComputeAndKeepResiduals;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value indicating whether skewness is not suggested during additional
     * cameras estimation.
     *
     * @return true if skewness is suggested, false otherwise.
     */
    public boolean isAdditionalCamerasSuggestSkewnessValueEnabled() {
        return mAdditionalCamerasSuggestSkewnessValueEnabled;
    }

    /**
     * Sets value indicating whether skewness is not suggested during additional
     * cameras estimation.
     *
     * @param additionalCamerasSuggestSkewnessValueEnabled true if skewness is suggested,
     *                                                     false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasSuggestSkewnessValueEnabled(
            final boolean additionalCamerasSuggestSkewnessValueEnabled) {
        mAdditionalCamerasSuggestSkewnessValueEnabled =
                additionalCamerasSuggestSkewnessValueEnabled;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value of skewness to be suggested when suggestion is enabled during
     * additional cameras estimation.
     *
     * @return value of skewness to be suggested when suggestion is enabled during
     * additional cameras estimation.
     */
    public double getAdditionalCamerasSuggestedSkewnessValue() {
        return mAdditionalCamerasSuggestedSkewnessValue;
    }

    /**
     * Sets value of skewness to be suggested when suggestion is enabled during additional
     * cameras estimation.
     *
     * @param additionalCamerasSuggestedSkewnessValue value of skewness to be suggested
     *                                                when suggestion is enabled during
     *                                                additional cameras estimation.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasSuggestedSkewnessValue(
            final double additionalCamerasSuggestedSkewnessValue) {
        mAdditionalCamerasSuggestedSkewnessValue = additionalCamerasSuggestedSkewnessValue;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether horizontal focal length value is suggested or not during
     * additional cameras estimation.
     *
     * @return true if horizontal focal length value is suggested, false otherwise.
     */
    public boolean isAdditionalCamerasSuggestHorizontalFocalLengthEnabled() {
        return mAdditionalCamerasSuggestHorizontalFocalLengthEnabled;
    }

    /**
     * Specifies whether horizontal focal length value is suggested or not during additional
     * cameras estimation.
     *
     * @param additionalCamerasSuggestHorizontalFocalLengthEnabled true if horizontal focal
     *                                                             length value is suggested, false
     *                                                             otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasSuggestHorizontalFocalLengthEnabled(
            final boolean additionalCamerasSuggestHorizontalFocalLengthEnabled) {
        mAdditionalCamerasSuggestHorizontalFocalLengthEnabled =
                additionalCamerasSuggestHorizontalFocalLengthEnabled;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value of suggested horizontal focal length during additional cameras
     * estimation.
     *
     * @return value of suggested horizontal focal length during additional cameras
     * estimation.
     */
    public double getAdditionalCamerasSuggestedHorizontalFocalLengthValue() {
        return mAdditionalCamerasSuggestedHorizontalFocalLengthValue;
    }

    /**
     * Sets value of suggested horizontal focal length during additional cameras
     * estimation.
     *
     * @param additionalCamerasSuggestedHorizontalFocalLengthValue value of suggested
     *                                                             horizontal focal length during
     *                                                             additional cameras estimation.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasSuggestedHorizontalFocalLengthValue(
            final double additionalCamerasSuggestedHorizontalFocalLengthValue) {
        mAdditionalCamerasSuggestedHorizontalFocalLengthValue =
                additionalCamerasSuggestedHorizontalFocalLengthValue;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value indicating whether vertical focal length value is suggested or not
     * during additional cameras estimation.
     *
     * @return true if vertical focal length value is suggested, false otherwise.
     */
    public boolean isAdditionalCamerasSuggestVerticalFocalLengthEnabled() {
        return mAdditionalCamerasSuggestVerticalFocalLengthEnabled;
    }

    /**
     * Sets value indicating whether vertical focal length value is suggested or not
     * during additional cameras estimation.
     *
     * @param additionalCamerasSuggestVerticalFocalLengthEnabled true if vertical focal
     *                                                           length is suggested, false
     *                                                           otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasSuggestVerticalFocalLengthEnabled(
            final boolean additionalCamerasSuggestVerticalFocalLengthEnabled) {
        mAdditionalCamerasSuggestVerticalFocalLengthEnabled =
                additionalCamerasSuggestVerticalFocalLengthEnabled;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value of suggested vertical focal length during additional cameras estimation.
     *
     * @return value of suggested vertical focal length during additional cameras
     * estimation.
     */
    public double getAdditionalCamerasSuggestedVerticalFocalLengthValue() {
        return mAdditionalCamerasSuggestedVerticalFocalLengthValue;
    }

    /**
     * Sets value of suggested vertical focal length during additional cameras estimation.
     *
     * @param additionalCamerasSuggestedVerticalFocalLengthValue value of suggested vertical
     *                                                           focal length during additional
     *                                                           cameras estimation.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasSuggestedVerticalFocalLengthValue(
            final double additionalCamerasSuggestedVerticalFocalLengthValue) {
        mAdditionalCamerasSuggestedVerticalFocalLengthValue =
                additionalCamerasSuggestedVerticalFocalLengthValue;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value indicating whether aspect ratio is suggested or not during additional
     * cameras estimation.
     *
     * @return true if aspect ratio is suggested, false otherwise.
     */
    public boolean isAdditionalCamerasSuggestAspectRatioEnabled() {
        return mAdditionalCamerasSuggestAspectRatioEnabled;
    }

    /**
     * Sets value indicating whether aspect ratio is suggested or not during additional
     * cameras estimation.
     *
     * @param additionalCamerasSuggestAspectRatioEnabled true if aspect ratio is suggested,
     *                                                   false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasSuggestAspectRatioEnabled(
            final boolean additionalCamerasSuggestAspectRatioEnabled) {
        mAdditionalCamerasSuggestAspectRatioEnabled =
                additionalCamerasSuggestAspectRatioEnabled;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value of aspect ratio to be suggested when suggestion is enabled during
     * additional cameras estimation.
     *
     * @return value of aspect ratio to be suggested.
     */
    public double getAdditionalCamerasSuggestedAspectRatioValue() {
        return mAdditionalCamerasSuggestedAspectRatioValue;
    }

    /**
     * Sets value of aspect ratio to be suggested when suggestion is enabled during
     * additional cameras estimation.
     *
     * @param additionalCamerasSuggestedAspectRatioValue value of aspect ratio to be
     *                                                   suggested.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasSuggestedAspectRatioValue(
            final double additionalCamerasSuggestedAspectRatioValue) {
        mAdditionalCamerasSuggestedAspectRatioValue =
                additionalCamerasSuggestedAspectRatioValue;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value indicating whether principal point is suggested or not during
     * additional cameras estimation.
     *
     * @return true if principal point is suggested, false otherwise.
     */
    public boolean isAdditionalCamerasSuggestPrincipalPointEnabled() {
        return mAdditionalCamerasSuggestPrincipalPointEnabled;
    }

    /**
     * Sets value indicating whether principal point is suggested or not during additional
     * cameras estimation.
     *
     * @param additionalCamerasSuggestPrincipalPointEnabled true if principal point is
     *                                                      suggested, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasSuggestPrincipalPointEnabled(
            final boolean additionalCamerasSuggestPrincipalPointEnabled) {
        mAdditionalCamerasSuggestPrincipalPointEnabled =
                additionalCamerasSuggestPrincipalPointEnabled;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets value of principal point to be suggested when suggestion is enabled during
     * additional cameras estimation.
     *
     * @return principal point to be suggested.
     */
    public InhomogeneousPoint2D getAdditionalCamerasSuggestedPrincipalPointValue() {
        return mAdditionalCamerasSuggestedPrincipalPointValue;
    }

    /**
     * Sets value of principal point to be suggested when suggestion is enabled during
     * additional cameras estimation.
     *
     * @param additionalCamerasSuggestedPrincipalPointValue principal point to be
     *                                                      suggested.
     * @return this instance so that method can be easily chained.
     */
    public T setAdditionalCamerasSuggestedPrincipalPointValue(
            final InhomogeneousPoint2D additionalCamerasSuggestedPrincipalPointValue) {
        mAdditionalCamerasSuggestedPrincipalPointValue =
                additionalCamerasSuggestedPrincipalPointValue;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Indicates whether homogeneous point triangulator must be used or not to estimate
     * 3D points when only two matches are available.
     *
     * @return true if homogeneous point triangulator must be used, false otherwise.
     */
    public boolean isHomogeneousPointTriangulatorUsed() {
        return mUseHomogeneousPointTriangulator;
    }

    /**
     * Specifies whether homogeneous point triangulator must be used or not to estimate
     * 3D points when only two matches are available.
     *
     * @param useHomogeneousPointTriangulator true if homogeneous point triangulator must
     *                                        be used, false otherwise.
     * @return this instance so that method can be easily chained.
     */
    public T setHomogeneousPointTriangulatorUsed(final boolean useHomogeneousPointTriangulator) {
        mUseHomogeneousPointTriangulator = useHomogeneousPointTriangulator;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets robust method for point triangulation when points are matched in more
     * than two views.
     *
     * @return robust method for point triangulation.
     */
    public RobustEstimatorMethod getRobustPointTriangulatorMethod() {
        return mRobustPointTriangulatorMethod;
    }

    /**
     * Sets robust method for point triangulation when points are matched in more
     * than two views.
     *
     * @param robustPointTriangulatorMethod robust method for point triangulation.
     * @return this instance so that method can be easily chained.
     */
    public T setRobustPointTriangulatorMethod(
            final RobustEstimatorMethod robustPointTriangulatorMethod) {
        mRobustPointTriangulatorMethod = robustPointTriangulatorMethod;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets confidence of robustly triangulated points. By default this is 99%.
     *
     * @return confidence of robustly triangulated points.
     */
    public double getPointTriangulatorConfidence() {
        return mPointTriangulatorConfidence;
    }

    /**
     * Sets confidence of robustly triangulated points. By default this is 99%.
     *
     * @param pointTriangulatorConfidence confidence of robustly triangulated points.
     * @return this instance so that method can be easily chained.
     */
    public T setPointTriangulatorConfidence(final double pointTriangulatorConfidence) {
        mPointTriangulatorConfidence = pointTriangulatorConfidence;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets maximum number of iterations to make while robustly estimating triangulated
     * points. By default this is 5000 iterations.
     *
     * @return maximum number of iterations to make while robustly estimating
     * triangulated points.
     */
    public int getPointTriangulatorMaxIterations() {
        return mPointTriangulatorMaxIterations;
    }

    /**
     * Sets maximum number of iterations to make while robustly estimating triangulated
     * points. By default this is 5000 iterations.
     *
     * @param pointTriangulatorMaxIterations maximum number of iterations to make while
     *                                       robustly estimating triangulated points.
     * @return this instance so that method can be easily chained.
     */
    public T setPointTriangulatorMaxIterations(final int pointTriangulatorMaxIterations) {
        mPointTriangulatorMaxIterations = pointTriangulatorMaxIterations;
        //noinspection unchecked
        return (T) this;
    }

    /**
     * Gets threshold to determine whether samples for robust point triangulator are
     * inliers or not.
     *
     * @return threshold to determine whether samples for robust point triangulator
     * are inliers or not.
     */
    public double getPointTriangulatorThreshold() {
        return mPointTriangulatorThreshold;
    }

    /**
     * Sets threshold to determine whether samples for robust point triangulator are
     * inliers or not.
     *
     * @param pointTriangulatorThreshold threshold to determine whether samples for
     *                                   robust point triangulator are inliers or not.
     * @return this instance so that method can be easily chained.
     */
    public T setPointTriangulatorThreshold(final double pointTriangulatorThreshold) {
        mPointTriangulatorThreshold = pointTriangulatorThreshold;
        //noinspection unchecked
        return (T) this;
    }
}
