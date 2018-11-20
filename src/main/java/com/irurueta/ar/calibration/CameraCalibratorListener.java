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

import com.irurueta.geometry.PinholeCameraIntrinsicParameters;

/**
 * Listener to be notified when calibration starts, finishes or any progress
 * changes.
 */
public interface CameraCalibratorListener {
    /**
     * Called when a calibrator starts the camera calibration process.
     * @param calibrator reference to a camera calibrator.
     */
    void onCalibrateStart(CameraCalibrator calibrator);
    
    /**
     * Called when a calibrator ends the camera calibration process.
     * @param calibrator reference to a camera calibrator.
     */
    void onCalibrateEnd(CameraCalibrator calibrator);
    
    /**
     * Called to notify changes in the camera calibration progress.
     * @param calibrator reference to a camera calibrator.
     * @param progress current percentage of progress expressed as a value
     * between 0.0f and 1.0f.
     */
    void onCalibrateProgressChange(CameraCalibrator calibrator,
            float progress);
    
    /**
     * Called when a calibrator starts the estimation of intrinsic parameters
     * of a pinhole camera.
     * Depending on the calibrator implementation, this method might be called
     * multiple times while the distortion parameters are being refined.
     * @param calibrator reference to a camera calibrator.
     */
    void onIntrinsicParametersEstimationStarts(
            CameraCalibrator calibrator);
    
    /**
     * Called when a calibrator finishes the estimation of intrinsic parameters
     * of a pinhole camera.
     * Depending on the calibrator implementation, this method might be called
     * multiple times while the distortion parameters are being refined.
     * @param calibrator reference to a camera calibrator.
     * @param intrinsicParameters intrinsic parameters that have been estimated
     * so far.
     */
    void onIntrinsicParametersEstimationEnds(CameraCalibrator calibrator,
            PinholeCameraIntrinsicParameters intrinsicParameters);
    
    /**
     * Called when a calibrator starts the estimation of radial distortion of
     * the camera lens.
     * Depending on the calibrator implementation, this method might be called
     * multiple times while the distortion parameters are being refined.
     * @param calibrator reference to a camera calibrator.
     */
    void onRadialDistortionEstimationStarts(CameraCalibrator calibrator);
    
    /**
     * Called when a calibrator finishes the estimation of radial distortion of
     * the camera lens.
     * Depending on the calibrator implementation, this method might be called
     * multiple times while the distortion parameters are being refined.
     * @param calibrator reference to a camera calibrator.
     * @param distortion radial distortion that has been estimated so far.
     */
    void onRadialDistortionEstimationEnds(CameraCalibrator calibrator,
            RadialDistortion distortion);
}
