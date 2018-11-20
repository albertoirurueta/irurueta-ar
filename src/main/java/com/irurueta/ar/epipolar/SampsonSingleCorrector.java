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
import com.irurueta.geometry.estimators.NotReadyException;

/**
 * Fixes a single matched pair of points so that they perfectly follow a given
 * epipolar geometry using the Sampson approximation.
 * When matching points typically the matching precission is about 1 pixel, 
 * however this makes that matched points under a given epipolar geometry (i.e.
 * fundamental or essential matrix), do not lie perfectly on the corresponding
 * epipolar plane or epipolar lines.
 * The consequence is that triangularization of these matches will fail or 
 * produce innaccurate results. 
 * By fixing matched points using a corrector following a given epipolar 
 * geometry, this effect is alleviated.
 * This corrector uses the Sampson approximation which is capable to remove 
 * small errors when matches are close to their real epipolar lines. This method
 * is faster than the Gold standard method, but can only correct small errors
 * (1 or 2 pixels).
 */
public class SampsonSingleCorrector extends SingleCorrector {
    
    /**
     * Constructor.
     */
    public SampsonSingleCorrector() {
        super();
    }
    
    /**
     * Constructor.
     * @param fundamentalMatrix fundamental matrix defining the epipolar 
     * geometry.
     */
    public SampsonSingleCorrector(FundamentalMatrix fundamentalMatrix) {
        super(fundamentalMatrix);
    }
    
    /**
     * Constructor.
     * @param leftPoint matched point on left view to be corrected.
     * @param rightPoint matched point on right view to be corrected.
     */
    public SampsonSingleCorrector(Point2D leftPoint, Point2D rightPoint) {
        super(leftPoint, rightPoint);
    }
    
    /**
     * Constructor.
     * @param leftPoint matched point on left view to be corrected.
     * @param rightPoint matched point on right view to be corrected.
     * @param fundamentalMatrix fundamental matrix defining an epipolar geometry.
     */
    public SampsonSingleCorrector(Point2D leftPoint, Point2D rightPoint,
            FundamentalMatrix fundamentalMatrix) {
        super(leftPoint, rightPoint, fundamentalMatrix);
    }
    
    /**
     * Corrects the pair of provided matched points to be corrected.
     * @throws NotReadyException if this instance is not ready (either points or
     * fundamental matrix has not been provided yet).
     */
    @Override
    public void correct() throws NotReadyException {
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        mLeftCorrectedPoint = Point2D.create(
                CoordinatesType.HOMOGENEOUS_COORDINATES);
        mRightCorrectedPoint = Point2D.create(
                CoordinatesType.HOMOGENEOUS_COORDINATES);        
        correct(mLeftPoint, mRightPoint, mFundamentalMatrix, 
                mLeftCorrectedPoint, mRightCorrectedPoint);
    }
    
    /**
     * Gets type of correction being used.
     * @return type of correction.
     */
    @Override
    public CorrectorType getType() {
        return CorrectorType.SAMPSON_CORRECTOR;
    }
    
    /**
     * Corrects the pair of provided matched points to be corrected using
     * provided fundamental matrix and stores the corrected points into provided
     * instances.
     * @param leftPoint point on left view to be corrected.
     * @param rightPoint point on right view to be corrected.
     * @param fundamentalMatrix fundamental matrix defining the epipolar 
     * geometry.
     * @param correctedLeftPoint point on left view after correction.
     * @param correctedRightPoint point on right view after correction.
     * @throws NotReadyException if provided fundamental matrix is not ready.
     */
    public static void correct(Point2D leftPoint, Point2D rightPoint,
            FundamentalMatrix fundamentalMatrix, Point2D correctedLeftPoint,
            Point2D correctedRightPoint) throws NotReadyException {

        
        Line2D leftEpipolarLine = new Line2D();
        Line2D rightEpipolarLine = new Line2D();
        correct(leftPoint, rightPoint, fundamentalMatrix, correctedLeftPoint,
                correctedRightPoint, leftEpipolarLine, rightEpipolarLine);
    }
    
    /**
     * Corrects the pair of provided matched points to be corrected using 
     * provided fundamental matrix and stores the corrected points into provided
     * instances. Provided epipolar lines are used for memory efficiency 
     * purposes only, so that those instances can be reused.
     * @param leftPoint point on left view to be corrected.
     * @param rightPoint point on right view to be corrected.
     * @param fundamentalMatrix fundamental matrix defining the epipolar 
     * geometry.
     * @param correctedLeftPoint point on left view after correction.
     * @param correctedRightPoint point on right view after correction.
     * @param leftEpipolarLine left epipolar line to be reused for memory 
     * efficiency purposes.
     * @param rightEpipolarLine right epipolar line to be reused for memory
     * efficiency purposes.
     * @throws NotReadyException if provided fundamental matrix is not ready.
     */
    protected static void correct(Point2D leftPoint, Point2D rightPoint,
            FundamentalMatrix fundamentalMatrix, Point2D correctedLeftPoint,
            Point2D correctedRightPoint, Line2D leftEpipolarLine, 
            Line2D rightEpipolarLine) throws NotReadyException {
        
        //normalize to increase accuracy
        leftPoint.normalize();
        rightPoint.normalize();
        fundamentalMatrix.normalize();
        
        fundamentalMatrix.leftEpipolarLine(rightPoint, leftEpipolarLine);
        fundamentalMatrix.rightEpipolarLine(leftPoint, rightEpipolarLine);
        
        leftEpipolarLine.normalize();
        rightEpipolarLine.normalize();
        
        double numerator = rightPoint.getHomX() * rightEpipolarLine.getA() +
                rightPoint.getHomY() * rightEpipolarLine.getB() +
                rightPoint.getHomW() * rightEpipolarLine.getC();
        
        double denominator = Math.pow(rightEpipolarLine.getA(), 2.0) +
                Math.pow(rightEpipolarLine.getB(), 2.0) +
                Math.pow(leftEpipolarLine.getA(), 2.0) +
                Math.pow(leftEpipolarLine.getB(), 2.0);
        
        double factor = numerator / denominator;
        
        double leftCorrectionX = factor * leftEpipolarLine.getA();
        double leftCorrectionY = factor * leftEpipolarLine.getB();
        double rightCorrectionX = factor * rightEpipolarLine.getA();
        double rightCorrectionY = factor * rightEpipolarLine.getB();
        
        double correctedLeftX = leftPoint.getInhomX() - leftCorrectionX;
        double correctedLeftY = leftPoint.getInhomY() - leftCorrectionY;
        double correctedRightX = rightPoint.getInhomX() - rightCorrectionX;
        double correctedRightY = rightPoint.getInhomY() - rightCorrectionY;
        
        correctedLeftPoint.setInhomogeneousCoordinates(correctedLeftX, 
                correctedLeftY);
        correctedRightPoint.setInhomogeneousCoordinates(correctedRightX, 
                correctedRightY);
    }
}
