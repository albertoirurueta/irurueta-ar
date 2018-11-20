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

import com.irurueta.algebra.*;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.*;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class EssentialMatrixTest {
    
    private static final int ESSENTIAL_MATRIX_ROWS = 3;
    private static final int ESSENTIAL_MATRIX_COLS = 3;
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;
    private static final double MIN_SKEWNESS = -1.0;
    private static final double MAX_SKEWNESS = 1.0;
    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;
    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = 30.0;
    
    private static final double MIN_CAMERA_SEPARATION = 5.0;
    private static final double MAX_CAMERA_SEPARATION = 10.0;
    
    private static final int TIMES = 100;
    
    public EssentialMatrixTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testEmptyContructor() {
        EssentialMatrix essentialMatrix = new EssentialMatrix();
        
        assertFalse(essentialMatrix.isInternalMatrixAvailable());
        try {
            essentialMatrix.getInternalMatrix();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException ignore) { }
    }
    
    @Test
    public void testConstructorWithInternalMatrix() throws WrongSizeException, 
            NotReadyException, LockedException, DecomposerException, 
            com.irurueta.algebra.NotAvailableException,
            InvalidEssentialMatrixException, NotAvailableException {
        for (int t = 0; t < TIMES; t++) {
            Matrix internalMatrix = Matrix.createWithUniformRandomValues(
                    ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    internalMatrix);
            decomposer.decompose();

            Matrix U = decomposer.getU();
            Matrix W = decomposer.getW();
            Matrix V = decomposer.getV();
            Matrix transV = V.transposeAndReturnNew();

            //Set last singular value to zero to enforce rank 2, and set equal 
            //singular values for non-zero ones
            W.setElementAt(0, 0, 1.0);
            W.setElementAt(1, 1, 1.0);
            W.setElementAt(2, 2, 0.0);

            internalMatrix = U.multiplyAndReturnNew(W.multiplyAndReturnNew(
                    transV));

            EssentialMatrix essentialMatrix = new EssentialMatrix(internalMatrix);
            assertTrue(essentialMatrix.isInternalMatrixAvailable());
            assertEquals(essentialMatrix.getInternalMatrix(), internalMatrix);

            //Force IllegalArgumentException
            essentialMatrix = null;
            try {
                essentialMatrix = new EssentialMatrix(internalMatrix, -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            assertNull(essentialMatrix);
            
            //Force InvalidEssentialMatrixException
            internalMatrix = Matrix.createWithUniformRandomValues(
                    ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer = new SingularValueDecomposer(internalMatrix);
            decomposer.decompose();

            U = decomposer.getU();
            W = decomposer.getW();
            V = decomposer.getV();
            transV = V.transposeAndReturnNew();

            //set non equal singular vlaues
            W.setElementAt(0, 0, 1.0);
            W.setElementAt(1, 1, 2.0);
            W.setElementAt(2, 2, 3.0);

            internalMatrix = U.multiplyAndReturnNew(W.multiplyAndReturnNew(
                    transV));

            essentialMatrix = null;
            try {
                essentialMatrix = new EssentialMatrix(internalMatrix);
                fail("InvalidEssentialMatrixException expected but not thrown");
            } catch (InvalidEssentialMatrixException ignore) { }
            assertNull(essentialMatrix);
        }        
    }

    @Test
    public void testConstructorWithTwoPinholeCameras() 
            throws InvalidPairOfCamerasException, WrongSizeException, 
            RankDeficientMatrixException, DecomposerException, 
            com.irurueta.geometry.estimators.NotReadyException, 
            NotAvailableException {
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);

            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            Point3D cameraCenter1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            Point3D cameraCenter2 = new InhomogeneousPoint3D(
                    cameraCenter1.getInhomX() + cameraSeparation,
                    cameraCenter1.getInhomY() + cameraSeparation,
                    cameraCenter1.getInhomZ() + cameraSeparation);

            Rotation3D rotation1 = new MatrixRotation3D(alphaEuler1, betaEuler1, 
                    gammaEuler1);
            Rotation3D rotation2 = new MatrixRotation3D(alphaEuler2, betaEuler2,
                    gammaEuler2);

            PinholeCameraIntrinsicParameters intrinsic1 = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                    verticalFocalLength2, horizontalPrincipalPoint2, 
                    verticalPrincipalPoint2, skewness2);        

            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    cameraCenter1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    cameraCenter2);

            //estimate essential matrix using provided cameras
            EssentialMatrix essentialMatrix = new EssentialMatrix(camera1, 
                    camera2);

            //now normalize cameras by their intrinsic parameters and compute 
            //their fundamental matrix
            Matrix cam1InternalMatrix = camera1.getInternalMatrix();
            Matrix cam1IntrinsicParameters = intrinsic1.getInternalMatrix();
            Matrix inverseCam1IntrinsicParameters = Utils.inverse(
                    cam1IntrinsicParameters);
            Matrix newCam1InternalMatrix = inverseCam1IntrinsicParameters.
                    multiplyAndReturnNew(cam1InternalMatrix);
            camera1.setInternalMatrix(newCam1InternalMatrix);

            Matrix cam2InternalMatrix = camera2.getInternalMatrix();
            Matrix cam2IntrinsicParameters = intrinsic2.getInternalMatrix();
            Matrix inverseCam2IntrinsicParameters = Utils.inverse(
                    cam2IntrinsicParameters);
            Matrix newCam2InternalMatrix = inverseCam2IntrinsicParameters.
                    multiplyAndReturnNew(cam2InternalMatrix);
            camera2.setInternalMatrix(newCam2InternalMatrix);

            //normalize cameras to increase accuracy
            camera1.normalize();
            camera2.normalize();

            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1,
                    camera2);

            //check equality up to scale
            fundamentalMatrix.normalize();
            essentialMatrix.normalize();

            Matrix Finternal = fundamentalMatrix.getInternalMatrix();
            Matrix Einternal = essentialMatrix.getInternalMatrix();
            //noinspection all
            double firstScale = Finternal.getElementAtIndex(0) / 
                    Einternal.getElementAtIndex(0);
            double previousScale = firstScale;
            double currentScale = 0.0;
            for(int i = 1; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++){
                currentScale = Finternal.getElementAtIndex(i) / 
                        Einternal.getElementAtIndex(i);
                assertEquals(previousScale - currentScale, 0.0, ABSOLUTE_ERROR);
                previousScale = currentScale;
            }
            assertEquals(previousScale - currentScale, 0.0, ABSOLUTE_ERROR);
            
            //Force IllegalArgumentException
            essentialMatrix = null;
            try {
                essentialMatrix = new EssentialMatrix(camera1, camera2, -1.0);
                fail("IllegalArgumentException expected but not thrown");
            }catch(IllegalArgumentException ignore) { }
            assertNull(essentialMatrix);
        }
    }
    
    @Test
    public void testConstructorWithTranslationAndRotation() 
            throws InvalidRotationAndTranslationException, 
            NotAvailableException, NotReadyException, LockedException,
            DecomposerException, com.irurueta.algebra.NotAvailableException, 
            WrongSizeException, InvalidRotationMatrixException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            HomogeneousPoint2D translation = new HomogeneousPoint2D(
                    cameraSeparation, cameraSeparation, cameraSeparation);

            MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, 
                    betaEuler, gammaEuler);

            EssentialMatrix essentialMatrix = new EssentialMatrix(rotation, 
                    translation);

            Matrix internalEssentialMatrix = 
                    essentialMatrix.getInternalMatrix();

            SingularValueDecomposer singularValueDecomposer = 
                    new SingularValueDecomposer(internalEssentialMatrix);
            singularValueDecomposer.decompose();
            Matrix U = singularValueDecomposer.getU();

            double scaleX = U.getElementAt(0, 2) / translation.getHomX();
            double scaleY = U.getElementAt(1, 2) / translation.getHomY();
            double scaleW = U.getElementAt(2, 2) / translation.getHomW();

            assertEquals(scaleX - scaleY, 0.0, ABSOLUTE_ERROR);
            assertEquals(scaleY - scaleW, 0.0, ABSOLUTE_ERROR);
            assertEquals(scaleW - scaleX, 0.0, ABSOLUTE_ERROR);

            Matrix W = new Matrix(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS);
            W.setElementAt(0, 1, -1.0);
            W.setElementAt(1, 0, 1.0);
            W.setElementAt(2, 2, 1.0);

            Matrix transW = W.transposeAndReturnNew();
            Matrix V = singularValueDecomposer.getV();
            Matrix transV = V.transposeAndReturnNew();

            //First possible rotation
            Matrix rotation1Matrix = U.multiplyAndReturnNew(
                    W.multiplyAndReturnNew(transV));
            MatrixRotation3D rotation1 = new MatrixRotation3D(rotation1Matrix);
            //second possible rotation
            Matrix rotation2Matrix = U.multiplyAndReturnNew(transW.
                    multiplyAndReturnNew(transV));
            MatrixRotation3D rotation2 = new MatrixRotation3D(rotation2Matrix);

            boolean valid = (Math.abs(Math.abs(rotation1.getAlphaEulerAngle()) -
                    Math.abs(rotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation2.getAlphaEulerAngle()) -
                    Math.abs(rotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation1.getBetaEulerAngle()) -
                    Math.abs(rotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation2.getBetaEulerAngle()) -
                    Math.abs(rotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation1.getGammaEulerAngle()) -
                    Math.abs(rotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation2.getGammaEulerAngle()) -
                    Math.abs(rotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);
            
            if (valid) {
                numValid++;
            }
            
            //Force IllegalArgumentException
            essentialMatrix = null;
            try {
                essentialMatrix = new EssentialMatrix(rotation, translation, 
                        -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            assertNull(essentialMatrix);
        }
        
        assertTrue(numValid > TIMES / 4);
    }
    
    @Test
    public void testConstructorWithRotationAndCameraCenter() 
            throws InvalidRotationAndTranslationException, 
            InvalidEssentialMatrixException, NotAvailableException, 
            WrongSizeException {
        
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
            
            MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, 
                    betaEuler, gammaEuler);
            Point3D center = new InhomogeneousPoint3D(cameraSeparation, 
                    cameraSeparation, cameraSeparation);
            
            //build essential matrix using provided rotation and center
            EssentialMatrix essentialMatrix = new EssentialMatrix(rotation, 
                    center);
            
            //compute possible rotations and translations
            essentialMatrix.computePossibleRotationAndTranslations();
            
            //check correctness
            Point2D firstEstimatedTranslation = 
                    essentialMatrix.getFirstPossibleTranslation();
            Point2D secondEstimatedTranslation = 
                    essentialMatrix.getSecondPossibleTranslation();
            MatrixRotation3D firstEstimatedCameraRotation = 
                    (MatrixRotation3D)essentialMatrix.getFirstPossibleRotation();
            MatrixRotation3D secondEstimatedCameraRotation = 
                    (MatrixRotation3D)essentialMatrix.getSecondPossibleRotation();            
            
            boolean valid = (Math.abs(Math.abs(rotation.getAlphaEulerAngle()) - 
                    Math.abs(firstEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getAlphaEulerAngle()) - 
                    Math.abs(secondEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getBetaEulerAngle()) - 
                    Math.abs(firstEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getBetaEulerAngle()) - 
                    Math.abs(secondEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getGammaEulerAngle()) - 
                    Math.abs(firstEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getGammaEulerAngle()) - 
                    Math.abs(secondEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);        

            if (valid) {
                numValid++;
            }
            
            
            //translation term is equal to t=-R*C, hence we can obtain camera
            //centers as: C = -inv(R)*t = -R'*t
            Matrix rotationMatrix1 = 
                    firstEstimatedCameraRotation.getInternalMatrix();
            Matrix rotationMatrix2 =
                    secondEstimatedCameraRotation.getInternalMatrix();
            Matrix transRotationMatrix1 = rotationMatrix1.
                    transposeAndReturnNew();
            Matrix transRotationMatrix2 = rotationMatrix2.
                    transposeAndReturnNew();
            Matrix translationMatrix1 = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix1.setElementAt(0, 0, 
                    firstEstimatedTranslation.getHomX());
            translationMatrix1.setElementAt(1, 0, 
                    firstEstimatedTranslation.getHomY());
            translationMatrix1.setElementAt(2, 0, 
                    firstEstimatedTranslation.getHomW());
            Matrix translationMatrix2 = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix2.setElementAt(0, 0, 
                    secondEstimatedTranslation.getHomX());
            translationMatrix2.setElementAt(1, 0, 
                    secondEstimatedTranslation.getHomY());
            translationMatrix2.setElementAt(2, 0, 
                    secondEstimatedTranslation.getHomW());
            
            Matrix centerMatrix1 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            Matrix centerMatrix2 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);
            Matrix centerMatrix3 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            Matrix centerMatrix4 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);
            
            double scaleX, scaleY, scaleZ;
            scaleX = centerMatrix1.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix1.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix1.getElementAt(2, 0) / center.getInhomZ();
            
            boolean valid1 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix2.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix2.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix2.getElementAt(2, 0) / center.getInhomZ();
            
            boolean valid2 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix3.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix3.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix3.getElementAt(2, 0) / center.getInhomZ();
            
            boolean valid3 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix4.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix4.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix4.getElementAt(2, 0) / center.getInhomZ();
            
            boolean valid4 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);
            
            assertTrue(valid1 || valid2 || valid3 || valid4);
            
            //Force IllegalArgumentException
            essentialMatrix = null;
            try {
                essentialMatrix = new EssentialMatrix(rotation, center, -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            assertNull(essentialMatrix);
        }
        assertTrue(numValid > TIMES / 4);
    }
    
    @Test
    public void testConstructorWithFundamentalMatrixAndIntrinsicParameters() 
            throws WrongSizeException, NotReadyException, LockedException, 
            DecomposerException, com.irurueta.algebra.NotAvailableException, 
            InvalidFundamentalMatrixException,
            InvalidPairOfIntrinsicParametersException, NotAvailableException {
        for (int t = 0; t < TIMES; t++) {
            Matrix a = Matrix.createWithUniformRandomValues(
                    ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();
            Matrix U = decomposer.getU();
            Matrix W = decomposer.getW();
            Matrix V = decomposer.getV();
            Matrix transV = V.transposeAndReturnNew();

            //Set last singular value to zero to enforce rank 2
            W.setElementAt(2, 2, 0.0);
            Matrix fundamentalInternalMatrix = U.multiplyAndReturnNew(
                    W.multiplyAndReturnNew(transV));

            //Creating the intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);    

            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);

            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic1 = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1, 
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2, 
                    verticalFocalLength2, horizontalPrincipalPoint2, 
                    verticalPrincipalPoint2, skewness2);

            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    fundamentalInternalMatrix);

            EssentialMatrix essentialMatrix = new EssentialMatrix(
                    fundamentalMatrix, intrinsic1, intrinsic2);

            Matrix kLeftMatrix = intrinsic1.getInternalMatrix();
            double normK1 = Utils.normF(kLeftMatrix);
            Matrix k1 = kLeftMatrix.multiplyByScalarAndReturnNew(1.0 / normK1);

            Matrix kRightMatrix = intrinsic2.getInternalMatrix();
            double normK2 = Utils.normF(kRightMatrix);
            Matrix k2 = kRightMatrix.multiplyByScalarAndReturnNew(1.0 / normK2);
            Matrix transK2 = k2.transposeAndReturnNew();

            double normFund = Utils.normF(fundamentalInternalMatrix);
            Matrix tempFundMatrix = fundamentalInternalMatrix.
                    multiplyByScalarAndReturnNew(1.0 / normFund);

            Matrix estimatedEssentialMatrix = transK2.multiplyAndReturnNew(
                    tempFundMatrix.multiplyAndReturnNew(k1));

            double normEssential = Utils.normF(estimatedEssentialMatrix);
            estimatedEssentialMatrix.multiplyByScalarAndReturnNew(
                    1.0 / normEssential);

            Matrix Einternal2 = essentialMatrix.getInternalMatrix();
            double firstScale = Einternal2.getElementAtIndex(0) / 
                    estimatedEssentialMatrix.getElementAtIndex(0);
            double previousScale = firstScale, currentScale = 0.0;
            for (int i = 1; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
                currentScale = Einternal2.getElementAtIndex(i) / 
                        estimatedEssentialMatrix.getElementAtIndex(i);
                assertEquals(previousScale - currentScale, 0.0, ABSOLUTE_ERROR);
                previousScale = currentScale;
            }
            assertEquals(currentScale - firstScale, 0.0, ABSOLUTE_ERROR);
        }
    }
    
    @Test
    public void testSetInternalMatrix() throws WrongSizeException, 
            NotReadyException, LockedException, DecomposerException, 
            com.irurueta.algebra.NotAvailableException, 
            InvalidEssentialMatrixException, NotAvailableException {
        
        for (int t = 0; t < TIMES; t++) {
            
            EssentialMatrix essentialMatrix = new EssentialMatrix();

            assertFalse(essentialMatrix.isInternalMatrixAvailable());

            //Force NotAvailableException
            try {
                essentialMatrix.getInternalMatrix();
                fail("NotAvailableException expected but not thrown");
            } catch (NotAvailableException ignore) { }

            Matrix internalMatrix = Matrix.createWithUniformRandomValues(
                    ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    internalMatrix);
            decomposer.decompose();

            Matrix U = decomposer.getU();
            Matrix W = decomposer.getW();
            Matrix V = decomposer.getV();
            Matrix transV = V.transposeAndReturnNew();

            //Set last singular value to zero to enforce rank 2
            W.setElementAt(0, 0, 1.0);
            W.setElementAt(1, 1, 1.0);
            W.setElementAt(2, 2, 0.0);

            internalMatrix = U.multiplyAndReturnNew(W.multiplyAndReturnNew(
                    transV));

            essentialMatrix.setInternalMatrix(internalMatrix);

            assertTrue(essentialMatrix.isInternalMatrixAvailable());
            assertEquals(essentialMatrix.getInternalMatrix(), internalMatrix);

            //Force IllegalArgumentException
            try {
                essentialMatrix.setInternalMatrix(internalMatrix, -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            
            //Force InvalidEssentialMatrixException by setting wrong rank
            essentialMatrix = new EssentialMatrix();
            internalMatrix = Matrix.createWithUniformRandomValues(
                    ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            decomposer = new SingularValueDecomposer(internalMatrix);
            decomposer.decompose();

            U = decomposer.getU();
            W = decomposer.getW();
            V = decomposer.getV();
            transV = V.transposeAndReturnNew();

            //Enforce wrong rank
            W.setElementAt(0, 0, 1.0);
            W.setElementAt(1, 1, 2.0);
            W.setElementAt(2, 2, 3.0);

            internalMatrix = U.multiplyAndReturnNew(W.multiplyAndReturnNew(
                    transV));

            try {
                essentialMatrix.setInternalMatrix(internalMatrix);
                fail("InvalidFundamentalMatrixException expected but not " + 
                        "thrown");
            } catch (InvalidEssentialMatrixException ignore) { }

            //Force InvalidEssentialMatrixException by setting wrong singular 
            //values
            essentialMatrix = new EssentialMatrix();
            internalMatrix = Matrix.createWithUniformRandomValues(
                    ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            decomposer = new SingularValueDecomposer(internalMatrix);
            decomposer.decompose();

            U = decomposer.getU();
            W = decomposer.getW();
            V = decomposer.getV();
            transV = V.transposeAndReturnNew();

            //Enforce wrong rank
            W.setElementAt(0, 0, 1.5);
            W.setElementAt(1, 1, 1.0);
            W.setElementAt(2, 2, 0.0);

            internalMatrix = U.multiplyAndReturnNew(W.multiplyAndReturnNew(
                    transV));

            try {
                essentialMatrix.setInternalMatrix(internalMatrix);
                fail("InvalidFundamentalMatrixException expected but not " + 
                        "thrown");
            } catch (InvalidEssentialMatrixException ignore) { }
        }
    }
    
    @Test
    public void testSetFromPairOfCameras() throws InvalidPairOfCamerasException, 
            WrongSizeException, RankDeficientMatrixException, 
            DecomposerException, 
            com.irurueta.geometry.estimators.NotReadyException, 
            NotAvailableException {
        
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);

            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            Point3D center1 = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            PinholeCameraIntrinsicParameters intrinsic1 = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1, 
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2, 
                    verticalFocalLength2, horizontalPrincipalPoint2, 
                    verticalPrincipalPoint2, skewness2);        

            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    center1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2,
                    center2);

            EssentialMatrix essentialMatrix = new EssentialMatrix();

            //set from pair of cameras
            essentialMatrix.setFromPairOfCameras(camera1, camera2);

            //check correctness
            Matrix cam1InternalMatrix = camera1.getInternalMatrix();
            Matrix cam1IntrinsicParameters = intrinsic1.getInternalMatrix();
            Matrix inverseCam1IntrinsicParameters = Utils.inverse(
                    cam1IntrinsicParameters);
            Matrix newCam1InternalMatrix = inverseCam1IntrinsicParameters.
                    multiplyAndReturnNew(cam1InternalMatrix);

            camera1.setInternalMatrix(newCam1InternalMatrix);

            Matrix cam2InternalMatrix = camera2.getInternalMatrix();
            Matrix cam2IntrinsicParameters = intrinsic2.getInternalMatrix();
            Matrix inverseCam2IntrinsicParameters = Utils.inverse(
                    cam2IntrinsicParameters);
            Matrix newCam2InternalMatrix = inverseCam2IntrinsicParameters.
                    multiplyAndReturnNew(cam2InternalMatrix);

            camera2.setInternalMatrix(newCam2InternalMatrix);

            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, 
                    camera2);

            //check equality up to scale
            fundamentalMatrix.normalize();
            essentialMatrix.normalize();

            Matrix Finternal = fundamentalMatrix.getInternalMatrix();
            Matrix Einternal = essentialMatrix.getInternalMatrix();
            double firstScale = Finternal.getElementAtIndex(0) / 
                    Einternal.getElementAtIndex(0);
            double previousScale = firstScale, currentScale = 0.0;
            for (int i = 0; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
                currentScale = Finternal.getElementAtIndex(i) / 
                        Einternal.getElementAtIndex(i);
                assertEquals(previousScale - currentScale, 0.0, ABSOLUTE_ERROR);
                previousScale = currentScale;
            }
            assertEquals(currentScale - firstScale, 0.0, ABSOLUTE_ERROR);
            
            //Force IllegalArgumentException
            try {
                essentialMatrix.setFromPairOfCameras(camera1, camera2, -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
        }
    }     
    
    @Test
    public void testSetFromRotationAndTranslation() throws 
            InvalidRotationAndTranslationException, NotAvailableException, 
            NotReadyException, LockedException, DecomposerException, 
            com.irurueta.algebra.NotAvailableException, WrongSizeException, 
            InvalidRotationMatrixException {
        
        int numValid = 0;
        for (int t = 0; t < 2 * TIMES; t++) {
                    
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

                double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;
                double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                        MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Point2D translation = new HomogeneousPoint2D(cameraSeparation,
                    cameraSeparation, cameraSeparation);

            MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, 
                    betaEuler, gammaEuler);

            EssentialMatrix essentialMatrix = new EssentialMatrix();

            //set from rotation and translation
            essentialMatrix.setFromRotationAndTranslation(rotation, 
                    translation);

            //check correctness
            Matrix internalEssentialMatrix = essentialMatrix.
                    getInternalMatrix();

            SingularValueDecomposer singularValueDecomposer =
                    new SingularValueDecomposer(internalEssentialMatrix);
            singularValueDecomposer.decompose();

            Matrix U = singularValueDecomposer.getU();

            double scaleX = U.getElementAt(0, 2) / translation.getHomX();
            double scaleY = U.getElementAt(1, 2) / translation.getHomY();
            double scaleW = U.getElementAt(2, 2) / translation.getHomW();

            assertEquals(scaleX - scaleY, 0.0, ABSOLUTE_ERROR);
            assertEquals(scaleY - scaleW, 0.0, ABSOLUTE_ERROR);
            assertEquals(scaleW - scaleX, 0.0, ABSOLUTE_ERROR);

            Matrix W = new Matrix(ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS);
            W.setElementAt(0, 1, -1.0);
            W.setElementAt(1, 0, 1.0);
            W.setElementAt(2, 2, 1.0);
            Matrix transW = W.transposeAndReturnNew();
            Matrix V = singularValueDecomposer.getV();
            Matrix transV = V.transposeAndReturnNew();


            //First possible rotation
            Matrix rotation1Matrix = U.multiplyAndReturnNew(
                    W.multiplyAndReturnNew(transV));
            MatrixRotation3D rotation1 = new MatrixRotation3D(rotation1Matrix);
            //second possible rotation
            Matrix rotation2Matrix = U.multiplyAndReturnNew(
                    transW.multiplyAndReturnNew(transV));
            MatrixRotation3D rotation2 = new MatrixRotation3D(rotation2Matrix);
        
            boolean valid = (Math.abs(Math.abs(rotation1.getAlphaEulerAngle()) -
                    Math.abs(rotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation2.getAlphaEulerAngle()) -
                    Math.abs(rotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation1.getBetaEulerAngle()) -
                    Math.abs(rotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation2.getBetaEulerAngle()) -
                    Math.abs(rotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation1.getGammaEulerAngle()) -
                    Math.abs(rotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation2.getGammaEulerAngle()) -
                    Math.abs(rotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);
            
            if(valid) numValid++;
            
            //Force IllegalArgumentException
            try {
                essentialMatrix.setFromRotationAndTranslation(rotation, 
                        translation, -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
            
            if (numValid > 2*TIMES / 4) {
                break;
            }
        }
        
        assertTrue(numValid > 2*TIMES / 4);
    }
    
    @Test
    public void testSetFromFundamentalMatrixAndIntrinsics() throws 
            WrongSizeException, NotReadyException, LockedException, 
            DecomposerException, com.irurueta.algebra.NotAvailableException, 
            InvalidFundamentalMatrixException, 
            InvalidPairOfIntrinsicParametersException, NotAvailableException {
        
        for (int t = 0; t < TIMES; t++) {
            Matrix a = Matrix.createWithUniformRandomValues(
                    ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();
            Matrix U = decomposer.getU();
            Matrix W = decomposer.getW();
            Matrix V = decomposer.getV();

            Matrix transV = V.transposeAndReturnNew();

            //set last singular value to zero to enforce rank 2
            W.setElementAt(2, 2, 0.0);
            Matrix fundamentalInternalMatrix = U.multiplyAndReturnNew(
                    W.multiplyAndReturnNew(transV));

            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);

            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic1 = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1, 
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2, 
                    verticalFocalLength2, horizontalPrincipalPoint2, 
                    verticalPrincipalPoint2, skewness2);

            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(
                    fundamentalInternalMatrix);

            EssentialMatrix essentialMatrix = new EssentialMatrix();

            //set from fundamental matrix and intrinsics
            essentialMatrix.setFromFundamentalMatrixAndIntrinsics(
                    fundamentalMatrix, intrinsic1, intrinsic2);

            Matrix kLeftMatrix = intrinsic1.getInternalMatrix();
            double normK1 = Utils.normF(kLeftMatrix);
            Matrix K1 = kLeftMatrix.multiplyByScalarAndReturnNew(1.0 / normK1);

            Matrix kRightMatrix = intrinsic2.getInternalMatrix();
            double normK2 = Utils.normF(kRightMatrix);
            Matrix K2 = kRightMatrix.multiplyByScalarAndReturnNew(1.0 / normK2);

            Matrix transK2 = K2.transposeAndReturnNew();

            double normFund = Utils.normF(fundamentalInternalMatrix);
            Matrix tempFundMatrix = fundamentalInternalMatrix.
                    multiplyByScalarAndReturnNew(1.0 / normFund);

            Matrix estimatedEssentialMatrix =
                    transK2.multiplyAndReturnNew(
                    tempFundMatrix.multiplyAndReturnNew(K1));

            double normEssential = Utils.normF(estimatedEssentialMatrix);
            estimatedEssentialMatrix.multiplyByScalar(1.0 / normEssential);

            Matrix Einternal2 = essentialMatrix.getInternalMatrix();
            double firstScale = Einternal2.getElementAtIndex(0) / 
                    estimatedEssentialMatrix.getElementAtIndex(0);
            double previousScale = firstScale, currentScale = 0.0;
            for (int i = 1; i < ESSENTIAL_MATRIX_ROWS * ESSENTIAL_MATRIX_COLS; i++) {
                currentScale = Einternal2.getElementAtIndex(i) / 
                        estimatedEssentialMatrix.getElementAtIndex(i);
                assertEquals(previousScale - currentScale, 0.0, ABSOLUTE_ERROR);
                previousScale = currentScale;
            }
            assertEquals(currentScale - firstScale, 0.0, ABSOLUTE_ERROR);
        }
    }
    
    @Test
    public void testToFundamentalMatrix() throws
            EpipolarException,
            com.irurueta.geometry.estimators.NotReadyException, 
            NotAvailableException, WrongSizeException, NotReadyException, 
            LockedException, DecomposerException, 
            com.irurueta.algebra.NotAvailableException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            Matrix a = Matrix.createWithUniformRandomValues(
                    ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();
            Matrix U = decomposer.getU();
            Matrix W = decomposer.getW();
            Matrix V = decomposer.getV();

            Matrix transV = V.transposeAndReturnNew();

            //set last singular value to zero to enforce rank 2
            W.setElementAt(2, 2, 0.0);
            Matrix fundamentalInternalMatrix = U.multiplyAndReturnNew(
                    W.multiplyAndReturnNew(transV));

            //create intrinsic parameters
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);

            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic1 = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1, 
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2, 
                    verticalFocalLength2, horizontalPrincipalPoint2, 
                    verticalPrincipalPoint2, skewness2);

            FundamentalMatrix fundamentalMatrix1 = new FundamentalMatrix(
                    fundamentalInternalMatrix);
            
            EssentialMatrix essential = new EssentialMatrix(fundamentalMatrix1, 
                    intrinsic1, intrinsic2);
            
            FundamentalMatrix fundamentalMatrix2 = 
                    essential.toFundamentalMatrix(intrinsic1, intrinsic2);
            fundamentalMatrix2.normalize();

            boolean condition = fundamentalMatrix1.getInternalMatrix().equals(
                    fundamentalMatrix2.getInternalMatrix(), ABSOLUTE_ERROR);
            if (!condition) {
                continue;
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }
    
    @Test
    public void testSetFromRotationAndCameraCenter() 
            throws InvalidRotationAndTranslationException, 
            InvalidEssentialMatrixException, NotAvailableException, WrongSizeException{
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
            
            MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, 
                    betaEuler, gammaEuler);
            Point3D center = new InhomogeneousPoint3D(cameraSeparation, 
                    cameraSeparation, cameraSeparation);
            
            EssentialMatrix essentialMatrix = new EssentialMatrix();
            
            //set from rotation and camera center
            essentialMatrix.setFromRotationAndCameraCenter(rotation, center);
            
            //compute possible rotations and translations
            essentialMatrix.computePossibleRotationAndTranslations();
            
            //check correctness
            Point2D firstEstimatedTranslation = 
                    essentialMatrix.getFirstPossibleTranslation();
            Point2D secondEstimatedTranslation = 
                    essentialMatrix.getSecondPossibleTranslation();
            MatrixRotation3D firstEstimatedCameraRotation = 
                    (MatrixRotation3D)essentialMatrix.getFirstPossibleRotation();
            MatrixRotation3D secondEstimatedCameraRotation = 
                    (MatrixRotation3D)essentialMatrix.getSecondPossibleRotation();            
            
            boolean valid = (Math.abs(Math.abs(rotation.getAlphaEulerAngle()) - 
                    Math.abs(firstEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getAlphaEulerAngle()) - 
                    Math.abs(secondEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getBetaEulerAngle()) - 
                    Math.abs(firstEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getBetaEulerAngle()) - 
                    Math.abs(secondEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getGammaEulerAngle()) - 
                    Math.abs(firstEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getGammaEulerAngle()) - 
                    Math.abs(secondEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);        

            if (valid) {
                numValid++;
            }
            
            
            //translation term is equal to t=-R*C, hence we can obtain camera
            //centers as: C = -inv(R)*t = -R'*t
            Matrix rotationMatrix1 = 
                    firstEstimatedCameraRotation.getInternalMatrix();
            Matrix rotationMatrix2 =
                    secondEstimatedCameraRotation.getInternalMatrix();
            Matrix transRotationMatrix1 = rotationMatrix1.
                    transposeAndReturnNew();
            Matrix transRotationMatrix2 = rotationMatrix2.
                    transposeAndReturnNew();
            Matrix translationMatrix1 = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix1.setElementAt(0, 0, 
                    firstEstimatedTranslation.getHomX());
            translationMatrix1.setElementAt(1, 0, 
                    firstEstimatedTranslation.getHomY());
            translationMatrix1.setElementAt(2, 0, 
                    firstEstimatedTranslation.getHomW());
            Matrix translationMatrix2 = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix2.setElementAt(0, 0, 
                    secondEstimatedTranslation.getHomX());
            translationMatrix2.setElementAt(1, 0, 
                    secondEstimatedTranslation.getHomY());
            translationMatrix2.setElementAt(2, 0, 
                    secondEstimatedTranslation.getHomW());
            
            Matrix centerMatrix1 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            Matrix centerMatrix2 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);
            Matrix centerMatrix3 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            Matrix centerMatrix4 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);
            
            double scaleX, scaleY, scaleZ;
            scaleX = centerMatrix1.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix1.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix1.getElementAt(2, 0) / center.getInhomZ();
            
            boolean valid1 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix2.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix2.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix2.getElementAt(2, 0) / center.getInhomZ();
            
            boolean valid2 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix3.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix3.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix3.getElementAt(2, 0) / center.getInhomZ();
            
            boolean valid3 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix4.getElementAt(0, 0) / center.getInhomX();
            scaleY = centerMatrix4.getElementAt(1, 0) / center.getInhomY();
            scaleZ = centerMatrix4.getElementAt(2, 0) / center.getInhomZ();
            
            boolean valid4 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);
            
            assertTrue(valid1 || valid2 || valid3 || valid4);
            
            //Force IllegalArgumentException
            try {
                essentialMatrix.setFromRotationAndCameraCenter(rotation, center, 
                        -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
        }
        assertTrue(numValid > TIMES / 4);        
    }
    
    @Test
    public void testComputePossibleRotationsAndTranslations() 
            throws InvalidRotationAndTranslationException, 
            InvalidEssentialMatrixException, NotAvailableException, 
            InvalidPairOfCamerasException, InvalidPairOfIntrinsicParametersException, WrongSizeException{
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            //Create rotation and translation
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES);
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES);
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES);
            MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, 
                    betaEuler, gammaEuler);
            double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);
            Point2D translation = new HomogeneousPoint2D(cameraSeparation, 
                    cameraSeparation, cameraSeparation);

            EssentialMatrix essentialMatrix = new EssentialMatrix(rotation, 
                    translation);

            //Test NotAvailableExceptions
            assertFalse(essentialMatrix.
                    arePossibleRotationsAndTranslationsAvailable());
            try {
                essentialMatrix.getFirstPossibleRotation();
                fail("NotAvailableException expected but not thrown");
            } catch (NotAvailableException ignore) { }
            try {
                essentialMatrix.getSecondPossibleRotation();
                fail("NotAvailableException expected but not thrown");
            } catch (NotAvailableException ignore) { }
            try {
                essentialMatrix.getFirstPossibleTranslation();
                fail("NotAvailableException expected but not thrown");
            } catch (NotAvailableException ignore) { }
            try {
                essentialMatrix.getSecondPossibleTranslation();
                fail("NotAvailableException expected but not thrown");
            } catch (NotAvailableException ignore) { }

            //compute possible rotations and translations
            essentialMatrix.computePossibleRotationAndTranslations();

            //check correctness
            Point2D firstEstimatedTranslation = 
                    essentialMatrix.getFirstPossibleTranslation();
            Point2D secondEstimatedTranslation =
                    essentialMatrix.getSecondPossibleTranslation();
            MatrixRotation3D firstEstimatedCameraRotation = 
                    (MatrixRotation3D)essentialMatrix.
                    getFirstPossibleRotation();
            MatrixRotation3D secondEstimatedCameraRotation =
                    (MatrixRotation3D)essentialMatrix.
                    getSecondPossibleRotation();

            assertTrue(translation.equals(firstEstimatedTranslation) &&
                    translation.equals(secondEstimatedTranslation));

            boolean valid = (Math.abs(Math.abs(rotation.getAlphaEulerAngle()) - 
                    Math.abs(firstEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getAlphaEulerAngle()) - 
                    Math.abs(secondEstimatedCameraRotation.getAlphaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getBetaEulerAngle()) - 
                    Math.abs(firstEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getBetaEulerAngle()) - 
                    Math.abs(secondEstimatedCameraRotation.getBetaEulerAngle())) <= ABSOLUTE_ERROR);
            valid &= (Math.abs(Math.abs(rotation.getGammaEulerAngle()) - 
                    Math.abs(firstEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR ||
                    Math.abs(Math.abs(rotation.getGammaEulerAngle()) - 
                    Math.abs(secondEstimatedCameraRotation.getGammaEulerAngle())) <= ABSOLUTE_ERROR);        

            if (valid) {
                numValid++;
            }
            
            //testing again from a pair of cameras
            double alphaEuler1 = 0.0;
            double betaEuler1 = 0.0;
            double gammaEuler1 = 0.0;
            double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            
            double horizontalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength1 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double horizontalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength2 = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            
            double skewness1 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, 
                    MAX_SKEWNESS);
            
            double horizontalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint1 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double horizontalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint2 = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            
            PinholeCameraIntrinsicParameters intrinsic1 = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength1,
                    verticalFocalLength1, horizontalPrincipalPoint1, 
                    verticalPrincipalPoint1, skewness1);
            PinholeCameraIntrinsicParameters intrinsic2 = 
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength2,
                    verticalFocalLength2, horizontalPrincipalPoint2, 
                    verticalPrincipalPoint2, skewness2);            
            
            Point3D cameraCenter1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            Point3D cameraCenter2 = new InhomogeneousPoint3D(cameraSeparation, 
                    cameraSeparation, cameraSeparation);
            
            MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1, 
                    betaEuler1, gammaEuler1);
            MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);
            
            PinholeCamera camera1 = new PinholeCamera(intrinsic1, rotation1, 
                    cameraCenter1);
            PinholeCamera camera2 = new PinholeCamera(intrinsic2, rotation2, 
                    cameraCenter2);
            
            //compute their respective fundamental matrix
            FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1, 
                    camera2);
            
            //obtain essential matrix from fundamental matrix and intrinsic 
            //parameters of both cameras
            essentialMatrix = new EssentialMatrix(fundamentalMatrix, intrinsic1, 
                    intrinsic2);
            
            //compute rotation and translation
            essentialMatrix.computePossibleRotationAndTranslations();
            
            //check that rotation is correct
            firstEstimatedTranslation = 
                    essentialMatrix.getFirstPossibleTranslation();
            secondEstimatedTranslation = 
                    essentialMatrix.getSecondPossibleTranslation();
            firstEstimatedCameraRotation = (MatrixRotation3D)essentialMatrix.
                    getFirstPossibleRotation();
            secondEstimatedCameraRotation = (MatrixRotation3D)essentialMatrix.
                    getSecondPossibleRotation();
            
            //compute 4 possible camera centers for second camera and check that 
            //the translation term is equal to -R*C where R is rotation and C is
            //camera center using inhomogeneous coordinates
            Matrix rotationMatrix1 = firstEstimatedCameraRotation.
                    getInternalMatrix();
            Matrix rotationMatrix2 = secondEstimatedCameraRotation.
                    getInternalMatrix();
            Matrix transRotationMatrix1 = rotationMatrix1.
                    transposeAndReturnNew();
            Matrix transRotationMatrix2 = rotationMatrix2.
                    transposeAndReturnNew();
            Matrix translationMatrix1 = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix1.setElementAt(0, 0, 
                    firstEstimatedTranslation.getHomX());
            translationMatrix1.setElementAt(1, 0, 
                    firstEstimatedTranslation.getHomY());
            translationMatrix1.setElementAt(2, 0, 
                    firstEstimatedTranslation.getHomW());
            Matrix translationMatrix2 = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            translationMatrix2.setElementAt(0, 0, 
                    secondEstimatedTranslation.getHomX());
            translationMatrix2.setElementAt(1, 0, 
                    secondEstimatedTranslation.getHomY());
            translationMatrix2.setElementAt(2, 0, 
                    secondEstimatedTranslation.getHomW());
            
            Matrix centerMatrix1 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            Matrix centerMatrix2 = transRotationMatrix1.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);
            Matrix centerMatrix3 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix1).multiplyByScalarAndReturnNew(-1.0);
            Matrix centerMatrix4 = transRotationMatrix2.multiplyAndReturnNew(
                    translationMatrix2).multiplyByScalarAndReturnNew(-1.0);
            
            double scaleX, scaleY, scaleZ;
            scaleX = centerMatrix1.getElementAt(0, 0) / cameraCenter2.getInhomX();
            scaleY = centerMatrix1.getElementAt(1, 0) / cameraCenter2.getInhomY();
            scaleZ = centerMatrix1.getElementAt(2, 0) / cameraCenter2.getInhomZ();
            
            boolean valid1 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);
            
            scaleX = centerMatrix2.getElementAt(0, 0) / cameraCenter2.getInhomX();
            scaleY = centerMatrix2.getElementAt(1, 0) / cameraCenter2.getInhomY();
            scaleZ = centerMatrix2.getElementAt(2, 0) / cameraCenter2.getInhomZ();
            
            boolean valid2 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);
            
            scaleX = centerMatrix3.getElementAt(0, 0) / cameraCenter2.getInhomX();
            scaleY = centerMatrix3.getElementAt(1, 0) / cameraCenter2.getInhomY();
            scaleZ = centerMatrix3.getElementAt(2, 0) / cameraCenter2.getInhomZ();

            boolean valid3 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);

            scaleX = centerMatrix4.getElementAt(0, 0) / cameraCenter2.getInhomX();
            scaleY = centerMatrix4.getElementAt(1, 0) / cameraCenter2.getInhomY();
            scaleZ = centerMatrix4.getElementAt(2, 0) / cameraCenter2.getInhomZ();

            boolean valid4 = (Math.abs(scaleX - scaleY) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleY - scaleZ) < ABSOLUTE_ERROR) &&
                    (Math.abs(scaleZ - scaleX) < ABSOLUTE_ERROR);
            
            assertTrue(valid1 || valid2 || valid3 || valid4);
        }
        assertTrue(numValid > TIMES / 4);        
    }
    
    @Test
    public void testIsValidInternalMatrix() throws WrongSizeException, 
            NotReadyException, LockedException, DecomposerException, 
            com.irurueta.algebra.NotAvailableException {
        
        for (int t = 0; t < TIMES; t++) {
            //testing invalid essential matrix with rank differento of 2
            Matrix internalMatrix = Matrix.createWithUniformRandomValues(
                    ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    internalMatrix);
            decomposer.decompose();

            Matrix U = decomposer.getU();
            Matrix W = decomposer.getW();
            Matrix V = decomposer.getV();
            Matrix transV = V.transposeAndReturnNew();

            //set all singular values to non-zero to enforce rank 3
            W.setElementAt(0, 0, 1.0);
            W.setElementAt(1, 1, 1.0);
            W.setElementAt(2, 2, 3.0);

            internalMatrix = U.multiplyAndReturnNew(W.multiplyAndReturnNew(
                    transV));

            assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix));

            //testing invalid essential matrix with more than 3 columns and rows
            internalMatrix = Matrix.createWithUniformRandomValues(
                    ESSENTIAL_MATRIX_ROWS + 1, ESSENTIAL_MATRIX_COLS + 1, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix));

            //testing invalid essential matrix with different singular values
            internalMatrix = Matrix.createWithUniformRandomValues(
                    ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer = new SingularValueDecomposer(internalMatrix);
            decomposer.decompose();

            U = decomposer.getU();
            W = decomposer.getW();
            V = decomposer.getV();
            transV = V.transposeAndReturnNew();

            //set last singular value to zero to enforce rank 2, but set 
            //different non-zero singular values
            W.setElementAt(0, 0, 1.0);
            W.setElementAt(1, 1, 1.5);
            W.setElementAt(2, 2, 0.0);

            internalMatrix = U.multiplyAndReturnNew(W.multiplyAndReturnNew(
                    transV));

            assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix));
            assertFalse(EssentialMatrix.isValidInternalMatrix(internalMatrix, 
                    0.2));

            //setting a large enough threshold makes it valid
            assertTrue(EssentialMatrix.isValidInternalMatrix(internalMatrix, 
                    0.5 + ABSOLUTE_ERROR));

            //Testing a valid essential matrix
            internalMatrix = Matrix.createWithUniformRandomValues(
                    ESSENTIAL_MATRIX_ROWS, ESSENTIAL_MATRIX_COLS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer = new SingularValueDecomposer(internalMatrix);
            decomposer.decompose();

            U = decomposer.getU();
            W = decomposer.getW();
            V = decomposer.getV();
            transV = V.transposeAndReturnNew();

            //set last singular value to zero to enforce rank 2
            W.setElementAt(0, 0, 1.0);
            W.setElementAt(1, 1, 1.0);
            W.setElementAt(2, 2, 0.0);

            internalMatrix = U.multiplyAndReturnNew(W.multiplyAndReturnNew(
                    transV));

            assertTrue(EssentialMatrix.isValidInternalMatrix(internalMatrix));

            //Force IllegalArgumentException
            try {
                EssentialMatrix.isValidInternalMatrix(internalMatrix, -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ignore) { }
        }
    }
}
