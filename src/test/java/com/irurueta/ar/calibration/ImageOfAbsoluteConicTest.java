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
import com.irurueta.geometry.DualConicNotAvailableException;
import com.irurueta.geometry.InvalidPinholeCameraIntrinsicParametersException;
import com.irurueta.geometry.NonSymmetricMatrixException;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class ImageOfAbsoluteConicTest {
    
    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;
    
    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;
    
    private static final double MIN_SKEWNESS = -1.0;
    private static final double MAX_SKEWNESS = 1.0;
    
    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-8;
    
    private static final int IAC_ROWS = 3;
    private static final int IAC_COLS = 3;
    
    public ImageOfAbsoluteConicTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructorAndGetIntrinsicParameters() 
            throws InvalidPinholeCameraIntrinsicParametersException, 
            NonSymmetricMatrixException, WrongSizeException {
        //create intrinsic parameters
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        
        double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        
        double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
                
        PinholeCameraIntrinsicParameters intrinsic = 
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength, horizontalPrincipalPoint, 
                verticalPrincipalPoint, skewness);
        
        //test constructor from intrinsic parameters
        ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);
        
        PinholeCameraIntrinsicParameters intrinsic2 =
                iac.getIntrinsicParameters();
        
        assertEquals(intrinsic.getHorizontalFocalLength(), 
                intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(),
                intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(),
                intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(),
                ABSOLUTE_ERROR);

        PinholeCameraIntrinsicParameters intrinsic3 =
                iac.getIntrinsicParametersCholesky();
        
        assertEquals(intrinsic.getHorizontalFocalLength(), 
                intrinsic3.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(),
                intrinsic3.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                intrinsic3.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(),
                intrinsic3.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(),
                ABSOLUTE_ERROR);
        
        //test constructor from parameters
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        iac = new ImageOfAbsoluteConic(a, b, c, d, e, f);
        
        //check correctness
        assertEquals(iac.getA(), a, 0.0);
        assertEquals(iac.getB(), b, 0.0);
        assertEquals(iac.getC(), c, 0.0);
        assertEquals(iac.getD(), d, 0.0);
        assertEquals(iac.getE(), e, 0.0);
        assertEquals(iac.getF(), f, 0.0);
        
        //test constructor from matrix
        Matrix m = new Matrix(IAC_ROWS, IAC_COLS);
        //get random values
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e);
        m.setElementAt(2, 0, d);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);
        
        iac = new ImageOfAbsoluteConic(m);
        
        assertEquals(iac.getA(), m.getElementAt(0, 0), 0.0);
        assertEquals(iac.getB(), m.getElementAt(0, 1), 0.0);
        assertEquals(iac.getC(), m.getElementAt(1, 1), 0.0);
        assertEquals(iac.getD(), m.getElementAt(0, 2), 0.0);
        assertEquals(iac.getE(), m.getElementAt(1, 2), 0.0);
        assertEquals(iac.getF(), m.getElementAt(2, 2), 0.0);
        
        //Constructor using matrix with wrong size exception
        m = new Matrix(IAC_ROWS, IAC_COLS + 1);
        iac = null;
        try {
            iac = new ImageOfAbsoluteConic(m);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(iac);
        
        //Constructor using non-symmetric matrix
        m = new Matrix(IAC_ROWS, IAC_COLS);
        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b + 1.0);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e + 1.0);
        m.setElementAt(2, 0, d + 1.0);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);
        
        iac = null;
        try {
            iac = new ImageOfAbsoluteConic(m);
            fail("NonSymmetricMatrixException expected but not thrown");
        } catch (NonSymmetricMatrixException ignore) { }
        assertNull(iac);                
    }
    
    @Test
    public void testGetDualConic() 
            throws InvalidPinholeCameraIntrinsicParametersException, 
            DualConicNotAvailableException {
        //create intrinsic parameters
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                MAX_FOCAL_LENGTH);
        
        double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        
        double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
                
        PinholeCameraIntrinsicParameters intrinsic = 
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength, horizontalPrincipalPoint, 
                verticalPrincipalPoint, skewness);
        
        ImageOfAbsoluteConic iac = new ImageOfAbsoluteConic(intrinsic);
        
        DualImageOfAbsoluteConic diac =
                (DualImageOfAbsoluteConic)iac.getDualConic();
        
        PinholeCameraIntrinsicParameters intrinsic2 = 
                diac.getIntrinsicParameters();
        
        assertEquals(intrinsic.getHorizontalFocalLength(), 
                intrinsic2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalFocalLength(),
                intrinsic2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getHorizontalPrincipalPoint(),
                intrinsic2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getVerticalPrincipalPoint(),
                intrinsic2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(intrinsic.getSkewness(), intrinsic2.getSkewness(),
                ABSOLUTE_ERROR);        
    }
}
