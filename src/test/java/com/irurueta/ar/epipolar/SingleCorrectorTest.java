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

import com.irurueta.geometry.Point2D;
import org.junit.*;

import static org.junit.Assert.*;

public class SingleCorrectorTest {
    
    public SingleCorrectorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testCreate() {
        //test create with method
        
        //Gold Standard
        SingleCorrector corrector = SingleCorrector.create(
                CorrectorType.GOLD_STANDARD);
        
        //check correctness
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);
        assertTrue(corrector instanceof GoldStandardSingleCorrector);
        
        //Sampson
        corrector = SingleCorrector.create(CorrectorType.SAMPSON_CORRECTOR);
        
        //check correctness
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        assertTrue(corrector instanceof SampsonSingleCorrector);
        
        //test create with fundamental matrix and type
        FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        
        //Gold standard
        corrector = SingleCorrector.create(fundamentalMatrix, 
                CorrectorType.GOLD_STANDARD);
        
        //check correctness
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);
        assertTrue(corrector instanceof GoldStandardSingleCorrector);
        
        //Sampson
        corrector = SingleCorrector.create(fundamentalMatrix, 
                CorrectorType.SAMPSON_CORRECTOR);
        
        //check correctness
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        assertTrue(corrector instanceof SampsonSingleCorrector);
        
        //test create with left and right points
        Point2D leftPoint = Point2D.create();
        Point2D rightPoint = Point2D.create();
        
        //Gold standard
        corrector = SingleCorrector.create(leftPoint, rightPoint, 
                CorrectorType.GOLD_STANDARD);
        
        //check correctness
        assertSame(corrector.getLeftPoint(), leftPoint);
        assertSame(corrector.getRightPoint(), rightPoint);
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);
        assertTrue(corrector instanceof GoldStandardSingleCorrector);
        
        //Sampson
        corrector = SingleCorrector.create(leftPoint, rightPoint, 
                CorrectorType.SAMPSON_CORRECTOR);
        
        //check correctness
        assertSame(corrector.getLeftPoint(), leftPoint);
        assertSame(corrector.getRightPoint(), rightPoint);
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        assertTrue(corrector instanceof SampsonSingleCorrector);
        
        //test create with left and right points and fundamental matrix
        
        //Gold standard
        corrector = SingleCorrector.create(leftPoint, rightPoint, 
                fundamentalMatrix, CorrectorType.GOLD_STANDARD);
        
        //check correctness
        assertSame(corrector.getLeftPoint(), leftPoint);
        assertSame(corrector.getRightPoint(), rightPoint);
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertFalse(corrector.isReady()); //fundamental matrix is not defined
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), CorrectorType.GOLD_STANDARD);
        assertTrue(corrector instanceof GoldStandardSingleCorrector);
        
        //Sampson
        corrector = SingleCorrector.create(leftPoint, rightPoint, 
                fundamentalMatrix, CorrectorType.SAMPSON_CORRECTOR);
        
        //check correctness
        assertSame(corrector.getLeftPoint(), leftPoint);
        assertSame(corrector.getRightPoint(), rightPoint);
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertFalse(corrector.isReady()); //fundamental matrix is not defined
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), CorrectorType.SAMPSON_CORRECTOR);
        assertTrue(corrector instanceof SampsonSingleCorrector);
        
        //test create without arguments
        corrector = SingleCorrector.create();
        
        //check corrector
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), SingleCorrector.DEFAULT_TYPE);
        
        //test create with fundamental matrix
        corrector = SingleCorrector.create(fundamentalMatrix);
        
        //check correctness
        assertNull(corrector.getLeftPoint());
        assertNull(corrector.getRightPoint());
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), SingleCorrector.DEFAULT_TYPE);
        
        //test create with left and right points
        corrector = SingleCorrector.create(leftPoint, rightPoint);
        
        //check correctness
        assertSame(corrector.getLeftPoint(), leftPoint);
        assertSame(corrector.getRightPoint(), rightPoint);
        assertNull(corrector.getFundamentalMatrix());
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), SingleCorrector.DEFAULT_TYPE);
        
        //test create with left and right points and fundamental matrix
        corrector = SingleCorrector.create(leftPoint, rightPoint, 
                fundamentalMatrix);
        
        //check correctness
        assertSame(corrector.getLeftPoint(), leftPoint);
        assertSame(corrector.getRightPoint(), rightPoint);
        assertSame(corrector.getFundamentalMatrix(), fundamentalMatrix);
        assertFalse(corrector.isReady());
        assertNull(corrector.getLeftCorrectedPoint());
        assertNull(corrector.getRightCorrectedPoint());
        assertEquals(corrector.getType(), SingleCorrector.DEFAULT_TYPE);
    }
}
