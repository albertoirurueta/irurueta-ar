/*
 * Copyright (C) 2016 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.ar.epipolar.FundamentalMatrix;
import org.junit.*;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

import static org.junit.Assert.*;

public class EstimatedFundamentalMatrixTest {
    
    public EstimatedFundamentalMatrixTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testConstructor() {
        EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();
        
        //check default values
        assertNull(efm.getId());
        assertNull(efm.getFundamentalMatrix());
        assertEquals(efm.getQualityScore(), 
                EstimatedFundamentalMatrix.DEFAULT_QUALITY_SCORE, 0.0);
        assertNull(efm.getCovariance());
        assertEquals(efm.getViewId1(), 0);
        assertEquals(efm.getViewId2(), 0);
        assertNull(efm.getInliers());
        assertNull(efm.getLeftSamples());
        assertNull(efm.getRightSamples());
    }
    
    @Test
    public void testGetSetId() {
        EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();
        
        //check default value
        assertNull(efm.getId());
        
        //set new value
        efm.setId("id");
        
        //check correctness
        assertEquals(efm.getId(), "id");
    }
    
    @Test
    public void testGetSetFundamentalMatrix() {
        EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();
        
        //check default value
        assertNull(efm.getFundamentalMatrix());
        
        //set new value
        FundamentalMatrix f = new FundamentalMatrix();
        efm.setFundamentalMatrix(f);
        
        //check correctness
        assertSame(efm.getFundamentalMatrix(), f);
    }
    
    @Test
    public void testGetSetQualityScore() {
        EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();
        
        //check default value
        assertEquals(efm.getQualityScore(), 
                EstimatedFundamentalMatrix.DEFAULT_QUALITY_SCORE, 0.0);
        
        //set new value
        efm.setQualityScore(5.0);
        
        //check correctness
        assertEquals(efm.getQualityScore(), 5.0, 0.0);
    }
    
    @Test
    public void testGetSetCovariance() throws WrongSizeException {
        EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();
        
        //check default vlaue
        assertNull(efm.getCovariance());
        
        //set new value
        Matrix cov = new Matrix(9,9);
        efm.setCovariance(cov);
        
        //check correctness
        assertSame(efm.getCovariance(), cov);
    }
    
    @Test
    public void testGetSetViewId1() {
        EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();
        
        //check default value
        assertEquals(efm.getViewId1(), 0);
        
        //set new value
        efm.setViewId1(5);
        
        //check correctness
        assertEquals(efm.getViewId1(), 5);
    }
    
    @Test
    public void testGetSetViewId2() {
        EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();
        
        //check default value
        assertEquals(efm.getViewId2(), 0);
        
        //set new value
        efm.setViewId2(10);
        
        //check correctness
        assertEquals(efm.getViewId2(), 10);
    }
    
    @Test
    public void testGetSetInliers() {
        EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();
        
        //check default value
        assertNull(efm.getInliers());
        
        //set new value
        BitSet inliers = new BitSet();
        efm.setInliers(inliers);
        
        //check correctness
        assertSame(efm.getInliers(), inliers);
    }
    
    @Test
    public void testGetSetLeftSamples() {
        EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();
        
        //check default value
        assertNull(efm.getLeftSamples());
        
        //set new value
        List<Sample2D> leftSamples = new ArrayList<>();
        efm.setLeftSamples(leftSamples);
        
        //check correctness
        assertSame(efm.getLeftSamples(), leftSamples);
    }
    
    @Test
    public void testGetSetRightSamples() {
        EstimatedFundamentalMatrix efm = new EstimatedFundamentalMatrix();
        
        //check default value
        assertNull(efm.getRightSamples());
        
        //set new value
        List<Sample2D> rightSamples = new ArrayList<>();
        efm.setRightSamples(rightSamples);
        
        //check correctness
        assertSame(efm.getRightSamples(), rightSamples);
    }
}
