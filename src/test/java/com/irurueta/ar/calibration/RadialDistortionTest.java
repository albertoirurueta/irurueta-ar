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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.NotSupportedException;
import com.irurueta.geometry.Point2D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class RadialDistortionTest {
    
    private static final double MIN_POINT_VALUE = -1.0;
    private static final double MAX_POINT_VALUE = 1.0;
    
    private static final double MIN_PARAM_VALUE = -1e-4;
    private static final double MAX_PARAM_VALUE = 1e-4;
    
    private static final double ERROR = 1e-6;
    
    private static final int NUM_POINTS = 10;
    
    private static final int TIMES = 100;
        
    public RadialDistortionTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() throws NotSupportedException,
            DistortionException {
        //default constructor
        RadialDistortion distortion = new RadialDistortion();
        
        //check correctness
        assertNull(distortion.getCenter());
        assertEquals(distortion.getHorizontalFocalLength(), 
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getVerticalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getSkew(), RadialDistortion.DEFAULT_SKEW, 0.0);
        assertEquals(distortion.getIntrinsic().getHorizontalPrincipalPoint(), 
                0.0, 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(distortion.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getIntrinsic().getSkewness(),
                RadialDistortion.DEFAULT_SKEW, 0.0);
        assertEquals(distortion.getK1(), 0.0, 0.0);
        assertEquals(distortion.getK2(), 0.0, 0.0);
        assertNull(distortion.getKParams());
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());
        assertEquals(distortion.getKind(), 
                DistortionKind.BROWN_RADIAL_DISTORTION);
        
        //constructor with parameters
        distortion = new RadialDistortion(1.0, 2.0);
        
        //check correctness
        assertNull(distortion.getCenter());
        assertEquals(distortion.getHorizontalFocalLength(), 
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getVerticalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getSkew(), RadialDistortion.DEFAULT_SKEW, 0.0);        
        assertEquals(distortion.getIntrinsic().getHorizontalPrincipalPoint(), 
                0.0, 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(distortion.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getIntrinsic().getSkewness(),
                RadialDistortion.DEFAULT_SKEW, 0.0);        
        assertEquals(distortion.getK1(), 1.0, 0.0);
        assertEquals(distortion.getK2(), 2.0, 0.0);
        assertEquals(distortion.getKParams()[0], 1.0, 0.0);
        assertEquals(distortion.getKParams()[1], 2.0, 0.0);
        assertTrue(distortion.canDistort());        
        assertTrue(distortion.canUndistort());
        assertEquals(distortion.getKind(), 
                DistortionKind.BROWN_RADIAL_DISTORTION);     
        
        //constructor with parameters array
        distortion = new RadialDistortion(new double[]{-1.0, -2.0});

        //check correctness
        assertNull(distortion.getCenter());
        assertEquals(distortion.getHorizontalFocalLength(), 
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getVerticalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getSkew(), RadialDistortion.DEFAULT_SKEW, 0.0);        
        assertEquals(distortion.getIntrinsic().getHorizontalPrincipalPoint(), 
                0.0, 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalPrincipalPoint(),
                0.0, 0.0);
        assertEquals(distortion.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getIntrinsic().getSkewness(),
                RadialDistortion.DEFAULT_SKEW, 0.0);        
        assertEquals(distortion.getK1(), -1.0, 0.0);
        assertEquals(distortion.getK2(), -2.0, 0.0);
        assertEquals(distortion.getKParams()[0], -1.0, 0.0);
        assertEquals(distortion.getKParams()[1], -2.0, 0.0);
        assertTrue(distortion.canDistort());        
        assertTrue(distortion.canUndistort());
        assertEquals(distortion.getKind(), 
                DistortionKind.BROWN_RADIAL_DISTORTION);     
        
        //Force IllegalArgumentException
        distortion = null;
        try {
            distortion = new RadialDistortion(null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(distortion);
        
        //constructor with parameters and center
        Point2D center = new InhomogeneousPoint2D(4.0, 5.0);
        distortion = new RadialDistortion(2.0, 3.0, center);
        
        //check correctness
        assertSame(distortion.getCenter(), center);
        assertEquals(distortion.getHorizontalFocalLength(), 
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getVerticalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getSkew(), RadialDistortion.DEFAULT_SKEW, 0.0);        
        assertEquals(distortion.getIntrinsic().getHorizontalPrincipalPoint(), 
                center.getInhomX(), 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalPrincipalPoint(),
                center.getInhomY(), 0.0);
        assertEquals(distortion.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getIntrinsic().getSkewness(),
                RadialDistortion.DEFAULT_SKEW, 0.0);        
        assertEquals(distortion.getK1(), 2.0, 0.0);
        assertEquals(distortion.getK2(), 3.0, 0.0);
        assertEquals(distortion.getKParams()[0], 2.0, 0.0);
        assertEquals(distortion.getKParams()[1], 3.0, 0.0);        
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());        
        assertEquals(distortion.getKind(), 
                DistortionKind.BROWN_RADIAL_DISTORTION); 
                
        //constructor with parameters array and center
        distortion = new RadialDistortion(new double[]{-2.0, -3.0}, center);
        
        //check correctness
        assertSame(distortion.getCenter(), center);
        assertEquals(distortion.getHorizontalFocalLength(), 
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getVerticalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getSkew(), RadialDistortion.DEFAULT_SKEW, 0.0);        
        assertEquals(distortion.getIntrinsic().getHorizontalPrincipalPoint(), 
                center.getInhomX(), 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalPrincipalPoint(),
                center.getInhomY(), 0.0);
        assertEquals(distortion.getIntrinsic().getHorizontalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getIntrinsic().getSkewness(),
                RadialDistortion.DEFAULT_SKEW, 0.0);                
        assertEquals(distortion.getK1(), -2.0, 0.0);
        assertEquals(distortion.getK2(), -3.0, 0.0);
        assertEquals(distortion.getKParams()[0], -2.0, 0.0);
        assertEquals(distortion.getKParams()[1], -3.0, 0.0);        
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());        
        assertEquals(distortion.getKind(), 
                DistortionKind.BROWN_RADIAL_DISTORTION);  
        
        //Force IllegalArgumentException
        distortion = null;
        try {
            distortion = new RadialDistortion(null, center);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(distortion);
                
        //constructor with parameters, center and intrinsic parameters
        distortion = new RadialDistortion(1.0, 2.0, center, 3.0, 4.0, 5.0);
        
        //check correctness
        assertSame(distortion.getCenter(), center);
        assertEquals(distortion.getHorizontalFocalLength(), 3.0, 0.0);
        assertEquals(distortion.getVerticalFocalLength(), 4.0, 0.0);
        assertEquals(distortion.getSkew(), 5.0, 0.0);
        assertEquals(distortion.getIntrinsic().getHorizontalPrincipalPoint(), 
                center.getInhomX(), 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalPrincipalPoint(),
                center.getInhomY(), 0.0);
        assertEquals(distortion.getIntrinsic().getHorizontalFocalLength(),
                3.0, 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalFocalLength(),
                4.0, 0.0);
        assertEquals(distortion.getIntrinsic().getSkewness(),
                5.0, 0.0);                
        assertEquals(distortion.getK1(), 1.0, 0.0);
        assertEquals(distortion.getK2(), 2.0, 0.0);
        assertEquals(distortion.getKParams()[0], 1.0, 0.0);
        assertEquals(distortion.getKParams()[1], 2.0, 0.0);        
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());        
        assertEquals(distortion.getKind(), 
                DistortionKind.BROWN_RADIAL_DISTORTION);  
        
        //Force RadialDistortionException
        distortion = null;
        try {
            distortion = new RadialDistortion(1.0, 2.0, center, 
                    0.0, 4.0, 5.0);
            fail("RadialDistortionException expected but not thrown");
        } catch (RadialDistortionException ignore) { }
        assertNull(distortion);        
        
        //constructor with parameters arrays, center and intrinsic parameters
        distortion = new RadialDistortion(new double[]{-1.0, -2.0}, center,
                3.0, 4.0, 5.0);
        
        //check correctness
        assertSame(distortion.getCenter(), center);
        assertEquals(distortion.getHorizontalFocalLength(), 3.0, 0.0);
        assertEquals(distortion.getVerticalFocalLength(), 4.0, 0.0);
        assertEquals(distortion.getSkew(), 5.0, 0.0);
        assertEquals(distortion.getIntrinsic().getHorizontalPrincipalPoint(), 
                center.getInhomX(), 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalPrincipalPoint(),
                center.getInhomY(), 0.0);
        assertEquals(distortion.getIntrinsic().getHorizontalFocalLength(),
                3.0, 0.0);
        assertEquals(distortion.getIntrinsic().getVerticalFocalLength(),
                4.0, 0.0);
        assertEquals(distortion.getIntrinsic().getSkewness(),
                5.0, 0.0);                        
        assertEquals(distortion.getK1(), -1.0, 0.0);
        assertEquals(distortion.getK2(), -2.0, 0.0);
        assertEquals(distortion.getKParams()[0], -1.0, 0.0);
        assertEquals(distortion.getKParams()[1], -2.0, 0.0);        
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());        
        assertEquals(distortion.getKind(), 
                DistortionKind.BROWN_RADIAL_DISTORTION);  
        
        //Force RadialDistortionException
        distortion = null;
        try {
            distortion = new RadialDistortion(new double[]{-1.0, -2.0}, center, 
                    0.0, 4.0, 5.0);
            fail("RadialDistortionException expected but not thrown");
        } catch (RadialDistortionException ignore) { }
        
        //Force IllegalArgumentException
        try {
            distortion = new RadialDistortion(null, center, 3.0,
                    4.0, 5.0);
            fail("RadialDistortionException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(distortion);
        
        //constructor with points
        UniformRandomizer randomizer = new UniformRandomizer(new Random());        
        
        double k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        double k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        
        center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        
        distortion = new RadialDistortion(k1, k2, center);        
        
        Point2D undistortedPoint1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        Point2D undistortedPoint2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        Point2D distortedPoint1 = distortion.distort(undistortedPoint1);
        Point2D distortedPoint2 = distortion.distort(undistortedPoint2);
        
        distortion = new RadialDistortion(distortedPoint1, distortedPoint2,
                undistortedPoint1, undistortedPoint2, center);
        
        //check correctness
        assertEquals(distortion.getK1(), k1, ERROR);
        assertEquals(distortion.getK2(), k2, ERROR);
        assertSame(distortion.getCenter(), center);
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());        
    }
    
    @Test
    public void testSetFromPointsAndCenter() throws NotSupportedException, 
            DistortionException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());        
        
        double k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        double k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        
        RadialDistortion distortion = new RadialDistortion(k1, k2, center);        
        
        Point2D undistortedPoint1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        Point2D undistortedPoint2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        Point2D distortedPoint1 = distortion.distort(undistortedPoint1);
        Point2D distortedPoint2 = distortion.distort(undistortedPoint2);

        //set from points
        distortion = new RadialDistortion();
        distortion.setFromPointsAndCenter(distortedPoint1, distortedPoint2, 
                undistortedPoint1, undistortedPoint2, center);
        
        //check correctness
        assertEquals(distortion.getK1(), k1, ERROR);
        assertEquals(distortion.getK2(), k2, ERROR);
        assertSame(distortion.getCenter(), center);
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());        
        assertEquals(distortion.getKind(), 
                DistortionKind.BROWN_RADIAL_DISTORTION);  
    }
    
    @Test
    public void testGetSetCenter() {
        RadialDistortion distortion = new RadialDistortion();
        
        //check default value
        assertNull(distortion.getCenter());
        
        //set new value
        Point2D center = Point2D.create();
        distortion.setCenter(center);
        
        //check correctness
        assertSame(distortion.getCenter(), center);
    }
    
    @Test
    public void testGetSetHorizontalFocalLength() 
            throws RadialDistortionException {
        RadialDistortion distortion = new RadialDistortion();
        
        //check default value
        assertEquals(distortion.getHorizontalFocalLength(), 
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        
        //set new value
        distortion.setHorizontalFocalLength(2.0);
        
        //check correctness
        assertEquals(distortion.getHorizontalFocalLength(), 2.0, 0.0);
        
        //Force RadialDistortionException
        try {
            distortion.setHorizontalFocalLength(0.0);
            fail("RadialDistortionException expected but not thrown");
        } catch (RadialDistortionException ignore) { }
    }
    
    @Test
    public void testGetSetVerticalFocalLength()
            throws RadialDistortionException {
        RadialDistortion distortion = new RadialDistortion();
        
        //check default value
        assertEquals(distortion.getVerticalFocalLength(),
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        
        //set new value
        distortion.setVerticalFocalLength(2.0);
        
        //check correctness
        assertEquals(distortion.getVerticalFocalLength(), 2.0, 0.0);
        
        //Force RadialDistortionException
        try {
            distortion.setVerticalFocalLength(0.0);
            fail("RadialDistortionException expected but not thrown");
        } catch (RadialDistortionException ignore) { }
    }
    
    @Test
    public void testGetSetSkew() {
        RadialDistortion distortion = new RadialDistortion();
        
        //check default value
        assertEquals(distortion.getSkew(), RadialDistortion.DEFAULT_SKEW, 0.0);
        
        //set new value
        distortion.setSkew(2.0);
        
        //check correctness
        assertEquals(distortion.getSkew(), 2.0, 0.0);
    }
    
    @Test
    public void testSetIntrinsic() throws RadialDistortionException {
        RadialDistortion distortion = new RadialDistortion();
        
        //check default value
        assertNull(distortion.getCenter());
        assertEquals(distortion.getHorizontalFocalLength(), 
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getVerticalFocalLength(), 
                RadialDistortion.DEFAULT_FOCAL_LENGTH, 0.0);
        assertEquals(distortion.getSkew(), RadialDistortion.DEFAULT_SKEW, 0.0);
        
        //set new value
        Point2D center = Point2D.create();
        distortion.setIntrinsic(center, 2.0, 3.0, 4.0);
        
        //check correctness
        assertSame(distortion.getCenter(), center);
        assertEquals(distortion.getHorizontalFocalLength(), 2.0, 0.0);
        assertEquals(distortion.getVerticalFocalLength(), 3.0, 0.0);
        assertEquals(distortion.getSkew(), 4.0, 0.0);
        
        //Force RadialDistortionException
        try {
            distortion.setIntrinsic(center, 0.0, 0.0, 4.0);
            fail("RadialDistortionException expected but not thrown");
        } catch (RadialDistortionException ignore) { }
    }
    
    @Test
    public void testGetSetK1() {
        RadialDistortion distortion = new RadialDistortion();
        
        //check default value
        assertEquals(distortion.getK1(), 0.0, 0.0);
        
        //set new value
        distortion.setK1(1.0);
        
        //check correctness
        assertEquals(distortion.getK1(), 1.0, 0.0);
    }
    
    @Test
    public void testGetSetK2() {
        RadialDistortion distortion = new RadialDistortion();
        
        //check default value
        assertEquals(distortion.getK2(), 0.0, 0.0);
        
        //set new value
        distortion.setK2(2.0);
        
        //check correctness
        assertEquals(distortion.getK2(), 2.0, 0.0);
    }
    
    @Test
    public void testGetSetKParams() {
        RadialDistortion distortion = new RadialDistortion();
        
        //check default value
        assertNull(distortion.getKParams());
        assertEquals(distortion.getK1(), 0.0, 0.0);
        assertEquals(distortion.getK2(), 0.0, 0.0);
        
        //set new values
        double[] kParams = new double[]{1.0, 2.0, 3.0};
        distortion.setKParams(kParams);
        
        //check correctness
        assertSame(distortion.getKParams(), kParams);
        assertEquals(distortion.getK1(), 1.0, 0.0);
        assertEquals(distortion.getK2(), 2.0, 0.0);
    }
    
    @Test
    public void testDistortUndistortPoint() throws NotSupportedException, 
            DistortionException {
        for (int j = 0; j < TIMES; j++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
            double k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);        

            Point2D center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

            Point2D distorted = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

            RadialDistortion distortion = new RadialDistortion(k1, k2, center);

            Point2D undistorted = distortion.undistort(distorted);
            Point2D distorted2 = distortion.distort(undistorted);

            assertEquals(distorted.distanceTo(distorted2), 0.0, ERROR);    
        }
    }
    
    @Test
    public void testDistortUndistortPoints() throws NotSupportedException, 
            DistortionException {
        for (int j = 0; j < TIMES; j++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
            double k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);        

            Point2D center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));


            RadialDistortion distortion = new RadialDistortion(k1, k2, center);

            List<Point2D> distortedPoints = new ArrayList<>();
            List<Point2D> undistortedPoints = new ArrayList<>();
            List<Point2D> distortedPoints2;
            Point2D distortedPoint;
            for (int i = 0; i < NUM_POINTS; i++) {
                distortedPoint = new InhomogeneousPoint2D(
                        randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                        randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

                distortedPoints.add(distortedPoint);
                undistortedPoints.add(distortion.undistort(distortedPoint));
            }     

            distortedPoints2 = distortion.distort(undistortedPoints);

            for(int i = 0; i < NUM_POINTS; i++){
                assertEquals(distortedPoints.get(i).distanceTo(
                        distortedPoints2.get(i)), 0.0, ERROR);
            }
        }
    }

    @Test
    public void testDistortPoint() throws NotSupportedException, 
            DistortionException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        double k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        
        Point2D undistorted = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        
        RadialDistortion distortion = new RadialDistortion(k1, k2, center);
        
        Point2D distorted1 = distortion.distort(undistorted);
        Point2D distorted2 = Point2D.create();
        distortion.distort(undistorted, distorted2);
        
        assertEquals(distorted1, distorted2);
        
        double xu = undistorted.getInhomX();
        double yu = undistorted.getInhomY();
        double xc = center.getInhomX();
        double yc = center.getInhomY();
        double diffX = xu - xc;
        double diffY = yu - yc;
        double r2 = diffX * diffX + diffY * diffY;
        double r4 = r2 * r2;
        double factor = 1.0 + (k1 * r2) + (k2 * r4);
        double xd = xc + diffX * factor;
        double yd = yc + diffY * factor;
        
        assertEquals(distorted1.getInhomX(), xd, 
                ERROR);
        assertEquals(distorted1.getInhomY(), yd, 
                ERROR);            
    }
    
    @Test
    public void testDistortPoints() throws NotSupportedException, 
            DistortionException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        double k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        
        
        RadialDistortion distortion = new RadialDistortion(k1, k2, center);
        
        List<Point2D> undistortedPoints = new ArrayList<>();
        List<Point2D> distortedPoints1 = new ArrayList<>();
        List<Point2D> distortedPoints2;
        Point2D undistortedPoint, distortedPoint1, distortedPoint2;
        for (int i = 0; i < NUM_POINTS; i++) {
            undistortedPoint = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE), 
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));            
            undistortedPoints.add(undistortedPoint);
            distortedPoints1.add(Point2D.create());
        }
        
        //distort
        distortion.distort(undistortedPoints, distortedPoints1);
        distortedPoints2 = distortion.distort(undistortedPoints);
        
        assertEquals(distortedPoints1.size(), NUM_POINTS);
        assertEquals(distortedPoints2.size(), NUM_POINTS);
        for (int i = 0; i < NUM_POINTS; i++) {
            undistortedPoint = undistortedPoints.get(i);
            distortedPoint1 = distortedPoints1.get(i);
            distortedPoint2 = distortedPoints2.get(i);
            
            assertEquals(distortedPoint1, distortedPoint2);
            
            double xu = undistortedPoint.getInhomX();
            double yu = undistortedPoint.getInhomY();
            double xc = center.getInhomX();
            double yc = center.getInhomY();            
            double diffX = xu - xc;
            double diffY = yu - yc;
            double r2 = diffX * diffX + diffY * diffY;
            double r4 = r2 * r2;
            double factor = 1.0 + (k1 * r2) + (k2 * r4);
            double xd = xc + diffX * factor;
            double yd = yc + diffY * factor;
            
            assertEquals(distortedPoint1.getInhomX(), xd, ERROR);
            assertEquals(distortedPoint1.getInhomY(), yd, ERROR);                        
        }
    }
}
