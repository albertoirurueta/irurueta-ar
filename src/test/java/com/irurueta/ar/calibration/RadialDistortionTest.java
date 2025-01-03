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

import com.irurueta.ar.SerializationHelper;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.NotSupportedException;
import com.irurueta.geometry.Point2D;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class RadialDistortionTest {

    private static final double MIN_POINT_VALUE = -1.0;
    private static final double MAX_POINT_VALUE = 1.0;

    private static final double MIN_PARAM_VALUE = -1e-4;
    private static final double MAX_PARAM_VALUE = 1e-4;

    private static final double ERROR = 1e-6;

    private static final int NUM_POINTS = 10;

    private static final int TIMES = 100;

    @Test
    void testConstructor() throws NotSupportedException, DistortionException {
        // default constructor
        var distortion = new RadialDistortion();

        // check correctness
        assertNull(distortion.getCenter());
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_SKEW, distortion.getSkew(), 0.0);
        assertEquals(0.0, distortion.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, distortion.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getIntrinsic().getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getIntrinsic().getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortion.DEFAULT_SKEW, distortion.getIntrinsic().getSkewness(), 0.0);
        assertEquals(0.0, distortion.getK1(), 0.0);
        assertEquals(0.0, distortion.getK2(), 0.0);
        assertNull(distortion.getKParams());
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());
        assertEquals(DistortionKind.BROWN_RADIAL_DISTORTION, distortion.getKind());

        // constructor with parameters
        distortion = new RadialDistortion(1.0, 2.0);

        // check correctness
        assertNull(distortion.getCenter());
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_SKEW, distortion.getSkew(), 0.0);
        assertEquals(0.0, distortion.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, distortion.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getIntrinsic().getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getIntrinsic().getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortion.DEFAULT_SKEW, distortion.getIntrinsic().getSkewness(), 0.0);
        assertEquals(1.0, distortion.getK1(), 0.0);
        assertEquals(2.0, distortion.getK2(), 0.0);
        assertEquals(1.0, distortion.getKParams()[0], 0.0);
        assertEquals(2.0, distortion.getKParams()[1], 0.0);
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());
        assertEquals(DistortionKind.BROWN_RADIAL_DISTORTION, distortion.getKind());

        // constructor with parameters array
        distortion = new RadialDistortion(new double[]{-1.0, -2.0});

        // check correctness
        assertNull(distortion.getCenter());
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_SKEW, distortion.getSkew(), 0.0);
        assertEquals(0.0, distortion.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(0.0, distortion.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getIntrinsic().getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getIntrinsic().getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortion.DEFAULT_SKEW, distortion.getIntrinsic().getSkewness(), 0.0);
        assertEquals(-1.0, distortion.getK1(), 0.0);
        assertEquals(-2.0, distortion.getK2(), 0.0);
        assertEquals(-1.0, distortion.getKParams()[0], 0.0);
        assertEquals(-2.0, distortion.getKParams()[1], 0.0);
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());
        assertEquals(DistortionKind.BROWN_RADIAL_DISTORTION, distortion.getKind());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RadialDistortion(null));

        // constructor with parameters and center
        final var center1 = new InhomogeneousPoint2D(4.0, 5.0);
        distortion = new RadialDistortion(2.0, 3.0, center1);

        // check correctness
        assertSame(center1, distortion.getCenter());
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_SKEW, distortion.getSkew(), 0.0);
        assertEquals(center1.getInhomX(), distortion.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(center1.getInhomY(), distortion.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getIntrinsic().getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getIntrinsic().getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortion.DEFAULT_SKEW, distortion.getIntrinsic().getSkewness(), 0.0);
        assertEquals(2.0, distortion.getK1(), 0.0);
        assertEquals(3.0, distortion.getK2(), 0.0);
        assertEquals(2.0, distortion.getKParams()[0], 0.0);
        assertEquals(3.0, distortion.getKParams()[1], 0.0);
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());
        assertEquals(DistortionKind.BROWN_RADIAL_DISTORTION, distortion.getKind());

        // constructor with parameters array and center
        distortion = new RadialDistortion(new double[]{-2.0, -3.0}, center1);

        // check correctness
        assertSame(center1, distortion.getCenter());
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_SKEW, distortion.getSkew(), 0.0);
        assertEquals(center1.getInhomX(), distortion.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(center1.getInhomY(), distortion.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getIntrinsic().getHorizontalFocalLength(),
                0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getIntrinsic().getVerticalFocalLength(),
                0.0);
        assertEquals(RadialDistortion.DEFAULT_SKEW, distortion.getIntrinsic().getSkewness(), 0.0);
        assertEquals(-2.0, distortion.getK1(), 0.0);
        assertEquals(-3.0, distortion.getK2(), 0.0);
        assertEquals(-2.0, distortion.getKParams()[0], 0.0);
        assertEquals(-3.0, distortion.getKParams()[1], 0.0);
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());
        assertEquals(DistortionKind.BROWN_RADIAL_DISTORTION, distortion.getKind());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RadialDistortion(null, center1));

        // constructor with parameters, center and intrinsic parameters
        distortion = new RadialDistortion(1.0, 2.0, center1, 3.0, 4.0,
                5.0);

        // check correctness
        assertSame(center1, distortion.getCenter());
        assertEquals(3.0, distortion.getHorizontalFocalLength(), 0.0);
        assertEquals(4.0, distortion.getVerticalFocalLength(), 0.0);
        assertEquals(5.0, distortion.getSkew(), 0.0);
        assertEquals(center1.getInhomX(), distortion.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(center1.getInhomY(), distortion.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(3.0, distortion.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(4.0, distortion.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(5.0, distortion.getIntrinsic().getSkewness(), 0.0);
        assertEquals(1.0, distortion.getK1(), 0.0);
        assertEquals(2.0, distortion.getK2(), 0.0);
        assertEquals(1.0, distortion.getKParams()[0], 0.0);
        assertEquals(2.0, distortion.getKParams()[1], 0.0);
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());
        assertEquals(DistortionKind.BROWN_RADIAL_DISTORTION, distortion.getKind());

        // Force RadialDistortionException
        assertThrows(RadialDistortionException.class, () -> new RadialDistortion(1.0, 2.0, center1,
                0.0, 4.0, 5.0));

        // constructor with parameters arrays, center and intrinsic parameters
        distortion = new RadialDistortion(new double[]{-1.0, -2.0}, center1, 3.0,
                4.0, 5.0);

        // check correctness
        assertSame(center1, distortion.getCenter());
        assertEquals(3.0, distortion.getHorizontalFocalLength(), 0.0);
        assertEquals(4.0, distortion.getVerticalFocalLength(), 0.0);
        assertEquals(5.0, distortion.getSkew(), 0.0);
        assertEquals(center1.getInhomX(), distortion.getIntrinsic().getHorizontalPrincipalPoint(), 0.0);
        assertEquals(center1.getInhomY(), distortion.getIntrinsic().getVerticalPrincipalPoint(), 0.0);
        assertEquals(3.0, distortion.getIntrinsic().getHorizontalFocalLength(), 0.0);
        assertEquals(4.0, distortion.getIntrinsic().getVerticalFocalLength(), 0.0);
        assertEquals(5.0, distortion.getIntrinsic().getSkewness(), 0.0);
        assertEquals(-1.0, distortion.getK1(), 0.0);
        assertEquals(-2.0, distortion.getK2(), 0.0);
        assertEquals(-1.0, distortion.getKParams()[0], 0.0);
        assertEquals(-2.0, distortion.getKParams()[1], 0.0);
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());
        assertEquals(DistortionKind.BROWN_RADIAL_DISTORTION, distortion.getKind());

        // Force RadialDistortionException
        assertThrows(RadialDistortionException.class, () -> new RadialDistortion(new double[]{-1.0, -2.0}, center1,
                0.0, 4.0, 5.0));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new RadialDistortion(null, center1,
                3.0, 4.0, 5.0));

        // constructor with points
        final var randomizer = new UniformRandomizer();

        final var k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        final var k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);

        final var center2 = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

        distortion = new RadialDistortion(k1, k2, center2);

        final var undistortedPoint1 = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        final var undistortedPoint2 = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        final var distortedPoint1 = distortion.distort(undistortedPoint1);
        final var distortedPoint2 = distortion.distort(undistortedPoint2);

        distortion = new RadialDistortion(distortedPoint1, distortedPoint2, undistortedPoint1, undistortedPoint2,
                center2);

        // check correctness
        assertEquals(k1, distortion.getK1(), ERROR);
        assertEquals(k2, distortion.getK2(), ERROR);
        assertSame(center2, distortion.getCenter());
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());
    }

    @Test
    void testSetFromPointsAndCenter() throws NotSupportedException, DistortionException {
        final var randomizer = new UniformRandomizer();

        final var k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        final var k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);

        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

        var distortion = new RadialDistortion(k1, k2, center);

        final var undistortedPoint1 = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        final var undistortedPoint2 = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
        final var distortedPoint1 = distortion.distort(undistortedPoint1);
        final var distortedPoint2 = distortion.distort(undistortedPoint2);

        // set from points
        distortion = new RadialDistortion();
        distortion.setFromPointsAndCenter(distortedPoint1, distortedPoint2, undistortedPoint1, undistortedPoint2,
                center);

        // check correctness
        assertEquals(k1, distortion.getK1(), ERROR);
        assertEquals(k2, distortion.getK2(), ERROR);
        assertSame(center, distortion.getCenter());
        assertTrue(distortion.canDistort());
        assertTrue(distortion.canUndistort());
        assertEquals(DistortionKind.BROWN_RADIAL_DISTORTION, distortion.getKind());
    }

    @Test
    void testGetSetCenter() {
        final var distortion = new RadialDistortion();

        // check default value
        assertNull(distortion.getCenter());

        // set new value
        final var center = Point2D.create();
        distortion.setCenter(center);

        // check correctness
        assertSame(center, distortion.getCenter());
    }

    @Test
    void testGetSetHorizontalFocalLength() throws RadialDistortionException {
        final var distortion = new RadialDistortion();

        // check default value
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getHorizontalFocalLength(), 0.0);

        // set new value
        distortion.setHorizontalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, distortion.getHorizontalFocalLength(), 0.0);

        // Force RadialDistortionException
        assertThrows(RadialDistortionException.class, () -> distortion.setHorizontalFocalLength(0.0));
    }

    @Test
    void testGetSetVerticalFocalLength() throws RadialDistortionException {
        final var distortion = new RadialDistortion();

        // check default value
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getVerticalFocalLength(), 0.0);

        // set new value
        distortion.setVerticalFocalLength(2.0);

        // check correctness
        assertEquals(2.0, distortion.getVerticalFocalLength(), 0.0);

        // Force RadialDistortionException
        assertThrows(RadialDistortionException.class, () -> distortion.setVerticalFocalLength(0.0));
    }

    @Test
    void testGetSetSkew() {
        final var distortion = new RadialDistortion();

        // check default value
        assertEquals(RadialDistortion.DEFAULT_SKEW, distortion.getSkew(), 0.0);

        // set new value
        distortion.setSkew(2.0);

        // check correctness
        assertEquals(2.0, distortion.getSkew(), 0.0);
    }

    @Test
    void testSetIntrinsic() throws RadialDistortionException {
        final var distortion = new RadialDistortion();

        // check default value
        assertNull(distortion.getCenter());
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getHorizontalFocalLength(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_FOCAL_LENGTH, distortion.getVerticalFocalLength(), 0.0);
        assertEquals(RadialDistortion.DEFAULT_SKEW, distortion.getSkew(), 0.0);

        // set new value
        final var center = Point2D.create();
        distortion.setIntrinsic(center, 2.0, 3.0, 4.0);

        // check correctness
        assertSame(center, distortion.getCenter());
        assertEquals(2.0, distortion.getHorizontalFocalLength(), 0.0);
        assertEquals(3.0, distortion.getVerticalFocalLength(), 0.0);
        assertEquals(4.0, distortion.getSkew(), 0.0);

        // Force RadialDistortionException
        assertThrows(RadialDistortionException.class,
                () -> distortion.setIntrinsic(center, 0.0, 0.0, 4.0));
    }

    @Test
    void testGetSetK1() {
        final var distortion = new RadialDistortion();

        // check default value
        assertEquals(0.0, distortion.getK1(), 0.0);

        // set new value
        distortion.setK1(1.0);

        // check correctness
        assertEquals(1.0, distortion.getK1(), 0.0);
    }

    @Test
    void testGetSetK2() {
        final var distortion = new RadialDistortion();

        // check default value
        assertEquals(0.0, distortion.getK2(), 0.0);

        // set new value
        distortion.setK2(2.0);

        // check correctness
        assertEquals(2.0, distortion.getK2(), 0.0);
    }

    @Test
    void testGetSetKParams() {
        final var distortion = new RadialDistortion();

        // check default value
        assertNull(distortion.getKParams());
        assertEquals(0.0, distortion.getK1(), 0.0);
        assertEquals(0.0, distortion.getK2(), 0.0);

        // set new values
        final var kParams = new double[]{1.0, 2.0, 3.0};
        distortion.setKParams(kParams);

        // check correctness
        assertSame(kParams, distortion.getKParams());
        assertEquals(1.0, distortion.getK1(), 0.0);
        assertEquals(2.0, distortion.getK2(), 0.0);
    }

    @Test
    void testDistortUndistortPoint() throws NotSupportedException, DistortionException {
        for (var j = 0; j < TIMES; j++) {
            final var randomizer = new UniformRandomizer();

            final var k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
            final var k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);

            final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

            final var distorted = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

            final var distortion = new RadialDistortion(k1, k2, center);

            final var undistorted = distortion.undistort(distorted);
            final var distorted2 = distortion.distort(undistorted);

            assertEquals(0.0, distorted.distanceTo(distorted2), ERROR);
        }
    }

    @Test
    void testDistortUndistortPoints() throws NotSupportedException, DistortionException {
        for (var j = 0; j < TIMES; j++) {
            final var randomizer = new UniformRandomizer();

            final var k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
            final var k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);

            final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

            final var distortion = new RadialDistortion(k1, k2, center);

            final var distortedPoints = new ArrayList<Point2D>();
            final var undistortedPoints = new ArrayList<Point2D>();
            List<Point2D> distortedPoints2;
            Point2D distortedPoint;
            for (var i = 0; i < NUM_POINTS; i++) {
                distortedPoint = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                        randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

                distortedPoints.add(distortedPoint);
                undistortedPoints.add(distortion.undistort(distortedPoint));
            }

            distortedPoints2 = distortion.distort(undistortedPoints);

            for (var i = 0; i < NUM_POINTS; i++) {
                assertEquals(0.0, distortedPoints.get(i).distanceTo(distortedPoints2.get(i)), ERROR);
            }
        }
    }

    @Test
    void testDistortPoint() throws NotSupportedException, DistortionException {
        final var randomizer = new UniformRandomizer();

        final var k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        final var k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);

        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

        final var undistorted = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

        final var distortion = new RadialDistortion(k1, k2, center);

        final var distorted1 = distortion.distort(undistorted);
        final var distorted2 = Point2D.create();
        distortion.distort(undistorted, distorted2);

        assertEquals(distorted1, distorted2);

        final var xu = undistorted.getInhomX();
        final var yu = undistorted.getInhomY();
        final var xc = center.getInhomX();
        final var yc = center.getInhomY();
        final var diffX = xu - xc;
        final var diffY = yu - yc;
        final var r2 = diffX * diffX + diffY * diffY;
        final var r4 = r2 * r2;
        final var factor = 1.0 + (k1 * r2) + (k2 * r4);
        final var xd = xc + diffX * factor;
        final var yd = yc + diffY * factor;

        assertEquals(xd, distorted1.getInhomX(), ERROR);
        assertEquals(yd, distorted1.getInhomY(), ERROR);
    }

    @Test
    void testDistortPoints() throws NotSupportedException, DistortionException {
        final var randomizer = new UniformRandomizer();

        final var k1 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);
        final var k2 = randomizer.nextDouble(MIN_PARAM_VALUE, MAX_PARAM_VALUE);

        final var center = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));

        final var distortion = new RadialDistortion(k1, k2, center);

        final var undistortedPoints = new ArrayList<Point2D>();
        final var distortedPoints1 = new ArrayList<Point2D>();
        final List<Point2D> distortedPoints2;
        Point2D undistortedPoint;
        Point2D distortedPoint1;
        Point2D distortedPoint2;
        for (var i = 0; i < NUM_POINTS; i++) {
            undistortedPoint = new InhomogeneousPoint2D(randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE),
                    randomizer.nextDouble(MIN_POINT_VALUE, MAX_POINT_VALUE));
            undistortedPoints.add(undistortedPoint);
            distortedPoints1.add(Point2D.create());
        }

        // distort
        distortion.distort(undistortedPoints, distortedPoints1);
        distortedPoints2 = distortion.distort(undistortedPoints);

        assertEquals(NUM_POINTS, distortedPoints1.size());
        assertEquals(NUM_POINTS, distortedPoints2.size());
        for (var i = 0; i < NUM_POINTS; i++) {
            undistortedPoint = undistortedPoints.get(i);
            distortedPoint1 = distortedPoints1.get(i);
            distortedPoint2 = distortedPoints2.get(i);

            assertEquals(distortedPoint1, distortedPoint2);

            final var xu = undistortedPoint.getInhomX();
            final var yu = undistortedPoint.getInhomY();
            final var xc = center.getInhomX();
            final var yc = center.getInhomY();
            final var diffX = xu - xc;
            final var diffY = yu - yc;
            final var r2 = diffX * diffX + diffY * diffY;
            final var r4 = r2 * r2;
            final var factor = 1.0 + (k1 * r2) + (k2 * r4);
            final var xd = xc + diffX * factor;
            final var yd = yc + diffY * factor;

            assertEquals(xd, distortedPoint1.getInhomX(), ERROR);
            assertEquals(yd, distortedPoint1.getInhomY(), ERROR);
        }
    }

    @Test
    void testSerializeDeserialize() throws DistortionException, IOException, ClassNotFoundException {
        final var distortion1 = new RadialDistortion();

        // set new values
        final var center = new InhomogeneousPoint2D();
        distortion1.setCenter(center);
        distortion1.setHorizontalFocalLength(0.5);
        distortion1.setVerticalFocalLength(0.4);
        distortion1.setSkew(1e-3);
        distortion1.setK1(1.0);
        distortion1.setK2(0.5);

        // check
        assertSame(center, distortion1.getCenter());
        assertEquals(0.5, distortion1.getHorizontalFocalLength(), 0.0);
        assertEquals(0.4, distortion1.getVerticalFocalLength(), 0.0);
        assertEquals(1e-3, distortion1.getSkew(), 0.0);
        assertEquals(1.0, distortion1.getK1(), 0.0);
        assertEquals(0.5, distortion1.getK2(), 0.0);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(distortion1);
        final var distortion2 = SerializationHelper.<RadialDistortion>deserialize(bytes);

        // check
        assertEquals(distortion1.getCenter(), distortion2.getCenter());
        assertEquals(distortion1.getHorizontalFocalLength(), distortion2.getHorizontalFocalLength(), 0.0);
        assertEquals(distortion1.getVerticalFocalLength(), distortion2.getVerticalFocalLength(), 0.0);
        assertEquals(distortion1.getSkew(), distortion2.getSkew(), 0.0);
        assertEquals(distortion1.getK1(), distortion2.getK1(), 0.0);
        assertEquals(distortion1.getK2(), distortion2.getK2(), 0.0);
    }
}
