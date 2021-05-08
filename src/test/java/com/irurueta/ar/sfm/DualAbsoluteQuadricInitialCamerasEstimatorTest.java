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

import com.irurueta.ar.epipolar.FundamentalMatrix;
import com.irurueta.ar.epipolar.InvalidFundamentalMatrixException;
import com.irurueta.ar.epipolar.InvalidPairOfCamerasException;
import com.irurueta.geometry.*;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class DualAbsoluteQuadricInitialCamerasEstimatorTest implements
        InitialCamerasEstimatorListener {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_ANGLE_DEGREES = -30.0;
    private static final double MAX_ANGLE_DEGREES = 30.0;

    private static final double MIN_CAMERA_SEPARATION = 5.0;
    private static final double MAX_CAMERA_SEPARATION = 10.0;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-5;

    private static final int TIMES = 50;

    @Test
    public void testConstructor() {
        DualAbsoluteQuadricInitialCamerasEstimator estimator =
                new DualAbsoluteQuadricInitialCamerasEstimator();

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(estimator.getMethod(),
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);
        assertFalse(estimator.isReady());

        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        estimator = new DualAbsoluteQuadricInitialCamerasEstimator(
                fundamentalMatrix);

        // check default values
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
        assertNull(estimator.getListener());
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(estimator.getMethod(),
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);
        assertTrue(estimator.isReady());

        estimator = new DualAbsoluteQuadricInitialCamerasEstimator(this);

        // check default values
        assertNull(estimator.getFundamentalMatrix());
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(estimator.getMethod(),
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);
        assertFalse(estimator.isReady());

        estimator = new DualAbsoluteQuadricInitialCamerasEstimator(
                fundamentalMatrix, this);

        // check default values
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
        assertSame(estimator.getListener(), this);
        assertFalse(estimator.isLocked());
        assertNull(estimator.getEstimatedLeftCamera());
        assertNull(estimator.getEstimatedRightCamera());
        assertEquals(estimator.getMethod(),
                InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC);
        assertTrue(estimator.isReady());
    }

    @Test
    public void testGetSetFundamentalMatrix() throws
            com.irurueta.geometry.estimators.LockedException {
        final DualAbsoluteQuadricInitialCamerasEstimator estimator =
                new DualAbsoluteQuadricInitialCamerasEstimator();

        // check default value
        assertNull(estimator.getFundamentalMatrix());

        // set new value
        final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix();
        estimator.setFundamentalMatrix(fundamentalMatrix);

        // check correctness
        assertSame(estimator.getFundamentalMatrix(), fundamentalMatrix);
    }

    @Test
    public void testGetSetListener() {
        final DualAbsoluteQuadricInitialCamerasEstimator estimator =
                new DualAbsoluteQuadricInitialCamerasEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check correctness
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetAspectRatio()
            throws com.irurueta.geometry.estimators.LockedException {
        final DualAbsoluteQuadricInitialCamerasEstimator estimator =
                new DualAbsoluteQuadricInitialCamerasEstimator();

        // check default value
        assertEquals(estimator.getAspectRatio(), 1.0, 0.0);

        // set new value
        estimator.setAspectRatio(-1.0);

        // check correctness
        assertEquals(estimator.getAspectRatio(), -1.0, 0.0);
    }

    @Test
    public void testEstimate() throws InvalidPairOfCamerasException,
            LockedException, NotReadyException,
            InitialCamerasEstimationFailedException, CameraException,
            NotAvailableException, InvalidFundamentalMatrixException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double aspectRatio = 1.0;
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPoint, principalPoint,
                            skewness);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1,
                    camera2);

            // generate initial cameras from fundamental matrix
            final DualAbsoluteQuadricInitialCamerasEstimator estimator =
                    new DualAbsoluteQuadricInitialCamerasEstimator(
                            fundamentalMatrix, this);
            estimator.setAspectRatio(aspectRatio);

            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedLeftCamera());
            assertNull(estimator.getEstimatedRightCamera());

            estimator.estimate();

            final PinholeCamera camera1b = estimator.getEstimatedLeftCamera();
            final PinholeCamera camera2b = estimator.getEstimatedRightCamera();

            camera1b.decompose();
            camera2b.decompose();

            final PinholeCameraIntrinsicParameters intrinsic1b =
                    camera1b.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters intrinsic2b =
                    camera2b.getIntrinsicParameters();

            final Rotation3D rotation1b = camera1b.getCameraRotation();
            final Rotation3D rotation2b = camera2b.getCameraRotation();

            final Point3D center1b = camera1b.getCameraCenter();
            final Point3D center2b = camera2b.getCameraCenter();

            if (Math.abs(intrinsic1b.getHorizontalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            if (Math.abs(intrinsic1b.getVerticalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            if (Math.abs(intrinsic1b.getSkewness()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            if (Math.abs(intrinsic1b.getHorizontalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getHorizontalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);
            if (Math.abs(intrinsic1b.getVerticalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getVerticalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);

            if (Math.abs(intrinsic2b.getHorizontalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2b.getVerticalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2b.getSkewness()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2b.getHorizontalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getHorizontalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2b.getVerticalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getVerticalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);

            assertNotNull(rotation1b);
            assertNotNull(rotation2b);
            assertNotNull(center1b);
            assertNotNull(center2b);

            // check that estimated cameras generate the same input fundamental
            // matrix
            final FundamentalMatrix fundamentalMatrixB = new FundamentalMatrix(
                    camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final Point2D epipole1 = camera1.project(center2);
            final Point2D epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final Point2D epipole1a = fundamentalMatrix.getLeftEpipole();
            final Point2D epipole2a = fundamentalMatrix.getRightEpipole();

            final Point2D epipole1b = fundamentalMatrixB.getLeftEpipole();
            final Point2D epipole2b = fundamentalMatrixB.getRightEpipole();

            if (epipole1.distanceTo(epipole1a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole1.distanceTo(epipole1a), 0.0, ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole2.distanceTo(epipole2a), 0.0,
                    LARGE_ABSOLUTE_ERROR);

            if (epipole1.distanceTo(epipole1b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole1.distanceTo(epipole1b), 0.0, ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole2.distanceTo(epipole2b), 0.0,
                    LARGE_ABSOLUTE_ERROR);

            // generate random 3D points
            final Point3D point3Da = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D point3Db = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            // project 3D points with each pair of cameras
            final Point2D point2D1a = camera1.project(point3Da);
            final Point2D point2D2a = camera2.project(point3Da);

            final Point2D point2D1b = camera1b.project(point3Db);
            final Point2D point2D2b = camera2b.project(point3Db);

            // obtain epipolar lines
            final Line2D line1a = fundamentalMatrix.getLeftEpipolarLine(point2D2a);
            final Line2D line2a = fundamentalMatrix.getRightEpipolarLine(point2D1a);

            final Line2D line1b = fundamentalMatrixB.getLeftEpipolarLine(point2D2b);
            final Line2D line2b = fundamentalMatrixB.getRightEpipolarLine(point2D1b);

            // check that points lie on their corresponding epipolar lines
            if (!line1a.isLocus(point2D1a, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(line1a.isLocus(point2D1a, ABSOLUTE_ERROR));
            if (!line2a.isLocus(point2D2a, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(line2a.isLocus(point2D2a, ABSOLUTE_ERROR));

            if (!line1b.isLocus(point2D1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(line1b.isLocus(point2D1b, ABSOLUTE_ERROR));
            if (!line2b.isLocus(point2D2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(line2b.isLocus(point2D2b, ABSOLUTE_ERROR));

            // back-project epipolar lines for each pair of cameras and check that
            // each pair of lines correspond to the same epipolar plane
            final Plane epipolarPlane1a = camera1.backProject(line1a);
            final Plane epipolarPlane2a = camera2.backProject(line2a);

            final Plane epipolarPlane1b = camera1b.backProject(line1b);
            final Plane epipolarPlane2b = camera2b.backProject(line2b);

            if (!epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
            if (!epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

            // check that 3D point and both camera centers for each pair of
            // cameras belong to their corresponding epipolar plane
            if (!epipolarPlane1a.isLocus(point3Da, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1a.isLocus(point3Da, ABSOLUTE_ERROR));
            if (!epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
            if (!epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));

            if (!epipolarPlane2a.isLocus(point3Da, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2a.isLocus(point3Da, ABSOLUTE_ERROR));
            if (!epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
            if (!epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

            if (!epipolarPlane1b.isLocus(point3Db, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1b.isLocus(point3Db, ABSOLUTE_ERROR));
            if (!epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
            if (!epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));

            if (!epipolarPlane2b.isLocus(point3Db, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2b.isLocus(point3Db, ABSOLUTE_ERROR));
            if (!epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
            if (!epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGenerateInitialMetricCamerasUsingDAQ1()
            throws InvalidPairOfCamerasException,
            InitialCamerasEstimationFailedException, CameraException,
            NotAvailableException, NotReadyException,
            InvalidFundamentalMatrixException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPoint, principalPoint,
                            skewness);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1,
                    camera2);

            // generate initial cameras from fundamental matrix
            final PinholeCamera camera1b = new PinholeCamera();
            final PinholeCamera camera2b = new PinholeCamera();
            DualAbsoluteQuadricInitialCamerasEstimator.
                    generateInitialMetricCamerasUsingDAQ(fundamentalMatrix,
                            camera1b, camera2b);

            camera1b.decompose();
            camera2b.decompose();

            final PinholeCameraIntrinsicParameters intrinsic1b =
                    camera1b.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters intrinsic2b =
                    camera2b.getIntrinsicParameters();

            final Rotation3D rotation1b = camera1b.getCameraRotation();
            final Rotation3D rotation2b = camera2b.getCameraRotation();

            final Point3D center1b = camera1b.getCameraCenter();
            final Point3D center2b = camera2b.getCameraCenter();

            if (Math.abs(intrinsic1b.getHorizontalFocalLength() - focalLength) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getHorizontalFocalLength(), focalLength,
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic1b.getVerticalFocalLength() - focalLength) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getVerticalFocalLength(), focalLength,
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic1b.getSkewness()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getSkewness(), 0.0, LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic1b.getHorizontalPrincipalPoint()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getHorizontalPrincipalPoint(), 0.0,
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic1b.getVerticalPrincipalPoint()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getVerticalPrincipalPoint(), 0.0,
                    LARGE_ABSOLUTE_ERROR);

            if (Math.abs(intrinsic2b.getHorizontalFocalLength() - focalLength) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getHorizontalFocalLength(), focalLength,
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2b.getVerticalFocalLength() - focalLength) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getVerticalFocalLength(), focalLength,
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2b.getSkewness()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getSkewness(), 0.0, LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2b.getHorizontalPrincipalPoint()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getHorizontalPrincipalPoint(), 0.0,
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2b.getVerticalPrincipalPoint()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getVerticalPrincipalPoint(), 0.0,
                    LARGE_ABSOLUTE_ERROR);

            assertNotNull(rotation1b);
            assertNotNull(rotation2b);
            assertNotNull(center1b);
            assertNotNull(center2b);

            // check that estimated cameras generate the same input fundamental
            // matrix
            final FundamentalMatrix fundamentalMatrixB = new FundamentalMatrix(
                    camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final Point2D epipole1 = camera1.project(center2);
            final Point2D epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final Point2D epipole1a = fundamentalMatrix.getLeftEpipole();
            final Point2D epipole2a = fundamentalMatrix.getRightEpipole();

            final Point2D epipole1b = fundamentalMatrixB.getLeftEpipole();
            final Point2D epipole2b = fundamentalMatrixB.getRightEpipole();

            if (epipole1.distanceTo(epipole1a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole1.distanceTo(epipole1a), 0.0, ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole2.distanceTo(epipole2a), 0.0,
                    LARGE_ABSOLUTE_ERROR);

            if (epipole1.distanceTo(epipole1b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole1.distanceTo(epipole1b), 0.0, ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole2.distanceTo(epipole2b), 0.0,
                    LARGE_ABSOLUTE_ERROR);

            // generate random 3D points
            final Point3D point3Da = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D point3Db = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            // project 3D points with each pair of cameras
            final Point2D point2D1a = camera1.project(point3Da);
            final Point2D point2D2a = camera2.project(point3Da);

            final Point2D point2D1b = camera1b.project(point3Db);
            final Point2D point2D2b = camera2b.project(point3Db);

            // obtain epipolar lines
            final Line2D line1a = fundamentalMatrix.getLeftEpipolarLine(point2D2a);
            final Line2D line2a = fundamentalMatrix.getRightEpipolarLine(point2D1a);

            final Line2D line1b = fundamentalMatrixB.getLeftEpipolarLine(point2D2b);
            final Line2D line2b = fundamentalMatrixB.getRightEpipolarLine(point2D1b);

            // check that points lie on their corresponding epipolar lines
            assertTrue(line1a.isLocus(point2D1a, ABSOLUTE_ERROR));
            assertTrue(line2a.isLocus(point2D2a, ABSOLUTE_ERROR));

            assertTrue(line1b.isLocus(point2D1b, ABSOLUTE_ERROR));
            assertTrue(line2b.isLocus(point2D2b, ABSOLUTE_ERROR));

            // back-project epipolar lines for each pair of cameras and check that
            // each pair of lines correspond to the same epipolar plane
            final Plane epipolarPlane1a = camera1.backProject(line1a);
            final Plane epipolarPlane2a = camera2.backProject(line2a);

            final Plane epipolarPlane1b = camera1b.backProject(line1b);
            final Plane epipolarPlane2b = camera2b.backProject(line2b);

            assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

            // check that 3D point and both camera centers for each pair of
            // cameras belong to their corresponding epipolar plane
            assertTrue(epipolarPlane1a.isLocus(point3Da, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));

            assertTrue(epipolarPlane2a.isLocus(point3Da, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

            assertTrue(epipolarPlane1b.isLocus(point3Db, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));

            assertTrue(epipolarPlane2b.isLocus(point3Db, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGenerateInitialMetricCamerasUsingDAQ2()
            throws InvalidPairOfCamerasException,
            InitialCamerasEstimationFailedException, CameraException,
            NotAvailableException, NotReadyException,
            InvalidFundamentalMatrixException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double alphaEuler1 = 0.0;
            final double betaEuler1 = 0.0;
            final double gammaEuler1 = 0.0;
            final double alphaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double betaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double gammaEuler2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            final double aspectRatio = 1.0;
            final double skewness = 0.0;
            final double principalPoint = 0.0;

            final double cameraSeparation = randomizer.nextDouble(
                    MIN_CAMERA_SEPARATION, MAX_CAMERA_SEPARATION);

            final Point3D center1 = new InhomogeneousPoint3D(0.0, 0.0, 0.0);
            final Point3D center2 = new InhomogeneousPoint3D(
                    center1.getInhomX() + cameraSeparation,
                    center1.getInhomY() + cameraSeparation,
                    center1.getInhomZ() + cameraSeparation);

            final MatrixRotation3D rotation1 = new MatrixRotation3D(alphaEuler1,
                    betaEuler1, gammaEuler1);
            final MatrixRotation3D rotation2 = new MatrixRotation3D(alphaEuler2,
                    betaEuler2, gammaEuler2);

            final PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                            focalLength, principalPoint, principalPoint,
                            skewness);

            final PinholeCamera camera1 = new PinholeCamera(intrinsic, rotation1,
                    center1);
            final PinholeCamera camera2 = new PinholeCamera(intrinsic, rotation2,
                    center2);

            final FundamentalMatrix fundamentalMatrix = new FundamentalMatrix(camera1,
                    camera2);

            // generate initial cameras from fundamental matrix
            final PinholeCamera camera1b = new PinholeCamera();
            final PinholeCamera camera2b = new PinholeCamera();
            DualAbsoluteQuadricInitialCamerasEstimator.
                    generateInitialMetricCamerasUsingDAQ(fundamentalMatrix,
                            aspectRatio, camera1b, camera2b);

            camera1b.decompose();
            camera2b.decompose();

            final PinholeCameraIntrinsicParameters intrinsic1b =
                    camera1b.getIntrinsicParameters();
            final PinholeCameraIntrinsicParameters intrinsic2b =
                    camera2b.getIntrinsicParameters();

            final Rotation3D rotation1b = camera1b.getCameraRotation();
            final Rotation3D rotation2b = camera2b.getCameraRotation();

            final Point3D center1b = camera1b.getCameraCenter();
            final Point3D center2b = camera2b.getCameraCenter();

            if (Math.abs(intrinsic1b.getHorizontalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            if (Math.abs(intrinsic1b.getVerticalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            if (Math.abs(intrinsic1b.getSkewness()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            if (Math.abs(intrinsic1b.getHorizontalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getHorizontalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);
            if (Math.abs(intrinsic1b.getVerticalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic1b.getVerticalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);

            if (Math.abs(intrinsic2b.getHorizontalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getHorizontalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2b.getVerticalFocalLength() - focalLength) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getVerticalFocalLength(), focalLength,
                    ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2b.getSkewness()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getSkewness(), 0.0, ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2b.getHorizontalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getHorizontalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);
            if (Math.abs(intrinsic2b.getVerticalPrincipalPoint()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(intrinsic2b.getVerticalPrincipalPoint(), 0.0,
                    ABSOLUTE_ERROR);

            assertNotNull(rotation1b);
            assertNotNull(rotation2b);
            assertNotNull(center1b);
            assertNotNull(center2b);

            // check that estimated cameras generate the same input fundamental
            // matrix
            final FundamentalMatrix fundamentalMatrixB = new FundamentalMatrix(
                    camera1b, camera2b);

            // compare fundamental matrices by checking generated epipolar
            // geometry
            fundamentalMatrix.normalize();
            fundamentalMatrixB.normalize();

            final Point2D epipole1 = camera1.project(center2);
            final Point2D epipole2 = camera2.project(center1);

            fundamentalMatrix.computeEpipoles();
            fundamentalMatrixB.computeEpipoles();

            final Point2D epipole1a = fundamentalMatrix.getLeftEpipole();
            final Point2D epipole2a = fundamentalMatrix.getRightEpipole();

            final Point2D epipole1b = fundamentalMatrixB.getLeftEpipole();
            final Point2D epipole2b = fundamentalMatrixB.getRightEpipole();

            if (epipole1.distanceTo(epipole1a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole1.distanceTo(epipole1a), 0.0, ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2a) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole2.distanceTo(epipole2a), 0.0,
                    LARGE_ABSOLUTE_ERROR);

            if (epipole1.distanceTo(epipole1b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole1.distanceTo(epipole1b), 0.0, ABSOLUTE_ERROR);
            if (epipole2.distanceTo(epipole2b) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(epipole2.distanceTo(epipole2b), 0.0,
                    LARGE_ABSOLUTE_ERROR);

            // generate random 3D points
            final Point3D point3Da = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            final Point3D point3Db = new InhomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            // project 3D points with each pair of cameras
            final Point2D point2D1a = camera1.project(point3Da);
            final Point2D point2D2a = camera2.project(point3Da);

            final Point2D point2D1b = camera1b.project(point3Db);
            final Point2D point2D2b = camera2b.project(point3Db);

            // obtain epipolar lines
            final Line2D line1a = fundamentalMatrix.getLeftEpipolarLine(point2D2a);
            final Line2D line2a = fundamentalMatrix.getRightEpipolarLine(point2D1a);

            final Line2D line1b = fundamentalMatrixB.getLeftEpipolarLine(point2D2b);
            final Line2D line2b = fundamentalMatrixB.getRightEpipolarLine(point2D1b);

            // check that points lie on their corresponding epipolar lines
            assertTrue(line1a.isLocus(point2D1a, ABSOLUTE_ERROR));
            assertTrue(line2a.isLocus(point2D2a, ABSOLUTE_ERROR));

            assertTrue(line1b.isLocus(point2D1b, ABSOLUTE_ERROR));
            assertTrue(line2b.isLocus(point2D2b, ABSOLUTE_ERROR));

            // back-project epipolar lines for each pair of cameras and check that
            // each pair of lines correspond to the same epipolar plane
            final Plane epipolarPlane1a = camera1.backProject(line1a);
            final Plane epipolarPlane2a = camera2.backProject(line2a);

            final Plane epipolarPlane1b = camera1b.backProject(line1b);
            final Plane epipolarPlane2b = camera2b.backProject(line2b);

            assertTrue(epipolarPlane1a.equals(epipolarPlane2a, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1b.equals(epipolarPlane2b, ABSOLUTE_ERROR));

            // check that 3D point and both camera centers for each pair of
            // cameras belong to their corresponding epipolar plane
            assertTrue(epipolarPlane1a.isLocus(point3Da, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1a.isLocus(center1, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1a.isLocus(center2, ABSOLUTE_ERROR));

            assertTrue(epipolarPlane2a.isLocus(point3Da, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane2a.isLocus(center1, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane2a.isLocus(center2, ABSOLUTE_ERROR));

            assertTrue(epipolarPlane1b.isLocus(point3Db, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1b.isLocus(center1b, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane1b.isLocus(center2b, ABSOLUTE_ERROR));

            assertTrue(epipolarPlane2b.isLocus(point3Db, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane2b.isLocus(center1b, ABSOLUTE_ERROR));
            assertTrue(epipolarPlane2b.isLocus(center2b, ABSOLUTE_ERROR));

            numValid++;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onStart(final InitialCamerasEstimator estimator) {
        checkLocked((DualAbsoluteQuadricInitialCamerasEstimator) estimator);
    }

    @Override
    public void onFinish(final InitialCamerasEstimator estimator,
                         final PinholeCamera estimatedLeftCamera,
                         final PinholeCamera estimatedRightCamera) {
        checkLocked((DualAbsoluteQuadricInitialCamerasEstimator) estimator);
    }

    @Override
    public void onFail(final InitialCamerasEstimator estimator,
                       final InitialCamerasEstimationFailedException e) {
        checkLocked((DualAbsoluteQuadricInitialCamerasEstimator) estimator);
    }

    private void checkLocked(
            final DualAbsoluteQuadricInitialCamerasEstimator estimator) {
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        } catch (final com.irurueta.geometry.estimators.NotReadyException |
                InitialCamerasEstimationFailedException ex) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAspectRatio(-1.0);
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        }
        try {
            estimator.setFundamentalMatrix(null);
            fail("LockedException expected but not thrown");
        } catch (final com.irurueta.geometry.estimators.LockedException ignore) {
        }
    }
}
