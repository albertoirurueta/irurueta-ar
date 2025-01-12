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
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.statistics.UniformRandomizer;

/**
 * Compares two fundamental matrices by estimating average epipolar distances.
 * This class uses epipolar geometry to determine how similar fundamental
 * matrices are. This is done by assuming a certain size on the retinal planes
 * and generating random 2D points to obtain their associated epipolar lines and
 * the distances of matched points to those epipolar lines
 * This class simply computes the norm of the difference of both fundamental
 * matrices. The smaller the value the more similar the fundamental matrices
 * will be from a pure algebraic point of view.
 */
public class EpipolarDistanceFundamentalMatrixComparator extends FundamentalMatrixComparator {

    /**
     * Defines default minimum horizontal coordinate when generating random
     * samples.
     */
    public static final double DEFAULT_MIN_X = 0.0;

    /**
     * Defines default maximum horizontal coordinate when generating random
     * samples.
     */
    public static final double DEFAULT_MAX_X = 640.0;

    /**
     * Defines default minimum vertical coordinate when generating random
     * samples.
     */
    public static final double DEFAULT_MIN_Y = 0.0;

    /**
     * Defines default maximum vertical coordinate when generating random
     * samples.
     */
    public static final double DEFAULT_MAX_Y = 480.0;

    /**
     * Default number of random samples to generate to compare fundamental
     * matrices.
     */
    public static final int DEFAULT_N_SAMPLES = 100;

    /**
     * Minimum number of samples that must be generated to compare fundamental
     * matrices.
     */
    public static final int MIN_N_SAMPLES = 1;

    /**
     * Minimum disparity factor respect to retinal plane size defined by
     * minimum and maximum samples coordinates. When computing residuals,
     * matched samples are created along the corresponding epipolar lines with a
     * random disparity within provided range, and then the epipolar line for
     * the randomly generated matched sample is generated on the original view
     * to determine the distance to such line and the original sample.
     */
    public static final double DEFAULT_MIN_DISPARITY_FACTOR = -0.1;

    /**
     * Maximum disparity factor respect to retinal plane size defined by
     * minimum and maximum samples coordinates. When computing residuals,
     * matched samples are created along the corresponding epipolar lines with a
     * random disparity within provided range, and then the epipolar line for
     * the randomly generated matched sample is generated on the original view
     * to determine the distance to such line and the original sample.
     */
    public static final double DEFAULT_MAX_DISPARITY_FACTOR = 0.1;

    /**
     * Default factor to determine maximum number of iterations respect to the
     * number of samples to compute comparison.
     */
    public static final double DEFAULT_MAX_ITERATIONS_FACTOR = 10.0;

    /**
     * Minimum value for the factor to determine maximum number of iterations
     * respect to the number of samples to compute comparison.
     */
    public static final double MIN_MAX_ITERATIONS_FACTOR = 1.0;

    /**
     * Default amount of progress variation before notifying a change in
     * comparison progress. By default, this is set to 5%.
     */
    public static final float DEFAULT_PROGRESS_DELTA = 0.05f;

    /**
     * Minimum allowed value for progress delta.
     */
    public static final float MIN_PROGRESS_DELTA = 0.0f;

    /**
     * Maximum allowed value for progress delta.
     */
    public static final float MAX_PROGRESS_DELTA = 1.0f;

    /**
     * Minimum horizontal coordinate when generating random samples.
     */
    private double minX;

    /**
     * Maximum horizontal coordinate when generating random samples.
     */
    private double maxX;

    /**
     * Minimum vertical coordinate when generating random samples.
     */
    private double minY;

    /**
     * Maximum vertical coordinate when generating random samples.
     */
    private double maxY;

    /**
     * Number of random samples to generate to compare fundamental matrices.
     */
    private int nSamples;

    /**
     * Minimum horizontal disparity factor respect to retinal plane size defined
     * by minimum and maximum samples coordinates. When computing residuals,
     * matched samples are created along the corresponding epipolar lines with a
     * random disparity within provided range of disparities, and then the
     * epipolar line for the randomly generated matched sample is generated on
     * the original view to determine the distance to such line and the original
     * sample.
     */
    private double minHorizontalDisparityFactor;

    /**
     * Maximum horizontal disparity factor respect to retinal plane size defined
     * by minimum and maximum samples coordinates. When computing residuals,
     * matched samples are created along the corresponding epipolar lines with a
     * random disparity within provided range of disparities, and then the
     * epipolar line for the randomly generated matched sample is generated on
     * the original view to determine the distance to such line and the original
     * sample.
     */
    private double maxHorizontalDisparityFactor;

    /**
     * Minimum vertical disparity factor respect to retinal plane size defined
     * by minimum and maximum samples coordinates. When computing residuals,
     * matched samples are created along the corresponding epipolar lines with a
     * random disparity within provided range of disparities, and then the
     * epipolar line for the randomly generated matched sample is generated on
     * the original view to determine the distance to such line and the original
     * sample.
     */
    private double minVerticalDisparityFactor;

    /**
     * Maximum vertical disparity factor respect to retinal plane size defined
     * by minimum and maximum samples coordinates. When computing residuals,
     * matched samples are created along the corresponding epipolar lines with a
     * random disparity within provided range of disparities, and then the
     * epipolar line for the randomly generated matched sample is generated on
     * the original view to determine the distance to such line and the original
     * sample.
     */
    private double maxVerticalDisparityFactor;

    /**
     * Factor to determine maximum number of iterations respect to the
     * number of samples to compute comparison.
     */
    private double maxIterationsFactor;

    /**
     * Amount of progress variation before notifying a progress change during
     * comparison.
     */
    protected float progressDelta;

    /**
     * Constructor.
     */
    public EpipolarDistanceFundamentalMatrixComparator() {
        super();
        init();
    }

    /**
     * Constructor.
     *
     * @param groundTruthFundamentalMatrix fundamental matrix to be considered
     *                                     as ground truth to compare against.
     * @param otherFundamentalMatrix       other fundamental matrix being compared.
     */
    public EpipolarDistanceFundamentalMatrixComparator(
            final FundamentalMatrix groundTruthFundamentalMatrix, final FundamentalMatrix otherFundamentalMatrix) {
        super(groundTruthFundamentalMatrix, otherFundamentalMatrix);
        init();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events generated by this class.
     */
    public EpipolarDistanceFundamentalMatrixComparator(final FundamentalMatrixComparatorListener listener) {
        super(listener);
        init();
    }

    /**
     * Constructor.
     *
     * @param groundTruthFundamentalMatrix fundamental matrix to be considered
     *                                     as ground truth to compare against.
     * @param otherFundamentalMatrix       other fundamental matrix being compared.
     * @param listener                     listener to handle events generated by this class.
     */
    public EpipolarDistanceFundamentalMatrixComparator(
            final FundamentalMatrix groundTruthFundamentalMatrix, final FundamentalMatrix otherFundamentalMatrix,
            final FundamentalMatrixComparatorListener listener) {
        super(groundTruthFundamentalMatrix, otherFundamentalMatrix, listener);
        init();
    }

    /**
     * Returns minimum horizontal coordinate when generating random samples.
     *
     * @return minimum horizontal coordinate when generating random samples.
     */
    public double getMinX() {
        return minX;
    }

    /**
     * Returns maximum horizontal coordinate when generating random samples.
     *
     * @return maximum horizontal coordinate when generating random samples.
     */
    public double getMaxX() {
        return maxX;
    }

    /**
     * Sets minimum and maximum horizontal coordinates when generating random
     * samples.
     *
     * @param minX minimum horizontal coordinate when generating random samples.
     * @param maxX maximum horizontal coordinate when generating random samples.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if minimum value is larger or equal
     *                                  than maximum one.
     */
    public void setMinMaxX(final double minX, final double maxX) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (minX >= maxX) {
            throw new IllegalArgumentException();
        }

        this.minX = minX;
        this.maxX = maxX;
    }

    /**
     * Returns minimum vertical coordinate when generating random samples.
     *
     * @return minimum vertical coordinate when generating random samples.
     */
    public double getMinY() {
        return minY;
    }

    /**
     * Returns maximum vertical coordinate when generating random samples.
     *
     * @return maximum vertical coordinate when generating random samples.
     */
    public double getMaxY() {
        return maxY;
    }

    /**
     * Sets minimum and maximum vertical coordinates when generating random
     * samples.
     *
     * @param minY minimum vertical coordinate when generating random samples.
     * @param maxY maximum vertical coordinate when generating random samples.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if minimum value is larger or equal
     *                                  than maximum one.
     */
    public void setMinMaxY(final double minY, final double maxY) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (minY >= maxY) {
            throw new IllegalArgumentException();
        }

        this.minY = minY;
        this.maxY = maxY;
    }

    /**
     * Returns number of random samples to generate to compare fundamental
     * matrices.
     *
     * @return number of random samples to generate to compare fundamental
     * matrices.
     */
    public int getNSamples() {
        return nSamples;
    }

    /**
     * Sets number of random samples to generate to compare fundamental matrices.
     *
     * @param nSamples number of random samples to generate to compare
     *                 fundamental matrices.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is less than 1.
     */
    public void setNSamples(final int nSamples) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (nSamples < MIN_N_SAMPLES) {
            throw new IllegalArgumentException();
        }

        this.nSamples = nSamples;
    }

    /**
     * Returns minimum horizontal disparity factor respect to retinal plane size
     * defined by minimum and maximum samples coordinates. When computing
     * residuals, matched samples are created along the corresponding epipolar
     * lines with a random disparity within provided range of disparities, and
     * then the epipolar line for the randomly generated matched sample is
     * generated on the original view to determine the distance to such line and
     * the original sample.
     *
     * @return minimum horizontal disparity factor.
     */
    public double getMinHorizontalDisparityFactor() {
        return minHorizontalDisparityFactor;
    }

    /**
     * Returns maximum horizontal disparity factor respect to retinal plane size
     * defined by minimum and maximum samples coordinates. When computing
     * residuals, matched samples are created along the corresponding epipolar
     * lines with a random disparity within provided range of disparities, and
     * then the epipolar line for the randomly generated matched sample is
     * generated on the original view to determine the distance to such line and
     * the original sample.
     *
     * @return maximum horizontal disparity factor.
     */
    public double getMaxHorizontalDisparityFactor() {
        return maxHorizontalDisparityFactor;
    }

    /**
     * Sets minimum and maximum horizontal disparity factor respect to retinal
     * plane size defined by minimum and maximum samples coordinates. When
     * computing residuals, matched samples are created along the corresponding
     * epipolar lines with a random disparity within provided range of
     * disparities, and then the epipolar line for the randomly generated matched
     * sample is generated on the original view to determine the distance to
     * such line and the original sample.
     *
     * @param minHorizontalDisparityFactor minimum horizontal disparity factor.
     * @param maxHorizontalDisparityFactor maximum horizontal disparity factor.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if minimum value is larger or equal
     *                                  than maximum one.
     */
    public void setMinMaxHorizontalDisparityFactor(
            final double minHorizontalDisparityFactor, final double maxHorizontalDisparityFactor)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (minHorizontalDisparityFactor >= maxHorizontalDisparityFactor) {
            throw new IllegalArgumentException();
        }

        this.minHorizontalDisparityFactor = minHorizontalDisparityFactor;
        this.maxHorizontalDisparityFactor = maxHorizontalDisparityFactor;
    }

    /**
     * Returns minimum vertical disparity factor respect to retinal plane size
     * defined by minimum and maximum samples coordinates. When computing
     * residuals, matched samples are created along the corresponding epipolar
     * lines with a random disparity within provided range of disparities, and
     * then the epipolar line for the randomly generated matched sample is
     * generated on the original view to determine the distance to such line and
     * the original sample.
     *
     * @return minimum vertical disparity factor.
     */
    public double getMinVerticalDisparityFactor() {
        return minVerticalDisparityFactor;
    }

    /**
     * Returns maximum vertical disparity factor respect to retinal plane size
     * defined by minimum and maximum samples coordinates. When computing
     * residuals, matched samples are created along the corresponding epipolar
     * lines with a random disparity within provided range of disparities, and
     * then the epipolar line for the randomly generated matched sample is
     * generated on the original view to determine the distance to such line and
     * the original sample.
     *
     * @return maximum vertical disparity factor.
     */
    public double getMaxVerticalDisparityFactor() {
        return maxVerticalDisparityFactor;
    }

    /**
     * Sets minimum and maximum vertical disparity factor respect to retinal
     * plane size defined by minimum and maximum samples coordinates. When
     * computing residuals, matched samples are created along the corresponding
     * epipolar lines with a random disparity within provided range of
     * disparities, and then the epipolar line for the randomly generated matched
     * sample is generated on the original view to determine the distance to
     * such line and the original sample.
     *
     * @param minVerticalDisparityFactor minimum vertical disparity factor.
     * @param maxVerticalDisparityFactor maximum vertical disparity factor.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if minimum value is larger or equal
     *                                  than maximum one.
     */
    public void setMinMaxVerticalDisparityFactor(
            final double minVerticalDisparityFactor, final double maxVerticalDisparityFactor) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (minVerticalDisparityFactor >= maxVerticalDisparityFactor) {
            throw new IllegalArgumentException();
        }

        this.minVerticalDisparityFactor = minVerticalDisparityFactor;
        this.maxVerticalDisparityFactor = maxVerticalDisparityFactor;
    }

    /**
     * Returns factor to determine maximum number of iterations respect to the
     * number of samples to compute comparison.
     *
     * @return factor to determine maximum number of iterations respect to the
     * number of samples to compute comparison.
     */
    public double getMaxIterationsFactor() {
        return maxIterationsFactor;
    }

    /**
     * Sets factor to determine maximum number of iterations respect to the
     * number of samples to compute comparison.
     *
     * @param maxIterationsFactor maximum number of iterations respect to the
     *                            number of samples to compute comparison.
     * @throws LockedException          if this instance is locked.
     * @throws IllegalArgumentException if provided value is less than 1.0.
     */
    public void setMaxIterationsFactor(final double maxIterationsFactor) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxIterationsFactor < MIN_MAX_ITERATIONS_FACTOR) {
            throw new IllegalArgumentException();
        }

        this.maxIterationsFactor = maxIterationsFactor;
    }

    /**
     * Returns amount of progress variation before notifying a progress change
     * during comparison computation.
     *
     * @return amount of progress variation before notifying a progress change
     * during comparison computation.
     */
    public float getProgressDelta() {
        return progressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change
     * during comparison computation.
     *
     * @param progressDelta amount of progress variation before notifying a
     *                      progress change during comparison computation.
     * @throws IllegalArgumentException if progress delta is less than zero or
     *                                  greater than 1.
     * @throws LockedException          if this estimator is locked.
     */
    public void setProgressDelta(final float progressDelta) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA || progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        this.progressDelta = progressDelta;
    }

    /**
     * Compares two fundamental matrices and returns the comparison value.
     * Comparison value will depend on the method implemented to compare both
     * fundamental matrices.
     *
     * @return comparison value. Typically, the smaller the absolute value the
     * more similar the fundamental matrices are.
     * @throws NotReadyException                    if this comparator is not  yet ready to start
     *                                              the comparison.
     * @throws LockedException                      if this instance is locked.
     * @throws FundamentalMatrixComparatorException if comparison fails due to
     *                                              some other reason.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public double compare() throws NotReadyException, LockedException, FundamentalMatrixComparatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            locked = true;

            if (listener != null) {
                listener.onCompareStart(this);
            }

            final var minHorizontalDisparity = minHorizontalDisparityFactor * (maxX - minX);
            final var maxHorizontalDisparity = maxHorizontalDisparityFactor * (maxX - minX);
            final var minVerticalDisparity = minVerticalDisparityFactor * (maxY - minY);
            final var maxVerticalDisparity = maxVerticalDisparityFactor * (maxY - minY);

            double d1;
            double d1prime;
            double d2;
            double d2prime;
            double inhomX;
            double inhomY;
            boolean repeat;
            int counter;
            final var m1 = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
            final var m2 = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
            final var l1real = new Line2D();
            final var l1est = new Line2D();
            final var l2real = new Line2D();
            final var l2est = new Line2D();
            final var randomizer = new UniformRandomizer();
            var avgDist = 0.0;
            final var maxIterations = (int) (nSamples * maxIterationsFactor);
            var currentIter = 0;
            float progress;
            var previousProgress = 0.0f;
            for (var i = 0; i < nSamples; i++) {
                repeat = true;
                counter = 0;
                while (repeat && (counter < nSamples)) {
                    // set x value
                    inhomX = randomizer.nextDouble(minX, maxX);

                    // set y value
                    inhomY = randomizer.nextDouble(minY, maxY);
                    m1.setInhomogeneousCoordinates(inhomX, inhomY);
                    m1.normalize();

                    // real epipolar line for random point m1
                    groundTruthFundamentalMatrix.rightEpipolarLine(m1, l2real);
                    l2real.normalize();

                    // check that epipolar line lies within retinal plane size
                    // taking into account the following equation
                    // x*l2.getA() + y*l2.getB() + l2.getC() = 0
                    double yMinX;
                    double yMaxX;
                    if (Math.abs(l2real.getB()) > Double.MIN_VALUE) {
                        // for x = mMinX
                        yMinX = -(minX * l2real.getA() + l2real.getC()) / l2real.getB();

                        // for x = mMaxX
                        yMaxX = -(maxX * l2real.getA() + l2real.getC()) / l2real.getB();
                    } else {
                        yMinX = l2real.getA() >= 0.0 ? -Double.MAX_VALUE : Double.MAX_VALUE;
                        yMaxX = -yMinX;
                    }

                    // for y = mMinY
                    final var xMinY = -(minY * l2real.getB() + l2real.getC()) / l2real.getA();

                    // for y = mMaxY
                    final var xMaxY = -(maxY * l2real.getB() + l2real.getC()) / l2real.getA();

                    // if epipolar line does not intersect second image, we need
                    // to repeat with a different sample
                    repeat = (((yMinX < minY) && (yMaxX < minY)) || ((yMinX > maxY)
                            && (yMaxX > maxY)) || ((xMinY < minX) && (xMaxY < minX))
                            || ((xMinY > maxX) && (xMaxY > maxX)));
                    counter++;
                    currentIter++;
                }

                if (counter >= nSamples) {
                    continue;
                }

                // choose point lying on epipolar line l2real :
                // m2.getX() * l2real.getA() + m2.getY() * l2real.getB() +
                // l2real.getC() = 0
                // choose random horizontal component within provided disparity:
                inhomX = m1.getInhomX() + randomizer.nextDouble(minHorizontalDisparity, maxHorizontalDisparity);
                if (Math.abs(l2real.getB()) > Double.MIN_VALUE) {
                    inhomY = -(inhomX * l2real.getA() + l2real.getC()) / l2real.getB();
                } else {
                    inhomY = Double.MAX_VALUE;
                }

                // if point lies outside retinal plane limits, try setting random
                // vertical component within provided disparity
                if ((inhomY < minY) || (inhomY > maxY)) {
                    inhomY = m1.getInhomY() + randomizer.nextDouble(minVerticalDisparity, maxVerticalDisparity);
                    inhomX = -(inhomY * l2real.getB() + l2real.getC()) / l2real.getA();
                }

                m2.setInhomogeneousCoordinates(inhomX, inhomY);
                m2.normalize();

                // estimated epipolar line for some random point m1
                otherFundamentalMatrix.rightEpipolarLine(m1, l2est);
                l2est.normalize();

                // compute distance from l2est to m2 (distance from estimated to
                // real)
                d1prime = Math.abs(l2est.signedDistance(m2));

                otherFundamentalMatrix.leftEpipolarLine(m2, l1est);
                l1est.normalize();
                d1 = Math.abs(l1est.signedDistance(m1));

                // repeat reversing roles of ground truth and other fundamental
                // matrix
                repeat = true;
                counter = 0;
                while (repeat && (counter < nSamples)) {
                    // set x value
                    inhomX = randomizer.nextDouble(minX, maxX);

                    // set y value
                    inhomY = randomizer.nextDouble(minY, maxY);
                    m1.setInhomogeneousCoordinates(inhomX, inhomY);
                    m1.normalize();

                    // real epipolar line for random point m1
                    otherFundamentalMatrix.rightEpipolarLine(m1, l2est);
                    l2est.normalize();

                    // check that epipolar line lies within retinal plane size
                    // taking into account the following equation
                    // x*l2.getA() + y*l2.getB() + l2.getC() = 0
                    double yMinX;
                    double yMaxX;
                    if (Math.abs(l2est.getB()) > Double.MIN_VALUE) {
                        // for x = mMinX
                        yMinX = -(minX * l2est.getA() + l2est.getC()) / l2est.getB();

                        // for x = mMaxX
                        yMaxX = -(maxX * l2est.getA() + l2est.getC()) / l2est.getB();
                    } else {
                        yMinX = l2est.getA() >= 0.0 ? -Double.MAX_VALUE : Double.MAX_VALUE;
                        yMaxX = -yMinX;
                    }

                    // for y = mMinY
                    final var xMinY = -(minY * l2est.getB() + l2est.getC()) / l2est.getA();

                    // for y = mMaxY
                    final var xMaxY = -(maxY * l2est.getB() + l2est.getC()) / l2est.getA();

                    // if epipolar line does not intersect second image, we need
                    // to repeat with a different sample
                    repeat = (((yMinX < minY) && (yMaxX < minY)) || ((yMinX > maxY)
                            && (yMaxX > maxY)) || ((xMinY < minX) && (xMaxY < minX))
                            || ((xMinY > maxX) && (xMaxY > maxX)));
                    counter++;
                    currentIter++;
                }

                if (counter >= nSamples) {
                    continue;
                }

                // choose point lying on epipolar line l2est :
                // m2.getX() * l2est.getA() + m2.getY() * l2est.getB() +
                // l2est.getC() = 0
                // choose random horizontal component within provided disparity:
                inhomX = m1.getInhomX() + randomizer.nextDouble(minHorizontalDisparity, maxHorizontalDisparity);
                if (Math.abs(l2real.getB()) > Double.MIN_VALUE) {
                    inhomY = -(inhomX * l2est.getA() + l2est.getC()) / l2est.getB();
                } else {
                    inhomY = Double.MAX_VALUE;
                }

                // if point lies outside retinal plane limits, try setting random
                // Vertical component within provided disparity
                if ((inhomY < minY) || (inhomY > maxY)) {
                    inhomY = m1.getInhomY() + randomizer.nextDouble(minVerticalDisparity, maxVerticalDisparity);
                    inhomX = -(inhomY * l2est.getB() + l2est.getC()) / l2est.getA();
                }

                m2.setInhomogeneousCoordinates(inhomX, inhomY);
                m2.normalize();

                // estimated epipolar line for some random point m1
                otherFundamentalMatrix.rightEpipolarLine(m1, l2real);
                l2real.normalize();

                // compute distance from l2real to m2 (distance from estimated to
                // real)
                d2prime = Math.abs(l2real.signedDistance(m2));

                otherFundamentalMatrix.leftEpipolarLine(m2, l1real);
                l1real.normalize();
                d2 = Math.abs(l1real.signedDistance(m1));

                avgDist += (d1 + d1prime + d2 + d2prime) / 4.0;

                if (currentIter > maxIterations) {
                    throw new FundamentalMatrixComparatorException();
                }

                progress = (float) currentIter / (float) maxIterations;

                if (listener != null && progress - previousProgress > progressDelta) {
                    previousProgress = progress;
                    listener.onCompareProgressChange(this, progress);
                }
            }


            if (listener != null) {
                listener.onCompareEnd(this);
            }

            return avgDist / nSamples;

        } finally {
            locked = false;
        }
    }

    /**
     * Returns type of comparator.
     *
     * @return type of comparator.
     */
    @Override
    public FundamentalMatrixComparatorType getType() {
        return FundamentalMatrixComparatorType.EPIPOLAR_DISTANCE_COMPARATOR;
    }

    /**
     * Initializes default settings.
     */
    private void init() {
        minX = DEFAULT_MIN_X;
        maxX = DEFAULT_MAX_X;
        minY = DEFAULT_MIN_Y;
        maxY = DEFAULT_MAX_Y;
        nSamples = DEFAULT_N_SAMPLES;
        minHorizontalDisparityFactor = minVerticalDisparityFactor = DEFAULT_MIN_DISPARITY_FACTOR;
        maxHorizontalDisparityFactor = maxVerticalDisparityFactor = DEFAULT_MAX_DISPARITY_FACTOR;
        maxIterationsFactor = DEFAULT_MAX_ITERATIONS_FACTOR;
        progressDelta = DEFAULT_PROGRESS_DELTA;
    }
}
