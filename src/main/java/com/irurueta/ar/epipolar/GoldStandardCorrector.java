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
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.util.ArrayList;
import java.util.List;

/**
 * Fixes matched pairs of points so that they perfectly follow a given epipolar
 * geometry.
 * When matching points typically the matching precision is about 1 pixel,
 * however this makes that matched points under a given epipolar geometry (i.e.
 * fundamental or essential matrix), do not lie perfectly on the corresponding
 * epipolar plane or epipolar liens.
 * The consequence is that triangularization of these matches will fail or
 * produce inaccurate results.
 * By fixing matched points using a corrector following a given epipolar
 * geometry, this effect is alleviated.
 * This corrector uses the Gold Standard method, which is more expensive to
 * compute than the Sampson approximation, but is capable to remove larger
 * errors assuming their gaussianity. Contrary to the Sampson corrector, the
 * Gold Standard method might fail in some situations, while in those cases
 * probably the Sampson corrector produces wrong results without failing.
 */
public class GoldStandardCorrector extends Corrector {

    /**
     * Indicates that correction must use Sampson method as a fallback if Gold
     * Standard correction fails.
     */
    public static final boolean DEFAULT_FALLBACK_TO_SAMPSON_ENABLED = true;

    /**
     * Indicates whether correction must use Sampson method as a fallback if Gold
     * Standard correction fails.
     */
    private boolean fallbackToSampsonEnabled;

    /**
     * Constructor.
     */
    public GoldStandardCorrector() {
        super();
        fallbackToSampsonEnabled = DEFAULT_FALLBACK_TO_SAMPSON_ENABLED;
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix to be set.
     */
    public GoldStandardCorrector(final FundamentalMatrix fundamentalMatrix) {
        super(fundamentalMatrix);
        fallbackToSampsonEnabled = DEFAULT_FALLBACK_TO_SAMPSON_ENABLED;
    }

    /**
     * Constructor.
     *
     * @param leftPoints  points to be corrected on left view.
     * @param rightPoints points to be corrected on right view.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size.
     */
    public GoldStandardCorrector(final List<Point2D> leftPoints, final List<Point2D> rightPoints) {
        super(leftPoints, rightPoints);
        fallbackToSampsonEnabled = DEFAULT_FALLBACK_TO_SAMPSON_ENABLED;
    }

    /**
     * Constructor.
     *
     * @param leftPoints        points to be corrected on left view.
     * @param rightPoints       points to be corrected on right view.
     * @param fundamentalMatrix fundamental matrix to be set.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size.
     */
    public GoldStandardCorrector(final List<Point2D> leftPoints, final List<Point2D> rightPoints,
                                 final FundamentalMatrix fundamentalMatrix) {
        super(leftPoints, rightPoints, fundamentalMatrix);
        fallbackToSampsonEnabled = DEFAULT_FALLBACK_TO_SAMPSON_ENABLED;
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events generated by this class.
     */
    public GoldStandardCorrector(final CorrectorListener listener) {
        super(listener);
        fallbackToSampsonEnabled = DEFAULT_FALLBACK_TO_SAMPSON_ENABLED;
    }

    /**
     * Constructor.
     *
     * @param fundamentalMatrix fundamental matrix to be set.
     * @param listener          listener to handle events generated by this class.
     */
    public GoldStandardCorrector(final FundamentalMatrix fundamentalMatrix, final CorrectorListener listener) {
        super(fundamentalMatrix, listener);
        fallbackToSampsonEnabled = DEFAULT_FALLBACK_TO_SAMPSON_ENABLED;
    }

    /**
     * Constructor.
     *
     * @param leftPoints  points to be corrected on left view.
     * @param rightPoints points to be corrected on right view.
     * @param listener    listener to handle events generated by this class.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size.
     */
    public GoldStandardCorrector(final List<Point2D> leftPoints, final List<Point2D> rightPoints,
                                 final CorrectorListener listener) {
        super(leftPoints, rightPoints, listener);
        fallbackToSampsonEnabled = DEFAULT_FALLBACK_TO_SAMPSON_ENABLED;
    }

    /**
     * Constructor.
     *
     * @param leftPoints        points to be corrected on left view.
     * @param rightPoints       points to be corrected on right view.
     * @param fundamentalMatrix fundamental matrix to be set.
     * @param listener          listener to handle events generated by this class.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size.
     */
    public GoldStandardCorrector(
            final List<Point2D> leftPoints, final List<Point2D> rightPoints, final FundamentalMatrix fundamentalMatrix,
            final CorrectorListener listener) {
        super(leftPoints, rightPoints, fundamentalMatrix, listener);
        fallbackToSampsonEnabled = DEFAULT_FALLBACK_TO_SAMPSON_ENABLED;
    }

    /**
     * Indicates whether correction must use Sampson method as a fallback if Gold
     * Standard correction fails.
     * If true, whenever the correction of a matched pair of points fails,
     * its correction will be done with Sampson method instead. This allows
     * to continue correcting all points at the expense of getting worse
     * results in those points where Gold Standard method failed.
     * If false, whenever a single pair of matched pair of points fails to
     * be corrected, the algorithm stops and no points are corrected at all.
     * By default, fallback is enabled.
     *
     * @return true if fallback is enabled, false otherwise.
     */
    public boolean isFallbackToSampsonEnabled() {
        return fallbackToSampsonEnabled;
    }

    /**
     * Sets boolean indicating whether correction must use Sampson
     * method as a fallback if Gold Standard correction fails.
     * If true, whenever the correction of a matched pair of points fails,
     * its correction will be done with Sampson method instead. This allows
     * to continue correcting all points at the expense of getting worse
     * results in those points where Gold Standard method failed.
     * If false, whenever a single pair of matched pair of points fails to
     * be corrected, the algorithm stops and no points are corrected at all.
     * By default, fallback is enabled.
     *
     * @param fallback true to enable fallback, false otherwise.
     * @throws LockedException if this instance is locked doing computations.
     */
    public void setFallbackToSampsonEnabled(final boolean fallback) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        fallbackToSampsonEnabled = fallback;
    }

    /**
     * Corrects the lists of provided matched points to be corrected.
     *
     * @throws NotReadyException   if this instance is not ready (either points or
     *                             fundamental matrix has not been provided yet).
     * @throws LockedException     if this instance is locked doing computations.
     * @throws CorrectionException if correction fails.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public void correct() throws NotReadyException, LockedException, CorrectionException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        locked = true;

        if (listener != null) {
            listener.onCorrectStart(this);
        }

        leftCorrectedPoints = new ArrayList<>();
        rightCorrectedPoints = new ArrayList<>();

        final var size = leftPoints.size();
        float progress;
        var previousProgress = 0.0f;
        for (var i = 0; i < size; i++) {
            final var leftPoint = leftPoints.get(i);
            final var rightPoint = rightPoints.get(i);

            final var leftCorrectedPoint = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
            final var rightCorrectedPoint = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);

            // correct single pair
            try {
                GoldStandardSingleCorrector.correct(leftPoint, rightPoint, fundamentalMatrix, leftCorrectedPoint,
                        rightCorrectedPoint);
            } catch (final CorrectionException e) {
                if (fallbackToSampsonEnabled) {
                    // try using Sampson method instead
                    SampsonSingleCorrector.correct(leftPoint, rightPoint, fundamentalMatrix, leftCorrectedPoint,
                            rightCorrectedPoint);
                } else {
                    // let algorithm fail
                    throw e;
                }
            }

            leftCorrectedPoints.add(leftCorrectedPoint);
            rightCorrectedPoints.add(rightCorrectedPoint);

            if (listener != null) {
                progress = i / (float) size;
                if (progress - previousProgress > progressDelta) {
                    // progress has changed significantly
                    previousProgress = progress;
                    listener.onCorrectProgressChange(this, progress);
                }
            }
        }

        if (listener != null) {
            listener.onCorrectEnd(this);
        }

        locked = false;
    }

    /**
     * Gets type of correction being used.
     *
     * @return type of correction.
     */
    @Override
    public CorrectorType getType() {
        return CorrectorType.GOLD_STANDARD;
    }
}
