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
import com.irurueta.geometry.Point2D;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Contains coordinates of ideal points for a circle pattern.
 */
public class CirclesPattern2D extends Pattern2D implements Serializable {

    /**
     * Default point separation in the circles pattern.
     */
    public static final double DEFAULT_POINT_SEPARATION = 4.25e-2; //4.25cm

    /**
     * Default number of columns in the circles pattern.
     */
    public static final int DEFAULT_COLS = 4;

    /**
     * Default number of rows in the circles pattern.
     */
    public static final int DEFAULT_ROWS = 11;

    /**
     * Default number of points used by this 2D pattern.
     */
    public static final int DEFAULT_NUMBER_OF_POINTS = DEFAULT_COLS * DEFAULT_ROWS;

    /**
     * Minimum number of columns.
     */
    public static final int MIN_COLS = 2;

    /**
     * Minimum number of rows.
     */
    public static final int MIN_ROWS = 2;


    /**
     * Point separation in the circles pattern.
     */
    private double pointSeparation;

    /**
     * Number of columns in the circles pattern.
     */
    private int cols;

    /**
     * Number of rows in the circles pattern.
     */
    private int rows;

    /**
     * Constructor.
     */
    public CirclesPattern2D() {
        pointSeparation = DEFAULT_POINT_SEPARATION;
        cols = DEFAULT_COLS;
        rows = DEFAULT_ROWS;
    }

    /**
     * Returns point separation in the circles pattern.
     *
     * @return point separation in the circles pattern.
     */
    public double getPointSeparation() {
        return pointSeparation;
    }

    /**
     * Sets point separation in the circles pattern.
     *
     * @param pointSeparation point separation in the circles pattern.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setPointSeparation(final double pointSeparation) {
        if (pointSeparation <= 0.0) {
            throw new IllegalArgumentException();
        }

        this.pointSeparation = pointSeparation;
    }

    /**
     * Returns number of columns in the circles pattern.
     *
     * @return number of columns in the circles pattern.
     */
    public int getCols() {
        return cols;
    }

    /**
     * Sets number of columns in the circles pattern.
     *
     * @param cols number of columns in the circles pattern.
     * @throws IllegalArgumentException if provided value is less than 2.
     */
    public void setCols(final int cols) {
        if (cols < MIN_COLS) {
            throw new IllegalArgumentException();
        }

        this.cols = cols;
    }

    /**
     * Returns number of rows in the circles pattern.
     *
     * @return number of rows in the circles pattern.
     */
    public int getRows() {
        return rows;
    }

    /**
     * Sets number of rows in the circles pattern.
     *
     * @param rows number of rows in the circles pattern.
     * @throws IllegalArgumentException if provided value is less than 2.
     */
    public void setRows(final int rows) {
        if (rows < MIN_ROWS) {
            throw new IllegalArgumentException();
        }

        this.rows = rows;
    }

    /**
     * Returns ideal points coordinates contained in a circles 2D pattern and
     * expressed in meters. These values are used for calibration purposes.
     *
     * @return ideal points coordinates.
     */
    @Override
    public List<Point2D> getIdealPoints() {
        final var points = new ArrayList<Point2D>();

        double x;
        double y;
        for (var i = 0; i < rows; i++) {
            for (var j = 0; j < cols; j++) {
                x = (2 * j + i % 2) * pointSeparation;
                y = i * pointSeparation;
                points.add(new InhomogeneousPoint2D(x, y));
            }
        }
        return points;
    }

    /**
     * Returns number of 2D points used by this pattern.
     *
     * @return number of 2D points used by this pattern.
     */
    @Override
    public int getNumberOfPoints() {
        return rows * cols;
    }

    /**
     * Returns type of pattern.
     *
     * @return type of pattern.
     */
    @Override
    public Pattern2DType getType() {
        return Pattern2DType.CIRCLES;
    }
}
