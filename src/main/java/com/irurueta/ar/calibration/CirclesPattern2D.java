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
@SuppressWarnings("WeakerAccess")
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
    public static final int DEFAULT_NUMBER_OF_POINTS = 
            DEFAULT_COLS * DEFAULT_ROWS;

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
    private double mPointSeparation;
    
    /**
     * Number of columns in the circles pattern.
     */
    private int mCols;
    
    /**
     * Number of rows in the circles pattern.
     */
    private int mRows;
    
    /**
     * Constructor.
     */
    public CirclesPattern2D() {
        mPointSeparation = DEFAULT_POINT_SEPARATION;
        mCols = DEFAULT_COLS;
        mRows = DEFAULT_ROWS;
    }
    
    /**
     * Returns point separation in the circles pattern.
     * @return point separation in the circles pattern.
     */
    public double getPointSeparation() {
        return mPointSeparation;
    }
    
    /**
     * Sets point separation in the circles pattern.
     * @param pointSeparation point separation in the circles pattern.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setPointSeparation(double pointSeparation)
            throws IllegalArgumentException {
        if (pointSeparation <= 0.0) {
            throw new IllegalArgumentException();
        }
        
        mPointSeparation = pointSeparation;
    }
    
    /**
     * Returns number of columns in the circles pattern.
     * @return number of columns in the circles pattern.
     */
    public int getCols() {
        return mCols;
    }
    
    /**
     * Sets number of columns in the circles pattern.
     * @param cols number of columns in the circles pattern.
     * @throws IllegalArgumentException if provided value is less than 2.
     */
    public void setCols(int cols) throws IllegalArgumentException {
        if (cols < MIN_COLS) {
            throw new IllegalArgumentException();
        }
        
        mCols = cols;
    }
    
    /**
     * Returns number of rows in the circles pattern.
     * @return number of rows in the circles pattern.
     */
    public int getRows() {
        return mRows;
    }
    
    /**
     * Sets number of rows in the circles pattern.
     * @param rows number of rows in the circles pattern.
     * @throws IllegalArgumentException if provided value is less than 2.
     */
    public void setRows(int rows) throws IllegalArgumentException {
        if (rows < MIN_ROWS) {
            throw new IllegalArgumentException();
        }
        
        mRows = rows;
    }
    
    /**
     * Returns ideal points coordinates contained in a circles 2D pattern and 
     * expressed in meters. These values are used for calibration purposes.
     * @return ideal points coordinates.
     */
    @Override
    public List<Point2D> getIdealPoints() {
        List<Point2D> points = new ArrayList<>();
        
        double x, y;
        for (int i = 0; i < mRows; i++) {
            for (int j = 0; j < mCols; j++) {
                x = (double)(2 * j + i % 2) * mPointSeparation;
                y = (double)i * mPointSeparation;
                points.add(new InhomogeneousPoint2D(x, y));
            }
        }
        return points;
    }    
    
    /**
     * Returns number of 2D points used by this pattern.
     * @return number of 2D points used by this pattern.
     */
    @Override
    public int getNumberOfPoints() {
        return mRows * mCols;
    }
    
    /**
     * Returns type of pattern.
     * @return type of pattern.
     */
    @Override
    public Pattern2DType getType() {
        return Pattern2DType.CIRCLES;
    }    
}
