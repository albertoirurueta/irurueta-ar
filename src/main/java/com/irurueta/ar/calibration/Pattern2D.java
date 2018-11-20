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

import com.irurueta.geometry.Point2D;

import java.util.List;

/**
 * Abstract representation of the 2D samples contained in a pattern.
 * Implementations for each pattern type will return the ideal coordinates
 * of a pattern to be used for camera calibration.
 * Coordinates of points returned by a pattern will be measured values in
 * the real world (expressed in meters). Measures are ideal values not affected
 * by any kind of distortion and as precise as possible so they can be used for
 * camera calibration.
 */
public abstract class Pattern2D {

    /**
     * Returns ideal points coordinates contained in a pattern and expressed
     * in meters. These values are use for calibration purposes.
     * @return ideal points coordinates.
     */
    public abstract List<Point2D> getIdealPoints();
    
    /**
     * Returns number of 2D points used by this pattern.
     * @return number of 2D points used by this pattern.
     */
    public abstract int getNumberOfPoints();
    
    /**
     * Returns type of pattern.
     * @return type of pattern.
     */
    public abstract Pattern2DType getType();
    
    /**
     * Creates an instance of a 2D pattern using provided type.
     * @param type type of 2D pattern to create.
     * @return an instance of a 2D pattern.
     */
    public static Pattern2D create(Pattern2DType type) {
        switch (type) {
            case QR:
                return new QRPattern2D();
            case CIRCLES:
            default:
                return new CirclesPattern2D();
        }
    }
}
