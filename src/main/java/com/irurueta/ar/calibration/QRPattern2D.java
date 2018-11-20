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
 * Contains coordinates of ideal points for a QR code pattern version 2.
 * When physical size of a 2D QR code is known, its corner markets can be
 * used for camera calibration purposes.
 * This pattern takes into account that a QR code has 3 finder patterns and
 * 1 alignment pattern.
 * The points returned by this pattern indicates where should these points be
 * placed for a QR code having provided size.
 * Points are returned in the following order:
 * - Bottom-left finder pattern.
 * - Top-left finder pattern (located at origin of coordinates 0,0).
 * - Top-right finder pattern.
 * - Bottom-right alignment pattern.
 * 
 * The contents of this class are based on:
 * http://www.thonky.com/qr-code-tutorial/module-placement-matrix/
 */
@SuppressWarnings("WeakerAccess")
public class QRPattern2D extends Pattern2D implements Serializable {
    
    /**
     * Returns number of points used by this 2D pattern.
     */
    public static final int NUMBER_OF_POINTS = 4;
    
    /**
     * Supported QR code version.
     */
    public static final int QR_VERSION = 2;
    
    /**
     * Offset of origin expressed in modules so that top-left finder pattern
     * is placed at 0,0.
     */
    public static final int ORIGIN_OFFSET = 4;
    
    /**
     * Default QR code width expressed in meters.
     * This value is used to obtain a reference physical measure.
     */
    public static final double DEFAULT_QR_CODE_WIDTH = 1.1e-2; //1.1 cm
    
    /**
     * Default QR code height expressed in meters.
     * This value is used to obtain a reference physical measure.
     */
    public static final double DEFAULT_QR_CODE_HEIGHT = 1.1e-2; //1.1 cm
    
    /**
     * QR code width expressed in meters.
     * This value is used to obtain a reference physical measure.
     */
    private double mCodeWidth;
    
    /**
     * QR code height expressed in meters.
     * This value is used to obtain a reference physical measure.
     */
    private double mCodeHeight;
    
    /**
     * Constructor.
     */
    public QRPattern2D() {
        mCodeWidth = DEFAULT_QR_CODE_WIDTH;
        mCodeHeight = DEFAULT_QR_CODE_HEIGHT;
    }
    
    /**
     * Returns QR code width expressed in meters.
     * This value is used to obtain a reference physical measure.
     * @return QR code width expressed in meters.
     */
    public double getCodeWidth() {
        return mCodeWidth;
    }
    
    /**
     * Sets QR code width expressed in meters.
     * This value is used to obtain a reference physical measure.
     * @param codeWidth QR code width expressed in meters.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setCodeWidth(double codeWidth) throws IllegalArgumentException {
        if (codeWidth <= 0.0) {
            throw new IllegalArgumentException();
        }
        
        mCodeWidth = codeWidth;
    }
    
    /**
     * Returns QR code height expressed in meters.
     * This value is used to obtain a reference physical measure.
     * @return QR code height expressed in meters.
     */
    public double getCodeHeight() {
        return mCodeHeight;
    }
    
    /**
     * Sets QR code height expressed in meters.
     * This value is used to obtain a reference physical measure.
     * @param codeHeight QR code width expressed in meters.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setCodeHeight(double codeHeight) 
            throws IllegalArgumentException {
        if (codeHeight <= 0.0) {
            throw new IllegalArgumentException();
        }
        
        mCodeHeight = codeHeight;
    }

    /**
     * Returns ideal points coordinates contained in a QR 2D pattern and 
     * expressed in meters. These values are used for calibration purposes
     * @return ideal points coordinates.
     */
    @Override
    public List<Point2D> getIdealPoints() {
        List<Point2D> points = new ArrayList<>();
        
        //The size of a QR code (expressed in modules, i.e. each small square) 
        //can be calculated with the fomula (((V-1)*4)+21), where V is the QR 
        //code version.
        //For version 2, then the size will be 25 modules by 25 modules, where
        //each module will be the smallest square that can be represented in the
        //QR code and whose size will be equal to mCodeWidth/25 per 
        //mCodeHeight/25 expressed in meters (since both mCodeWidth and 
        //mCodeHeight are expressed in meters)
        
        //Each finder pattern consists of an outer 7 modules by 7 modules outer
        //black square, with an inner white square being 5x5 and finally and 
        //inner black square which is 3x3.
        
        //Hence, assuming the top-left finder is at location 0,0 (which is 
        //indeed the module 3,3 assuming modules positions start at 1).
        //Then the top-right finder pattern is located at: 
        //([(((V-1)*4)+21) - 7], 0), where we have subtracted 7 to the QR code
        //width expressed in modules to account for the fact that finder modules
        //are centered at 3x3 (we subtract 4 modules for top-left plus 3 modules
        //for top-right). For version 2 this is equal to (18, 0)
        //Likewise, bottom-left module will be placed at 
        //(0, [(((V-1)*4)+21) - 7]). For version 2 this is equal to (0, 18)
        
        //The alignment patterns are 5x5 black squares containing a
        //3x3 white inner square, which contains a 1x2 black square
        
        //On QR version 2 alignment patterns are placed with a 6x6 module margin 
        //respect to QR code boundaries. Because bottom-left, top-left and 
        //top-right alignment patterns would overlap the finder patterns, 
        //version 2 only includes the bottom right alignment pattern, which
        //is located at ([(((V-1)*4)+21) - 7 - 3], [(((V-1)*4)+21) - 7 - 3]),
        //so for version 2 this is equal to (15, 15)

        //noinspection all
        int numModules = ((QR_VERSION - 1) * 4) + 21;
        double moduleWidth = mCodeWidth / (double)numModules;
        double moduleHeight = mCodeHeight / (double)numModules;
        
        //below is equivalent to [(((V-1)*4)+21) - 7] = 18
        //noinspection all
        int finderModulePos = ((QR_VERSION - 1) * 4) + 21 - 3 - ORIGIN_OFFSET;
        //below is always 15
        int alignModulePos = finderModulePos - 3;
        
        //bottom-left finder pattern
        points.add(new InhomogeneousPoint2D(0.0, 
                (double)finderModulePos * moduleHeight));        
        //top-left finder pattern
        points.add(new InhomogeneousPoint2D(0.0, 0.0));
        //top-right finder pattern
        points.add(new InhomogeneousPoint2D(
                (double)finderModulePos * moduleWidth, 0.0));
        
        //bottom-right alignment pattern
        points.add(new InhomogeneousPoint2D(
                (double)alignModulePos * moduleWidth, 
                (double)alignModulePos * moduleHeight));
        
        return points;
    }
    
    /**
     * Returns number of 2D points used by this pattern.
     * @return number of 2D points used by this pattern.
     */
    @Override
    public int getNumberOfPoints() {
        return NUMBER_OF_POINTS;
    }


    @Override
    public Pattern2DType getType() {
        return Pattern2DType.QR;
    }
}
