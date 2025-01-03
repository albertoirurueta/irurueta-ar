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
package com.irurueta.ar.epipolar.estimators;

import com.irurueta.geometry.EuclideanTransformation3D;

/**
 * Contains decomposition data of an homography.
 */
public class HomographyDecomposition {

    /**
     * Length of plane normal vector.
     */
    public static final int PLANE_NORMAL_LENGTH = 3;

    /**
     * Contains rotation and translation obtained from homography decomposition.
     */
    private EuclideanTransformation3D transformation;

    /**
     * Contains computed plane normal obtained from homography decomposition.
     */
    private double[] planeNormal;

    /**
     * Distance to computed plane.
     */
    private double planeDistance;

    /**
     * Constructor.
     */
    public HomographyDecomposition() {
    }

    /**
     * Constructor.
     *
     * @param transformation transformation containing rotation and translation
     *                       obtained from homography decomposition.
     * @param planeNormal    computed plane normal obtained from homography
     *                       decomposition.
     * @param planeDistance  distance to computed plane.
     * @throws IllegalArgumentException if provided plane normal does not have
     *                                  length 3.
     */
    public HomographyDecomposition(final EuclideanTransformation3D transformation, final double[] planeNormal,
                                   final double planeDistance) {
        setPlaneNormal(planeNormal);
        this.transformation = transformation;
        this.planeDistance = planeDistance;
    }

    /**
     * Gets rotation and translation obtained from homography decomposition.
     *
     * @return rotation and translation obtained from homography decomposition.
     */
    public EuclideanTransformation3D getTransformation() {
        return transformation;
    }

    /**
     * Sets rotation and translation obtained from homography decomposition.
     *
     * @param transformation rotation and translation obtained from homography
     *                       decomposition.
     */
    public void setTransformation(final EuclideanTransformation3D transformation) {
        this.transformation = transformation;
    }

    /**
     * Gets computed plane normal obtained from homography decomposition.
     *
     * @return plane normal obtained from homography decomposition.
     */
    public double[] getPlaneNormal() {
        return planeNormal;
    }

    /**
     * Sets computed plane normal obtained from homography decomposition.
     *
     * @param planeNormal plane normal obtained from homography decomposition.
     * @throws IllegalArgumentException if provided plane normal does not have
     *                                  length 3.
     */
    public final void setPlaneNormal(final double[] planeNormal) {
        if (planeNormal.length != PLANE_NORMAL_LENGTH) {
            throw new IllegalArgumentException();
        }
        this.planeNormal = planeNormal;
    }

    /**
     * Gets distance to computed plane.
     *
     * @return distance to computed plane.
     */
    public double getPlaneDistance() {
        return planeDistance;
    }

    /**
     * Sets distance to computed plane.
     *
     * @param planeDistance distance to computed plane.
     */
    public void setPlaneDistance(final double planeDistance) {
        this.planeDistance = planeDistance;
    }
}
