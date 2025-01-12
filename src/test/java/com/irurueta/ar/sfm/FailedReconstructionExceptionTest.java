/*
 * Copyright (C) 2016 Alberto Irurueta Carro (alberto@irurueta.com)
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

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertNotNull;

class FailedReconstructionExceptionTest {

    @Test
    void testConstructor() {
        var ex = new FailedReconstructionException();
        assertNotNull(ex);

        ex = new FailedReconstructionException("message");
        assertNotNull(ex);

        ex = new FailedReconstructionException("message", new Exception());
        assertNotNull(ex);

        ex = new FailedReconstructionException(new Exception());
        assertNotNull(ex);
    }
}
