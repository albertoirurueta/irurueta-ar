/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.ar;

import org.junit.Test;

import static org.junit.Assert.assertNotNull;

public class ARExceptionTest {

    @Test
    public void testConstructor() {
        ARException ex = new ARException();
        assertNotNull(ex);

        ex = new ARException("message");
        assertNotNull(ex);

        ex = new ARException(new Exception());
        assertNotNull(ex);

        ex = new ARException("message", new Exception());
        assertNotNull(ex);
    }
}
