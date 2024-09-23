#ifndef ERRORS_H
#define ERRORS_H


/*
 * @brief Example template.
 *
 * See implementation file for information about this module.
 *
 * MIT License
 * 
 * Copyright (c) 2024 Federico Osti
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */



////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

// Error codes.
#define MOD_RET_OK                   0
// Motion planning
#define MP_ERR_BAD_STATUS           -1
#define MP_ERR_END_OF_TRAJECTORY    -1
#define MP_ERR_BAD_ARGUMENT         -1
#define MP_ERR_BAD_TRAVEL_TIME      -1
#define MP_ERR_BAD_VIA_POINT        -1
#define MP_ERR_BAD_TRAJECTORY       -1
#define MP_ERR_WS_LIMIT             -1
#define MP_ERR_JOINT_LIMIT          -1
// Buffer
#define BFR_ERR_EMPTY               -1
#define BFR_ERR_FULL                -2
#define BFR_ERR_BUSY                -3
// Robot
#define RBT_ERR_SB                  -3


////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Public (global) externs
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Public (global) function declarations
////////////////////////////////////////////////////////////////////////////////


#endif // ERRORS_H