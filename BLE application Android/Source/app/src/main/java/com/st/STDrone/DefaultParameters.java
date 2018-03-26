/*
 * Copyright (c) 2018  STMicroelectronics – All rights reserved
 * The STMicroelectronics corporate logo is a trademark of STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this list of conditions
 *    and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * - Neither the name nor trademarks of STMicroelectronics International N.V. nor any other
 *    STMicroelectronics company nor the names of its contributors may be used to endorse or
 *    promote products derived from this software without specific prior written permission.
 *
 * - All of the icons, pictures, logos and other images that are provided with the source code
 *    in a directory whose title begins with st_images may only be used for internal purposes and
 *    shall not be redistributed to any third party or modified in any way.
 *
 * - Any redistributions in binary form shall not include the capability to display any of the
 *    icons, pictures, logos and other images that are provided with the source code in a directory
 *    whose title begins with st_images.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */

package com.st.STDrone;


/**
 * Created by ST on 7/13/2017.
 */

public class DefaultParameters{

    public static float mCoeff_a_Yaw_default;
    public static float mCoeff_b_Yaw_default;
    public static float mCoeff_c_Yaw_default;
    public static float mRatio_r_Yaw_default;
    public static float mFactor_f_Yaw_default;

    public static float mCoeff_a_y_default;
    public static float mCoeff_b_y_default;
    public static float mCoeff_c_y_default;
    public static float mRatio_r_y_default;
    public static float mFactor_f_y_default;

    public static float mCoeff_a_xz_default;
    public static float mCoeff_b_xz_default;
    public static float mCoeff_c_xz_default;
    public static float mRatio_r_xz_default;
    public static float mFactor_f_xz_default;

    private final static int PLANE_XY = 0;
    private final static int PLANE_XZ = 1;
    private final static int PLANE_ZY = 2;

    public static int mSwapCommandcode_default = PLANE_XZ; // default

    public static Boolean mFollowerCursor_default = false;
    public static Boolean mYawDisalble_default = false;
    public static Boolean mDataDroneVisible_default = false;
    public static Boolean mLogFile_default = false;
    public static Boolean mShapeCircle_default = true;


    public static void BuildDefaultParameters() {

        mSwapCommandcode_default = PLANE_XZ; // default

        mFollowerCursor_default = false;
        mYawDisalble_default = false;
        mDataDroneVisible_default = false;
        mLogFile_default = false;
        mShapeCircle_default = true;

        mFactor_f_Yaw_default = 10; // ratio = slope rate/ slope rate linear at first value (zero or center analog)
        mFactor_f_y_default = 10; // ratio = slope rate/ slope rate linear at first value (zero or center analog)
        mFactor_f_xz_default = 10; // ratio = slope rate/ slope rate linear at first value (zero or center analog)
        NonLinearCoeffBuilding(0,-1);
        NonLinearCoeffBuilding(0,0);
        NonLinearCoeffBuilding(30,1); // to have 3.7 ratio
    }

    static void NonLinearCoeffBuilding(int seekValue, int axis_selector)
    {
        float factor_f,ratio_r,coeff_a,coeff_b,coeff_c;
        float s = (float) (seekValue);
        factor_f = 0;
        switch(axis_selector) {
            case -1:
                factor_f = mFactor_f_Yaw_default;
                break;
            case 0:
                factor_f = mFactor_f_y_default;
                break;
            case 1:
                factor_f = mFactor_f_xz_default;
                break;
        }
        ratio_r = 1.0f + s * 0.01f * (factor_f-1); // mRatio_r = ratio between slope at starting point non linear and linear  = [1 ... f], s = [0 ... 100]
        coeff_a = (ratio_r-1)*(factor_f-1)/((factor_f-2)*(factor_f-2));
        coeff_b = 1/(factor_f-2)-coeff_a/(factor_f-1);
        coeff_c = coeff_a*factor_f/(factor_f-1)-1/(factor_f-2);
        switch(axis_selector) {
            case -1:
                mRatio_r_Yaw_default = ratio_r;
                mCoeff_a_Yaw_default = coeff_a;
                mCoeff_b_Yaw_default = coeff_b;
                mCoeff_c_Yaw_default = coeff_c;
                break;
            case 0:
                mRatio_r_y_default = ratio_r;
                mCoeff_a_y_default = coeff_a;
                mCoeff_b_y_default = coeff_b;
                mCoeff_c_y_default = coeff_c;
                break;
            case 1:
                mRatio_r_xz_default = ratio_r;
                mCoeff_a_xz_default = coeff_a;
                mCoeff_b_xz_default = coeff_b;
                mCoeff_c_xz_default = coeff_c;
                break;
        }
    }
}



