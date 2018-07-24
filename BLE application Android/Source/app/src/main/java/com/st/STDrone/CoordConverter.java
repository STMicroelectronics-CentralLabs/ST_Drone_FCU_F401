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


import android.support.v7.app.AppCompatActivity;
import android.util.DisplayMetrics;

/**
 * Created by ST on 7/13/2017.
 */

public class CoordConverter  extends AppCompatActivity {

    public static int mWidthPixels;
    public static int mHeightPixels;
    public static float mDensityDpi;
    public static int mWidthDp;
    public static int mHeightDp;


    public static void init(int widthPixels,int heightPixels,float densityDpi){
        mWidthPixels  = widthPixels;
        mHeightPixels = heightPixels;
        mDensityDpi = densityDpi;
        mWidthDp  = fromPx_to_dp(mWidthPixels);
        mHeightDp  = fromPx_to_dp(mHeightPixels);
    }

    public static int fromDp_to_px(int dp) {
        int px = (int)(dp * (mDensityDpi / 160));
        return px;
    }

    public static int fromPx_to_dp(int px) {
        int dp = (int)(px*160/mDensityDpi);
        return dp;
    }

 }



