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

import android.os.Parcel;
import android.os.Parcelable;

/**
 * Created by ST on 4/3/2017.
 */

public class SensitivityParameter implements Parcelable {

    private float mCoeff_a_Yaw;
    private float mCoeff_b_Yaw;
    private float mCoeff_c_Yaw;
    private float mRatio_r_Yaw;
    private float mFactor_f_Yaw;

    private float mCoeff_a_y;
    private float mCoeff_b_y;
    private float mCoeff_c_y;
    private float mRatio_r_y;
    private float mFactor_f_y;

    private float mCoeff_a_xz;
    private float mCoeff_b_xz;
    private float mCoeff_c_xz;
    private float mRatio_r_xz;
    private float mFactor_f_xz;

    public SensitivityParameter(
                            float mCoeff_a_Yaw, float mCoeff_b_Yaw,float mCoeff_c_Yaw, float mRatio_r_Yaw, float mFactor_f_Yaw,
                            float mCoeff_a_y, float mCoeff_b_y,float mCoeff_c_y, float mRatio_r_y, float mFactor_f_y,
                            float mCoeff_a_xz, float mCoeff_b_xz,float mCoeff_c_xz, float mRatio_r_xz, float mFactor_f_xz
    ) {
        this.mCoeff_a_Yaw = mCoeff_a_Yaw;
        this.mCoeff_b_Yaw = mCoeff_b_Yaw;
        this.mCoeff_c_Yaw = mCoeff_c_Yaw;
        this.mRatio_r_Yaw = mRatio_r_Yaw;
        this.mFactor_f_Yaw = mFactor_f_Yaw;
        this.mCoeff_a_y = mCoeff_a_y;
        this.mCoeff_b_y = mCoeff_b_y;
        this.mCoeff_c_y = mCoeff_c_y;
        this.mRatio_r_y = mRatio_r_y;
        this.mFactor_f_y = mFactor_f_y;
        this.mCoeff_a_xz = mCoeff_a_xz;
        this.mCoeff_b_xz = mCoeff_b_xz;
        this.mCoeff_c_xz = mCoeff_c_xz;
        this.mRatio_r_xz = mRatio_r_xz;
        this.mFactor_f_xz = mFactor_f_xz;
    }

    protected SensitivityParameter(Parcel in) {
        mCoeff_a_Yaw = in.readFloat();
        mCoeff_b_Yaw = in.readFloat();
        mCoeff_c_Yaw = in.readFloat();
        mRatio_r_Yaw = in.readFloat();
        mFactor_f_Yaw = in.readFloat();
        mCoeff_a_y = in.readFloat();
        mCoeff_b_y = in.readFloat();
        mCoeff_c_y = in.readFloat();
        mRatio_r_y = in.readFloat();
        mFactor_f_y = in.readFloat();
        mCoeff_a_xz = in.readFloat();
        mCoeff_b_xz = in.readFloat();
        mCoeff_c_xz = in.readFloat();
        mRatio_r_xz = in.readFloat();
        mFactor_f_xz = in.readFloat();
    }

    public static final Creator<SensitivityParameter> CREATOR = new Creator<SensitivityParameter>() {
        @Override
        public SensitivityParameter createFromParcel(Parcel in) {
            return new SensitivityParameter(in);
        }

        @Override
        public SensitivityParameter[] newArray(int size) {
            return new SensitivityParameter[size];
        }
    };

    @Override
    public String toString() {
        return "SensitivityParameter{" +
                ", mCoeff_a_Yaw=" + mCoeff_a_Yaw +
                ", mCoeff_b_Yaw=" + mCoeff_b_Yaw +
                ", mCoeff_c_Yaw=" + mCoeff_c_Yaw +
                ", mRatio_r_Yaw=" + mRatio_r_Yaw +
                ", mFactor_f_Yaw=" + mFactor_f_Yaw +
                ", mCoeff_a_y=" + mCoeff_a_y +
                ", mCoeff_b_y=" + mCoeff_b_y +
                ", mCoeff_c_y=" + mCoeff_c_y +
                ", mRatio_r_y=" + mRatio_r_y +
                ", mFactor_f_y=" + mFactor_f_y +
                ", mCoeff_a_xz=" + mCoeff_a_xz +
                ", mCoeff_b_xz=" + mCoeff_b_xz +
                ", mCoeff_c_xz=" + mCoeff_c_xz +
                ", mRatio_r_xz=" + mRatio_r_xz +
                ", mFactor_f_xz=" + mFactor_f_xz +
                '}';
    }

/*
    public float getmCoeff_a_Yaw() {
        return mCoeff_a_Yaw;
    }

    public void setmCoeff_a_Yaw(float mCoeff_a_Yaw) {
        this.mCoeff_a_Yaw = mCoeff_a_Yaw;
    }

    public float getmCoeff_b_Yaw() {
        return mCoeff_b_Yaw;
    }

    public void setmCoeff_b_Yaw(float mCoeff_b_Yaw) {
        this.mCoeff_b_Yaw = mCoeff_b_Yaw;
    }

    public float getmCoeff_c_Yaw() {
        return mCoeff_c_Yaw;
    }

    public void setmCoeff_c_Yaw(float mCoeff_c_Yaw) {
        this.mCoeff_c_Yaw = mCoeff_c_Yaw;
    }

    public float getmRatio_r_Yaw() {
        return mRatio_r_Yaw;
    }

    public void setmRatio_r_Yaw(float mRatio_r_Yaw) {
        this.mRatio_r_Yaw = mRatio_r_Yaw;
    }

    public float getmFactor_f_Yaw() {
        return mFactor_f_Yaw;
    }

    public void setmFactor_f_Yaw(float mFactor_f_Yaw) {
        this.mFactor_f_Yaw = mFactor_f_Yaw;
    }


    public float getmCoeff_a_y() {
        return mCoeff_a_y;
    }

    public void setmCoeff_a_y(float mCoeff_a_y) {
        this.mCoeff_a_y = mCoeff_a_y;
    }

    public float getmCoeff_b_y() {
        return mCoeff_b_y;
    }

    public void setmCoeff_b_y(float mCoeff_b_y) {
        this.mCoeff_b_y = mCoeff_b_y;
    }

    public float getmCoeff_c_y() {
        return mCoeff_c_y;
    }

    public void setmCoeff_c_y(float mCoeff_c_y) {
        this.mCoeff_c_y = mCoeff_c_y;
    }

    public float getmRatio_r_y() {
        return mRatio_r_y;
    }

    public void setmRatio_r_y(float mRatio_r_y) {
        this.mRatio_r_y = mRatio_r_y;
    }

    public float getmFactor_f_y() {
        return mFactor_f_y;
    }

    public void setmFactor_f_y(float mFactor_f_y) {
        this.mFactor_f_y = mFactor_f_y;
    }



    public float getmCoeff_a_xz() {
        return mCoeff_a_xz;
    }

    public void setmCoeff_a_xz(float mCoeff_a_xz) {
        this.mCoeff_a_xz = mCoeff_a_xz;
    }

    public float getmCoeff_b_xz() {
        return mCoeff_b_xz;
    }

    public void setmCoeff_b_xz(float mCoeff_b_xz) {
        this.mCoeff_b_xz = mCoeff_b_xz;
    }

    public float getmCoeff_c_xz() {
        return mCoeff_c_xz;
    }

    public void setmCoeff_c_xz(float mCoeff_c_xz) {
        this.mCoeff_c_xz = mCoeff_c_xz;
    }

    public float getmRatio_r_xz() {
        return mRatio_r_xz;
    }

    public void setmRatio_r_xz(float mRatio_r_xz) {
        this.mRatio_r_xz = mRatio_r_xz;
    }

    public float getmFactor_f_xz() {
        return mFactor_f_xz;
    }

    public void setmFactor_f_xz(float mFactor_f_xz) {
        this.mFactor_f_xz = mFactor_f_xz;
    }
    */


    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel parcel, int i) {
        parcel.writeFloat(mCoeff_a_Yaw);
        parcel.writeFloat(mCoeff_b_Yaw);
        parcel.writeFloat(mCoeff_c_Yaw);
        parcel.writeFloat(mRatio_r_Yaw);
        parcel.writeFloat(mFactor_f_Yaw);
        parcel.writeFloat(mCoeff_a_y);
        parcel.writeFloat(mCoeff_b_y);
        parcel.writeFloat(mCoeff_c_y);
        parcel.writeFloat(mRatio_r_y);
        parcel.writeFloat(mFactor_f_y);
        parcel.writeFloat(mCoeff_a_xz);
        parcel.writeFloat(mCoeff_b_xz);
        parcel.writeFloat(mCoeff_c_xz);
        parcel.writeFloat(mRatio_r_xz);
        parcel.writeFloat(mFactor_f_xz);
    }
}
