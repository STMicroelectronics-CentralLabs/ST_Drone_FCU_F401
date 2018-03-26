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

public class SettingParameter implements Parcelable {
    private Boolean mFollowerCursor;
    private Boolean mYawDisalble;
    private int mSwapCommandcode;
    private Boolean mDataDroneVisible;
    private Boolean mLogFile;
    private Boolean mShapeCircle;

//    public SettingParameter(Boolean mFollowerCursor, Boolean mYawDisalble, int mSwapCommandcode, Boolean mDataDroneVisible, Boolean mLogFile, Boolean mShapeCircle
//    ) {
//        this.mFollowerCursor = mFollowerCursor;
//        this.mYawDisalble = mYawDisalble;
//        this.mSwapCommandcode = mSwapCommandcode;
//        this.mDataDroneVisible = mDataDroneVisible;
//        this.mLogFile = mLogFile;
//        this.mShapeCircle = mShapeCircle;
//    }

    protected SettingParameter(Parcel in) {
        mFollowerCursor = in.readByte() != 0;
        mYawDisalble = in.readByte() != 0;
        mSwapCommandcode = in.readInt();
        mDataDroneVisible = in.readByte() != 0;
        mLogFile = in.readByte() != 0;
        mShapeCircle = in.readByte() != 0;
    }

    public static final Creator<SettingParameter> CREATOR = new Creator<SettingParameter>() {
        @Override
        public SettingParameter createFromParcel(Parcel in) {
            return new SettingParameter(in);
        }

        @Override
        public SettingParameter[] newArray(int size) {
            return new SettingParameter[size];
        }
    };

    @Override
    public String toString() {
        return "SettingParameter{" +
                ", mFollowerCursor=" + mFollowerCursor +
                ", mYawDisalble=" + mYawDisalble +
                ", mSwapCommandcode=" + mSwapCommandcode +
                ", mDataDroneVisible=" + mDataDroneVisible +
                ", mLogFile=" + mLogFile +
                ", mShapeCircle=" + mShapeCircle +
                '}';
    }

/*
    public boolean ismShapeCircle() {
        return mShapeCircle;
    }

    public void setmShapeCircle(boolean mShapeCircle) {
        this.mShapeCircle = mShapeCircle;
    }

    public boolean ismFollowerCursor() {
        return mFollowerCursor;
    }

    public void setmFollowerCursor(boolean mFollowerCursor) {
        this.mFollowerCursor = mFollowerCursor;
    }

    public boolean ismYawDisalble() {
        return mYawDisalble;
    }

    public void setmYawDisalble(boolean mYawDisalble) {
        this.mYawDisalble = mYawDisalble;
    }

    public int getmSwapCommandcode() {
        return mSwapCommandcode;
    }

    public void setmSwapCommandcode(int mSwapCommandcode) {
        this.mSwapCommandcode = mSwapCommandcode;
    }

    public boolean ismDataDroneVisible() {
        return mDataDroneVisible;
    }

    public void setmDataDroneVisible(boolean mDataDroneVisible) {
        this.mDataDroneVisible = mDataDroneVisible;
    }

    public boolean ismLogFile() {
        return mLogFile;
    }

    public void setmLogFile(boolean mLogFile) {
        this.mLogFile = mLogFile;
    }
*/


    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel parcel, int i) {
        parcel.writeByte((byte) (mFollowerCursor ? 1 : 0));
        parcel.writeByte((byte) (mYawDisalble ? 1 : 0));
        parcel.writeInt((int) mSwapCommandcode);
        parcel.writeByte((byte) (mDataDroneVisible ? 1 : 0));
        parcel.writeByte((byte) (mLogFile ? 1 : 0));
        parcel.writeByte((byte) (mShapeCircle ? 1 : 0));
    }
}
