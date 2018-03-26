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

import com.st.BlueSTSDK.Feature;
import com.st.BlueSTSDK.Features.Field;
import com.st.BlueSTSDK.Node;

/**
 * Created by ST on 7/13/2017.
 */

public class FeatureJoystick extends Feature {

    //    [Transmit data format]
    //    The 1 nd byte: Digital Button
    // range values analog are 0-255, so each digital button pressure means +1 or -1 value

    //            (Right buttons)
    static int YPOS = 0x01;
    static int YNEG = 0x02;
    static int ROTYPOS = 0x03;
    static int ROTYNEG = 0x04;

    //            (Left buttons)
    static int ZPOS = 0x05;
    static int ZNEG = 0x06;
    static int XPOS = 0x07;
    static int XNEG = 0x08;

    //    The 2 - 3th byte: Left analog Stick
    //　            Left - Right: 0-255 (Neutral=128)
    //            　Up - Down: 0-255 (Neutral=0)
    //              only the byte value different from (Neutral+own offset) is active, because multiple analog bytes are sequentially sent one at time
    //
    //    The 4 - 5th byte: Right analog Stick
    //　            Left - Right: 0-255 (Neutral=128)
    //            　Up - Down: 0-255 (Neutral=128)
    //              only the byte value different from (Neutral+own offset) is active, because multiple analog bytes are sequentially sent one at time
    //
    //    The 6th byte: Accelerator
    //　            SeekBar value: 1-100 (Neutral=50)
    //
    //    The 7th byte: Status information
    //　             bit 0 : Takeoff (0-1)
    //　             bit 1 : Calib (0-1)
    //　             bit 2 : Arm (0-1)
    //　             bit 3 : Left offset (0-1)
    //　             bit 4 : Right offset (0-1)

    // first_index_bit
        //    The 7th byte: Status information
        //　             bit 0 : Switch1 (0-1)
        //　             bit 1-2 : Switch2 (0-2)
        //　             bit 3 : Switch3 (0-1)
        //　             bit 4 : Left offset (0-1)
        //　             bit 5 : Right offset (0-1)
    //private static int Switch1_first_index_bit = 0;
    //private static int Switch2_first_index_bit = 1;
    //private static int Switch3_first_index_bit = 3;
//    private static int Left_offset_first_index_bit = 4;
//    private static int Right_offset_first_index_bit = 5;
    private static int Takeoff_first_index_bit = 0;
    private static int Calib_first_index_bit = 1;
    private static int Arm_first_index_bit = 2;
    private static int Left_offset_first_index_bit = 3;
    private static int Right_offset_first_index_bit = 4;
    // mask
//    public static int Switch1_Mask = 0xFF - (1 << Switch1_first_index_bit); // 0 = off,  1 = on
//    public static int Switch2_Mask = 0xFF - (0x03 << Switch2_first_index_bit); // 0 = off,  1 = on
//    public static int Switch3_Mask = 0xFF - (1 << Switch3_first_index_bit); // 0 = off,  1 = on
    static int Takeoff_Mask = 0xFF - (1 << Takeoff_first_index_bit); // 0 = off,  1 = on
    static int Calib_Mask = 0xFF - (1 << Calib_first_index_bit); // When it changes status is active
    static int Arm_Mask = 0xFF - (1 << Arm_first_index_bit); // 0 = off,  1 = on
    static int Left_offset_Mask = 0xFF - (1 << Left_offset_first_index_bit); // 0 = Left_analog,  1 = offset rotY + axis Y
    static int Right_offset_Mask = 0xFF - (1 << Right_offset_first_index_bit);// 0 = Right_analog,  1 = offset axis Z + axis X
    // value
//    public static int Switch1_OFF = 0;
//    public static int Switch1_ON = (1 << Switch1_first_index_bit);
//    public static int Switch2_ZERO = 0;
//    public static int Switch2_NEG = (1 << Switch2_first_index_bit);
//    public static int Switch2_POS = (2 << Switch2_first_index_bit);
//    public static int Switch3_OFF = 0;
//    public static int Switch3_ON = (1 << Switch3_first_index_bit);
    static int Takeoff_OFF = 0;
    static int Takeoff_ON = (1 << Takeoff_first_index_bit);
    static int Calib_OFF = 0;
    static int Calib_ON = (1 << Calib_first_index_bit);
    static int Arm_OFF = 0;
    static int Arm_ON = (1 << Arm_first_index_bit);
    static int Left_offset_OFF = 0;
    static int Left_offset_ON = (1 << Left_offset_first_index_bit);
    static int Right_offset_OFF = 0;
    static int Right_offset_ON = (1 << Right_offset_first_index_bit);

    private int mDataStatus = 0;
    private int mSeekBarValue = 50;
    private int mAxisY = 0;
    private int mOffsetRotY = 0; // default
    private int mOffsetAxisX = 0; // default
    private int mOffsetAxisY = 0; // default
    private int mOffsetAxisZ = 0; // default

    private static int numWriteLeft = 0;
    private static int numWriteRight = 0;
    private boolean mEnableWriteBLEQueque = true;
    //private boolean isEmptyLeft = false;
    //private boolean isEmptyRight = false;


    /** feature name */
    private static final String FEATURE_NAME = "Joystick";

    private static int DIGITAL_BUTTOM_INDEX=0;
    private static final Field DIGITAL_BUTTOM_FIELD = new Field(
            "Digital Buttom","cmdCode", Field.Type.UInt8,0,1<<8);

    static final int ROTY_ANALOG_INDEX=1;
    private static final Field ROTY_ANALOG_FIELD = new Field(
            "Left analog Left - Right","Unit", Field.Type.UInt8,0,1<<8);

    static final int YAXIS_ANALOG_INDEX=2;
    private static final Field YAXIS_ANALOG_FIELD = new Field(
            "Left analog Up - Down","Unit", Field.Type.UInt8,0,1<<8);

    static final int XAXIS_ANALOG_INDEX=3;
    private static final Field XAXIS_ANALOG_FIELD = new Field(
            "Right analog Left - Right","Unit", Field.Type.UInt8,0,1<<8);

    static final int ZAXIS_ANALOG_INDEX=4;
    private static final Field ZAXIS_ANALOG_FIELD = new Field(
            "Right analog Up - Down","Unit", Field.Type.UInt8,0,1<<8);

    private static final int SEEKBAR_INDEX=5;
    private static final Field SEEKBAR_FIELD = new Field(
            "Right analog Up - Down","Unit", Field.Type.UInt8,0,1<<8);

    private static final int STATUS_INDEX=6;
    private static final Field STATUS_FIELD = new Field(
            "Status information","ByteMask", Field.Type.UInt8,0,1<<8);

    private byte[] initData(){

        byte[] data = new byte[STATUS_INDEX+1];
        //    The 1nd byte: Button
        data[DIGITAL_BUTTOM_INDEX] = 0;
         //    The 2 - 3th byte: Left analog Stick
        data[ROTY_ANALOG_INDEX] = (byte) (128+mOffsetRotY);
        data[YAXIS_ANALOG_INDEX] = (byte) (mAxisY+mOffsetAxisY);
        //    The 4 - 5th byte: Right analog Stick
        data[XAXIS_ANALOG_INDEX] = (byte) (128+mOffsetAxisX);
        data[ZAXIS_ANALOG_INDEX] = (byte) (128+mOffsetAxisZ);
        //    The 6th byte: SeekBar value
        data[SEEKBAR_INDEX] = (byte) mSeekBarValue;
        //    The 7th byte: Status
        data[STATUS_INDEX] = (byte) mDataStatus;

        return data;
    }

    /**
     * build a Joystick feature
     * @param node node where the feature will write the data
     */
    public FeatureJoystick(Node node) {
        super(FEATURE_NAME, node, new Field[]{
                DIGITAL_BUTTOM_FIELD,
                ROTY_ANALOG_FIELD,
                YAXIS_ANALOG_FIELD,
                XAXIS_ANALOG_FIELD,
                ZAXIS_ANALOG_FIELD,
                SEEKBAR_FIELD,
                STATUS_FIELD
        });

        //    The 7th byte: Status information
        mDataStatus = 0;
        mSeekBarValue = 50;
        mAxisY = 0;
    }//FeatureJoystick


    void writeDigitalButtomJoystick(int value) {
        byte[] data = initData();
        data[DIGITAL_BUTTOM_INDEX] = (byte)(value & 0xff);
        writeData(data);
    }

    void writeAnalogY(int index1, int value1, Node node, boolean isLeft) {

        value1 += mOffsetAxisY;
        if(value1 > 255)
            value1 = 255;
        if(value1 < 0)
            value1 = 0;
        mAxisY = value1-mOffsetAxisY;
        byte[] data = initData();
        data[index1] = (byte)(value1 & 0xff);

        boolean isEmpty = node.writeQueueIsEmpty();
        if(isEmpty)
        {
            numWriteLeft = 0;
            numWriteRight = 0;
        }
        if(isLeft)
        {
            isEmpty = isEmpty || ((numWriteRight == 1)&&(numWriteLeft == 0));
            mEnableWriteBLEQueque = mEnableWriteBLEQueque || ((numWriteRight == 1)&&(numWriteLeft == 0));
        }
        else
        {
            isEmpty = isEmpty || ((numWriteLeft == 1)&&(numWriteRight == 0));
            mEnableWriteBLEQueque = mEnableWriteBLEQueque || ((numWriteLeft == 1)&&(numWriteRight == 0));
        }

        if(isEmpty && mEnableWriteBLEQueque) // safer: the slowest win
        {
            writeData(data);
            if(isLeft)
            {
                numWriteLeft++;
            }
            else
            {
                numWriteRight++;
            }
            mEnableWriteBLEQueque = false;
        }
    }

    void writeAnalogJoystickDouble(int index1, int value1, int index2, int value2, Node node, boolean isLeft) {

        switch(index1)
        {
            case ROTY_ANALOG_INDEX:
                value1 += mOffsetRotY;
                break;
            case YAXIS_ANALOG_INDEX:
                value1 += mOffsetAxisY;
                break;
            case XAXIS_ANALOG_INDEX:
                value1 += mOffsetAxisX;
                break;
            case ZAXIS_ANALOG_INDEX:
                value1 += mOffsetAxisZ;
                break;
        }
        if(value1 > 255)
            value1 = 255;
        if(value1 < 0)
            value1 = 0;
        if(index1 == YAXIS_ANALOG_INDEX)
        {
            mAxisY = value1-mOffsetAxisY;
        }
        switch(index2)
        {
            case ROTY_ANALOG_INDEX:
                value2 += mOffsetRotY;
                break;
            case YAXIS_ANALOG_INDEX:
                value2 += mOffsetAxisY;
                break;
            case XAXIS_ANALOG_INDEX:
                value2 += mOffsetAxisX;
                break;
            case ZAXIS_ANALOG_INDEX:
                value2 += mOffsetAxisZ;
                break;
        }
        if(value2 > 255)
            value2 = 255;
        if(value2 < 0)
            value2 = 0;
        if(index2 == YAXIS_ANALOG_INDEX)
        {
            mAxisY = value2-mOffsetAxisY;
        }
        byte[] data = initData();
        data[index1] = (byte)(value1 & 0xff);
        data[index2] = (byte)(value2 & 0xff);

        boolean isEmpty = node.writeQueueIsEmpty();
        if(isEmpty)
        {
            numWriteLeft = 0;
            numWriteRight = 0;
        }
        if(isLeft)
        {
            isEmpty = isEmpty || ((numWriteRight == 1)&&(numWriteLeft == 0));
            mEnableWriteBLEQueque = mEnableWriteBLEQueque || ((numWriteRight == 1)&&(numWriteLeft == 0));
        }
        else
        {
            isEmpty = isEmpty || ((numWriteLeft == 1)&&(numWriteRight == 0));
            mEnableWriteBLEQueque = mEnableWriteBLEQueque || ((numWriteLeft == 1)&&(numWriteRight == 0));
        }

        if(isEmpty && mEnableWriteBLEQueque) // safer: the slowest win
        {
             writeData(data);
            if(isLeft)
            {
                numWriteLeft++;
            }
            else
            {
                numWriteRight++;
            }
            mEnableWriteBLEQueque = false;
        }

      }


    void writeAnalogJoystickFull(int index1, int value1, int index2, int value2, int index3, int value3, int index4, int value4, Node node) {

        switch(index1)
        {
            case ROTY_ANALOG_INDEX:
                value1 += mOffsetRotY;
                break;
            case YAXIS_ANALOG_INDEX:
                value1 += mOffsetAxisY;
                break;
            case XAXIS_ANALOG_INDEX:
                value1 += mOffsetAxisX;
                break;
            case ZAXIS_ANALOG_INDEX:
                value1 += mOffsetAxisZ;
                break;
        }
        if(value1 > 255)
            value1 = 255;
        if(value1 < 0)
            value1 = 0;
        if(index1 == YAXIS_ANALOG_INDEX)
        {
            mAxisY = value1-mOffsetAxisY;
        }
        switch(index2)
        {
            case ROTY_ANALOG_INDEX:
                value2 += mOffsetRotY;
                break;
            case YAXIS_ANALOG_INDEX:
                value2 += mOffsetAxisY;
                break;
            case XAXIS_ANALOG_INDEX:
                value2 += mOffsetAxisX;
                break;
            case ZAXIS_ANALOG_INDEX:
                value2 += mOffsetAxisZ;
                break;
        }
        if(value2 > 255)
            value2 = 255;
        if(value2 < 0)
            value2 = 0;
        if(index2 == YAXIS_ANALOG_INDEX)
        {
            mAxisY = value2-mOffsetAxisY;
        }
        switch(index3)
        {
            case ROTY_ANALOG_INDEX:
                value3 += mOffsetRotY;
                break;
            case YAXIS_ANALOG_INDEX:
                value3 += mOffsetAxisY;
                break;
            case XAXIS_ANALOG_INDEX:
                value3 += mOffsetAxisX;
                break;
            case ZAXIS_ANALOG_INDEX:
                value3 += mOffsetAxisZ;
                break;
        }
        if(value3 > 255)
            value3 = 255;
        if(value3 < 0)
            value3 = 0;
        if(index3 == YAXIS_ANALOG_INDEX)
        {
            mAxisY = value3-mOffsetAxisY;
        }
        switch(index4)
        {
            case ROTY_ANALOG_INDEX:
                value4 += mOffsetRotY;
                break;
            case YAXIS_ANALOG_INDEX:
                value4 += mOffsetAxisY;
                break;
            case XAXIS_ANALOG_INDEX:
                value4 += mOffsetAxisX;
                break;
            case ZAXIS_ANALOG_INDEX:
                value4 += mOffsetAxisZ;
                break;
        }
        if(value4 > 255)
            value4 = 255;
        if(value4 < 0)
            value4 = 0;
        if(index4 == YAXIS_ANALOG_INDEX)
        {
            mAxisY = value4-mOffsetAxisY;
        }
        byte[] data = initData();
        data[index1] = (byte)(value1 & 0xff);
        data[index2] = (byte)(value2 & 0xff);
        data[index3] = (byte)(value3 & 0xff);
        data[index4] = (byte)(value4 & 0xff);

        boolean isEmpty = node.writeQueueIsEmpty();
        if(isEmpty)
        {
            numWriteLeft = 0;
            numWriteRight = 0;
        }

        if(isEmpty && mEnableWriteBLEQueque) // safer: the slowest win
        {
            writeData(data);
            numWriteLeft++;
            numWriteRight++;

            mEnableWriteBLEQueque = false;
        }
    }

    void enableWriteBLEQueque() {
        mEnableWriteBLEQueque = true;
    }


    void writeAnalogJoystick(int index, int value) {

        switch(index)
        {
            case ROTY_ANALOG_INDEX:
                value += mOffsetRotY;
                break;
            case YAXIS_ANALOG_INDEX:
                value += mOffsetAxisY;
                break;
            case XAXIS_ANALOG_INDEX:
                value += mOffsetAxisX;
                break;
            case ZAXIS_ANALOG_INDEX:
                value += mOffsetAxisZ;
                break;
        }
        if(value > 255)
            value = 255;
        if(value < 0)
            value = 0;
        if(index == YAXIS_ANALOG_INDEX)
        {
            mAxisY = value-mOffsetAxisY;
        }
        byte[] data = initData();
        data[index] = (byte)(value & 0xff);
        writeData(data);
    }

    void setOffsetRotY(int value) {
        mOffsetRotY = value;
    }
    void setOffsetAxisX(int value) {
        mOffsetAxisX = value;
    }
    void setOffsetAxisY(int value) {
        mOffsetAxisY = value;
    }
    void setOffsetAxisZ(int value) {
        mOffsetAxisZ = value;
    }


    void setStatusJoystick(int mask, int value) {
        mDataStatus &= (byte)(mask);
        mDataStatus |= (byte)(value);
        byte[] data = initData();
        writeData(data);
    }

    /**
     * Extract the gyroscope data, it will read 3 int16
     *
     * @param data array where read the Field data
     * @param dataOffset offset where start to read the data
     * @return number of read byte (10) and data extracted (the Joystick information)
     * @throws IllegalArgumentException if the data array has not enough data
     */
    @Override
    protected ExtractResult extractData(long timestamp, byte[] data, int dataOffset) {
        if (data.length - dataOffset < (STATUS_INDEX+1))
            throw new IllegalArgumentException("There are no 7 bytes available to read");
        return new ExtractResult(null,STATUS_INDEX+1);
    }//update

    @Override
    public String toString(){
        //create the string with the feature data
        StringBuilder sb = new StringBuilder();

        sb.append("no data available");

        return sb.toString();
    }

}
