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

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Build;
import android.support.annotation.NonNull;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewTreeObserver;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.RadioButton;
import android.widget.SeekBar;
import android.widget.TextView;

import com.st.BlueSTSDK.Manager;
import com.st.BlueSTSDK.Node;

import java.util.Vector;

import static com.st.STDrone.DefaultParameters.mCoeff_a_Yaw_default;
import static com.st.STDrone.DefaultParameters.mCoeff_a_xz_default;
import static com.st.STDrone.DefaultParameters.mCoeff_a_y_default;
import static com.st.STDrone.DefaultParameters.mCoeff_b_Yaw_default;
import static com.st.STDrone.DefaultParameters.mCoeff_b_xz_default;
import static com.st.STDrone.DefaultParameters.mCoeff_b_y_default;
import static com.st.STDrone.DefaultParameters.mCoeff_c_Yaw_default;
import static com.st.STDrone.DefaultParameters.mCoeff_c_xz_default;
import static com.st.STDrone.DefaultParameters.mCoeff_c_y_default;
import static com.st.STDrone.DefaultParameters.mFactor_f_Yaw_default;
import static com.st.STDrone.DefaultParameters.mFactor_f_xz_default;
import static com.st.STDrone.DefaultParameters.mFactor_f_y_default;
import static com.st.STDrone.DefaultParameters.mRatio_r_Yaw_default;
import static com.st.STDrone.DefaultParameters.mRatio_r_xz_default;
import static com.st.STDrone.DefaultParameters.mRatio_r_y_default;

import static com.st.STDrone.CoordConverter.mWidthPixels;
import static com.st.STDrone.CoordConverter.mHeightPixels;

public class Sensitivity extends AppCompatActivity {

    private final static String NODE_TAG = Sensitivity.class.getCanonicalName() + "" +
            ".NODE_TAG";

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

    private RadioButton RadioButtonSlope_OFF;
    private RadioButton RadioButtonSlope_ON;
    private boolean RadioButtonSlope_standard = true;

    private SeekBar mSeekBarYaw;
    private SeekBar mSeekBarPitchRoll;
    private SeekBar mSeekBarThrottle;

    private final static int CASE_Yaw = -1;
    private final static int CASE_PitchRoll = 1;
    private final static int CASE_Throttle = 0;

    private int mProgressSeekBar;
    private int mProgressSeekBar_Yaw;
    private int mProgressSeekBar_PitchRoll;
    private int mProgressSeekBar_Throttle;

    private static boolean mNode_status = false;

    private SharedPreferences.Editor mEditor;

    private static int mNumObjectsTot = 10;
    private static int mNumObjects = 0;
    private static boolean mFirstTime = true;
    private static float mScale = 1.0f;
    private static float mScaleWorst = 1.0f;
    private int mIndex;
    private static int mWidth[] = new int[mNumObjectsTot];
    private static int mHeight[] = new int[mNumObjectsTot];
    private static int mLeft[] = new int[mNumObjectsTot];
    private static int mTop[] = new int[mNumObjectsTot];
//    private static Vector<Integer> mWidth = new Vector<>(mNumObjectsTot);
//    private static Vector<Integer> mHeight = new Vector<>(mNumObjectsTot);
//    private static Vector<Integer> mLeft = new Vector<>(mNumObjectsTot);
//    private static Vector<Integer> mTop = new Vector<>(mNumObjectsTot);


    /**
     * create an intent for start this activity
     *
     * @param c    context used for create the intent
      * @return intent for start a demo activity that use the node as data source
     */
        public static Intent getStartIntent(Context c, @NonNull Node node) {
        Intent i = new Intent(c, Sensitivity.class);
        if(node == null)
        {
            mNode_status = false;
        }
        else
        {
            mNode_status = true;
            i.putExtra(NODE_TAG, node.getTag());
            i.putExtras(NodeContainerFragment.prepareArguments(node));
        }
        return i;
    }//getStartIntent

    public void setText(){
        TextView textSeekBar = findViewById(R.id.textSeekBarYaw);
        textSeekBar.setText("         Yaw non linear start slope         = " + String.format("%.2f",mRatio_r_Yaw));
        textSeekBar = findViewById(R.id.textSeekBarPitchRoll);
        textSeekBar.setText("Pitch and Roll non linear start slope = " + String.format("%.2f",mRatio_r_xz));
        textSeekBar = findViewById(R.id.textSeekBarThrottle);
        textSeekBar.setText("     Throttle non linear start slope      = " + String.format("%.2f",mRatio_r_y));
    }

    public void setTextLong() {
        TextView textSeekBar = findViewById(R.id.textSeekBarYaw);
        textSeekBar.setText("                 ratio Yaw start Max       = " + String.format("%.2f", mFactor_f_Yaw) + "             so Yaw non linear start slope = " + String.format("%.2f", mRatio_r_Yaw));
        textSeekBar = findViewById(R.id.textSeekBarPitchRoll);
        textSeekBar.setText("ratio Pitch and Roll start Max = " + String.format("%.2f", mFactor_f_xz) + " so Pitch and Roll non linear start slope = " + String.format("%.2f", mRatio_r_xz));
        textSeekBar = findViewById(R.id.textSeekBarThrottle);
        textSeekBar.setText("           ratio Throttle start Max      = " + String.format("%.2f", mFactor_f_y) + "      so Throttle non linear start slope = " + String.format("%.2f", mRatio_r_y));
    }


    public void ProgressBarUpdate(){
        mProgressSeekBar_Yaw = (int)((mRatio_r_Yaw - 1.0f)/(mFactor_f_Yaw-1f) * 100f+0.5f);
        mProgressSeekBar_PitchRoll = (int)((mRatio_r_xz - 1.0f)/(mFactor_f_xz-1f) * 100f+0.5f);
        mProgressSeekBar_Throttle = (int)((mRatio_r_y - 1.0f)/(mFactor_f_y-1f) * 100f+0.5f);
        TextView textSeekBar = findViewById(R.id.textSeekBarYaw);
        if(mFirstTime)
            setTextLong();
        //setText();
        else
            setText();
    }

    void getParameters(SharedPreferences pref){
        mCoeff_a_Yaw = pref.getFloat("mCoeff_a_Yaw", mCoeff_a_Yaw_default);
        mCoeff_b_Yaw = pref.getFloat("mCoeff_b_Yaw", mCoeff_b_Yaw_default);
        mCoeff_c_Yaw = pref.getFloat("mCoeff_c_Yaw", mCoeff_c_Yaw_default);
        mRatio_r_Yaw = pref.getFloat("mRatio_r_Yaw", mRatio_r_Yaw_default);
        mFactor_f_Yaw = pref.getFloat("mFactor_f_Yaw", mFactor_f_Yaw_default);
        mCoeff_a_y = pref.getFloat("mCoeff_a_y", mCoeff_a_y_default);
        mCoeff_b_y = pref.getFloat("mCoeff_b_y", mCoeff_b_y_default);
        mCoeff_c_y = pref.getFloat("mCoeff_c_y", mCoeff_c_y_default);
        mRatio_r_y = pref.getFloat("mRatio_r_y", mRatio_r_y_default);
        mFactor_f_y = pref.getFloat("mFactor_f_y", mFactor_f_y_default);
        mCoeff_a_xz = pref.getFloat("mCoeff_a_xz", mCoeff_a_xz_default);
        mCoeff_b_xz = pref.getFloat("mCoeff_b_xz", mCoeff_b_xz_default);
        mCoeff_c_xz = pref.getFloat("mCoeff_c_xz", mCoeff_c_xz_default);
        mRatio_r_xz = pref.getFloat("mRatio_r_xz", mRatio_r_xz_default);
        mFactor_f_xz = pref.getFloat("mFactor_f_xz", mFactor_f_xz_default);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_sensivity);

        SharedPreferences pref = getApplicationContext().getSharedPreferences("MyPref", MODE_PRIVATE);
        mEditor = pref.edit();
        getParameters(pref);


        RadioButtonSlope_OFF = findViewById(R.id.RadioButtonSlope_OFF);
        RadioButtonSlope_OFF.setChecked(true);
        RadioButtonSlope_ON = findViewById(R.id.RadioButtonSlope_ON);

        ProgressBarUpdate();

        mSeekBarYaw = findViewById(R.id.seekBarYaw);
        mSeekBarYaw.setProgress(mProgressSeekBar_Yaw);
        mSeekBarYaw.setOnSeekBarChangeListener(mSeekBarListenerYaw);

        mSeekBarPitchRoll = findViewById(R.id.seekBarPitchRoll);
        mSeekBarPitchRoll.setProgress(mProgressSeekBar_PitchRoll);
        mSeekBarPitchRoll.setOnSeekBarChangeListener(mSeekBarListenerPitchRoll);

        mSeekBarThrottle = findViewById(R.id.seekBarThrottle);
        mSeekBarThrottle.setProgress(mProgressSeekBar_Throttle);
        mSeekBarThrottle.setOnSeekBarChangeListener(mSeekBarListenerThrottle);

        findViewById(R.id.Home).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                final Node n;
                if(mNode_status == false)
                {
                    n = null;
                }
                else
                {
                    String nodeTag = getIntent().getStringExtra(NODE_TAG);
                    n = Manager.getSharedInstance().getNodeWithTag(nodeTag);
                }
                Intent i = Joystick.getStartIntent(Sensitivity.this,n);
                startActivity(i);
            }
        });

        findViewById(R.id.Setting).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                final Node n;
                if(mNode_status == false)
                {
                    n = null;
                }
                else
                {
                    String nodeTag = getIntent().getStringExtra(NODE_TAG);
                    n = Manager.getSharedInstance().getNodeWithTag(nodeTag);
                }
                Intent i = Setting.getStartIntent(Sensitivity.this,n);
                startActivity(i);
            }
        });


        findViewById(R.id.Help).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                final Node n;
                if(mNode_status == false)
                {
                    n = null;
                }
                else
                {
                    String nodeTag = getIntent().getStringExtra(NODE_TAG);
                    n = Manager.getSharedInstance().getNodeWithTag(nodeTag);
                }

                Intent i = Help.getStartIntent(Sensitivity.this,n);
                startActivity(i);
            }
        });
    }

    public void onRadioButtonSlope(View view) {
        // Is the button now checked?
        boolean checked = ((RadioButton) view).isChecked();

        // Check which radio button was clicked
        switch(view.getId()) {
            case R.id.RadioButtonSlope_OFF:
                if (checked) {
                    RadioButtonSlope_standard = true;
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                        if(mProgressSeekBar_Yaw != ((int) mFactor_f_Yaw))
                            mSeekBarYaw.setProgress(mProgressSeekBar_Yaw);
                        if(mProgressSeekBar_PitchRoll != ((int) mFactor_f_xz))
                            mSeekBarPitchRoll.setProgress(mProgressSeekBar_PitchRoll);
                        if(mProgressSeekBar_Throttle != ((int) mFactor_f_y))
                            mSeekBarThrottle.setProgress(mProgressSeekBar_Throttle);
                        setText();
                        }
                    });
                }
                break;
            case R.id.RadioButtonSlope_ON:
                if (checked) {
                    RadioButtonSlope_standard = false;
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                        if(mProgressSeekBar_Yaw != ((int) mFactor_f_Yaw))
                            mSeekBarYaw.setProgress((int) mFactor_f_Yaw);
                        if(mProgressSeekBar_PitchRoll != ((int) mFactor_f_xz))
                            mSeekBarPitchRoll.setProgress((int) mFactor_f_xz);
                        if(mProgressSeekBar_Throttle != ((int) mFactor_f_y))
                            mSeekBarThrottle.setProgress((int) mFactor_f_y);
                        TextView textSeekBar = findViewById(R.id.textSeekBarYaw);
                        textSeekBar.setText("                 ratio Yaw start Max       = " + String.format("%.2f",mFactor_f_Yaw)+"             so Yaw non linear start slope = " + String.format("%.2f",mRatio_r_Yaw));
                        textSeekBar = findViewById(R.id.textSeekBarPitchRoll);
                        textSeekBar.setText("ratio Pitch and Roll start Max = " + String.format("%.2f",mFactor_f_xz)+" so Pitch and Roll non linear start slope = " + String.format("%.2f",mRatio_r_xz));
                        textSeekBar = findViewById(R.id.textSeekBarThrottle);
                        textSeekBar.setText("           ratio Throttle start Max      = " + String.format("%.2f",mFactor_f_y)+"      so Throttle non linear start slope = " + String.format("%.2f",mRatio_r_y));
                        }
                    });
                }
                break;
        }
    }


//    void fakePos(){ // to enable reading post()
//        FrameLayout.LayoutParams params;
//
//        Log.d("position", "fakePos"); // for debug
//
//        for(int i=0; i < 100000; i++) {
//            Button selectedButton = findViewById(R.id.Home);
//            params = (FrameLayout.LayoutParams) selectedButton.getLayoutParams();
//            params.setMargins(params.leftMargin, params.topMargin, params.rightMargin, params.bottomMargin);
//            selectedButton.setLayoutParams(params);
//
//            selectedButton = findViewById(R.id.Setting);
//            params = (FrameLayout.LayoutParams) selectedButton.getLayoutParams();
//            params.setMargins(params.leftMargin, params.topMargin, params.rightMargin, params.bottomMargin);
//            selectedButton.setLayoutParams(params);
//
//            selectedButton = findViewById(R.id.Help);
//            params = (FrameLayout.LayoutParams) selectedButton.getLayoutParams();
//            params.setMargins(params.leftMargin, params.topMargin, params.rightMargin, params.bottomMargin);
//            selectedButton.setLayoutParams(params);
//
//            SeekBar selectedSeekBar = findViewById(R.id.seekBarYaw);
//            params = (FrameLayout.LayoutParams) selectedSeekBar.getLayoutParams();
//            params.setMargins(params.leftMargin, params.topMargin, params.rightMargin, params.bottomMargin);
//            selectedSeekBar.setLayoutParams(params);
//
//            TextView selectedTextView = findViewById(R.id.textSeekBarYaw);
//            params = (FrameLayout.LayoutParams) selectedTextView.getLayoutParams();
//            params.setMargins(params.leftMargin, params.topMargin, params.rightMargin, params.bottomMargin);
//            selectedTextView.setLayoutParams(params);
//
//            selectedSeekBar = findViewById(R.id.seekBarPitchRoll);
//            params = (FrameLayout.LayoutParams) selectedSeekBar.getLayoutParams();
//            params.setMargins(params.leftMargin, params.topMargin, params.rightMargin, params.bottomMargin);
//            selectedSeekBar.setLayoutParams(params);
//
//            selectedTextView = findViewById(R.id.textSeekBarPitchRoll);
//            params = (FrameLayout.LayoutParams) selectedTextView.getLayoutParams();
//            params.setMargins(params.leftMargin, params.topMargin, params.rightMargin, params.bottomMargin);
//            selectedTextView.setLayoutParams(params);
//
//            selectedSeekBar = findViewById(R.id.seekBarThrottle);
//            params = (FrameLayout.LayoutParams) selectedSeekBar.getLayoutParams();
//            params.setMargins(params.leftMargin, params.topMargin, params.rightMargin, params.bottomMargin);
//            selectedSeekBar.setLayoutParams(params);
//
//            selectedTextView = findViewById(R.id.textSeekBarThrottle);
//            params = (FrameLayout.LayoutParams) selectedTextView.getLayoutParams();
//            params.setMargins(params.leftMargin, params.topMargin, params.rightMargin, params.bottomMargin);
//            selectedTextView.setLayoutParams(params);
//
//            LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonSlope);
//            params = (FrameLayout.LayoutParams) selectedLinearLayout.getLayoutParams();
//            params.setMargins(params.leftMargin, params.topMargin, params.rightMargin, params.bottomMargin);
//            selectedLinearLayout.setLayoutParams(params);
//        }
//
//    }
//
//    void fakePos2(){
//
//        Log.d("position", "fakePos2"); // for debug
//
//        for(int i=0; i < 100000; i++) {
//            Button selectedButton = findViewById(R.id.Home);
//            selectedButton.setScaleX(mScale);
//            selectedButton.setScaleY(mScale);
//
//            selectedButton = findViewById(R.id.Setting);
//            selectedButton.setScaleX(mScale);
//            selectedButton.setScaleY(mScale);
//
//            selectedButton = findViewById(R.id.Help);
//            selectedButton.setScaleX(mScale);
//            selectedButton.setScaleY(mScale);
//
//            SeekBar selectedSeekBar = findViewById(R.id.seekBarYaw);
//            selectedSeekBar.setScaleX(mScaleWorst);
//            selectedSeekBar.setScaleY(mScaleWorst);
//
//            TextView selectedTextView = findViewById(R.id.textSeekBarYaw);
//            selectedTextView.setScaleX(mScaleWorst);
//            selectedTextView.setScaleY(mScaleWorst);
//
//            selectedSeekBar = findViewById(R.id.seekBarPitchRoll);
//            selectedSeekBar.setScaleX(mScaleWorst);
//            selectedSeekBar.setScaleY(mScaleWorst);
//
//            selectedTextView = findViewById(R.id.textSeekBarPitchRoll);
//            selectedTextView.setScaleX(mScaleWorst);
//            selectedTextView.setScaleY(mScaleWorst);
//
//            selectedSeekBar = findViewById(R.id.seekBarThrottle);
//            selectedSeekBar.setScaleX(mScaleWorst);
//            selectedSeekBar.setScaleY(mScaleWorst);
//
//            selectedTextView = findViewById(R.id.textSeekBarThrottle);
//            selectedTextView.setScaleX(mScaleWorst);
//            selectedTextView.setScaleY(mScaleWorst);
//
//            LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonSlope);
//            selectedLinearLayout.setScaleX(mScale);
//            selectedLinearLayout.setScaleY(mScale);
//        }
//    }
//

    void movePos(){
        Log.d("notification", this+" movePos "); // for debug

        FrameLayout.LayoutParams params;

        int i = 0;

        Button selectedButton = findViewById(R.id.Home);
        params = (FrameLayout.LayoutParams) selectedButton.getLayoutParams();
        params.setMargins(mLeft[i], mTop[i], params.rightMargin, params.bottomMargin);
        selectedButton.setLayoutParams(params);
        selectedButton.setScaleX(mScale);
        selectedButton.setScaleY(mScale);
        i++;

        selectedButton = findViewById(R.id.Setting);
        params = (FrameLayout.LayoutParams) selectedButton.getLayoutParams();
        params.setMargins(mLeft[i], mTop[i], params.rightMargin, params.bottomMargin);
        selectedButton.setLayoutParams(params);
        selectedButton.setScaleX(mScale);
        selectedButton.setScaleY(mScale);
        i++;

        selectedButton = findViewById(R.id.Help);
        params = (FrameLayout.LayoutParams) selectedButton.getLayoutParams();
        params.setMargins(mLeft[i], mTop[i], params.rightMargin, params.bottomMargin);
        selectedButton.setLayoutParams(params);
        selectedButton.setScaleX(mScale);
        selectedButton.setScaleY(mScale);
        i++;

        SeekBar selectedSeekBar = findViewById(R.id.seekBarYaw);
        params = (FrameLayout.LayoutParams) selectedSeekBar.getLayoutParams();
        params.setMargins(mLeft[i], mTop[i], params.rightMargin, params.bottomMargin);
        selectedSeekBar.setLayoutParams(params);
        selectedSeekBar.setScaleX(mScaleWorst);
        selectedSeekBar.setScaleY(mScaleWorst);
        i++;

        TextView selectedTextView = findViewById(R.id.textSeekBarYaw);
        params = (FrameLayout.LayoutParams) selectedTextView.getLayoutParams();
        params.setMargins(mLeft[i], mTop[i], params.rightMargin, params.bottomMargin);
        selectedTextView.setLayoutParams(params);
        selectedTextView.setScaleX(mScaleWorst);
        selectedTextView.setScaleY(mScaleWorst);
        i++;

        selectedSeekBar = findViewById(R.id.seekBarPitchRoll);
        params = (FrameLayout.LayoutParams) selectedSeekBar.getLayoutParams();
        params.setMargins(mLeft[i], mTop[i], params.rightMargin, params.bottomMargin);
        selectedSeekBar.setLayoutParams(params);
        selectedSeekBar.setScaleX(mScaleWorst);
        selectedSeekBar.setScaleY(mScaleWorst);
        i++;

        selectedTextView = findViewById(R.id.textSeekBarPitchRoll);
        params = (FrameLayout.LayoutParams) selectedTextView.getLayoutParams();

        int leftMargin = params.leftMargin;
        Log.d("position", "leftMargin "+leftMargin); // for debug
        int topMargin = params.topMargin;
        Log.d("position", "topMargin "+topMargin); // for debug

        params.setMargins(mLeft[i], mTop[i], params.rightMargin, params.bottomMargin);
        selectedTextView.setLayoutParams(params);

        int width6pre = selectedTextView.getWidth();
        Log.d("position", "width6pre "+width6pre); // for debug

        selectedTextView.setScaleX(mScaleWorst);
        selectedTextView.setScaleY(mScaleWorst);

        int width6 = selectedTextView.getWidth();
        Log.d("position", "width6 "+width6); // for debug
        Log.d("position", "mWidth[6]*mScaleWorst "+(mWidth[6]*mScaleWorst)); // for debug


        i++;

        selectedSeekBar = findViewById(R.id.seekBarThrottle);
        params = (FrameLayout.LayoutParams) selectedSeekBar.getLayoutParams();
        params.setMargins(mLeft[i], mTop[i], params.rightMargin, params.bottomMargin);
        selectedSeekBar.setLayoutParams(params);
        selectedSeekBar.setScaleX(mScaleWorst);
        selectedSeekBar.setScaleY(mScaleWorst);
        i++;

        selectedTextView = findViewById(R.id.textSeekBarThrottle);
        params = (FrameLayout.LayoutParams) selectedTextView.getLayoutParams();
        params.setMargins(mLeft[i], mTop[i], params.rightMargin, params.bottomMargin);
        selectedTextView.setLayoutParams(params);
        selectedTextView.setScaleX(mScaleWorst);
        selectedTextView.setScaleY(mScaleWorst);
        i++;

        LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonSlope);
        params = (FrameLayout.LayoutParams) selectedLinearLayout.getLayoutParams();
        params.setMargins(mLeft[i], mTop[i], params.rightMargin, params.bottomMargin);
        selectedLinearLayout.setLayoutParams(params);
        selectedLinearLayout.setScaleX(mScale);
        selectedLinearLayout.setScaleY(mScale);
        i++;

    }

    void initPos(){

        Log.d("notification", this+" initPos "); // for debug

        int space;

        Log.d("position", "mWidthDp init "+CoordConverter.fromPx_to_dp(mWidthPixels)); // for debug
        space = mWidthPixels - (mWidth[0] + mWidth[1] + mWidth[2]);
        space /= 4;
        float avgSizeObj = (mWidth[0] + mWidth[1] + mWidth[2])/3.0f;
        mScale = space/avgSizeObj;
        Log.d("position", "mScale init "+mScale); // for debug

        // worst case
        float extentionTextFactor;
        if (CoordConverter.fromPx_to_dp(mWidthPixels) >= 700) {
            extentionTextFactor = 1.0f; //start slope rate max case (normal case)
        } else
            extentionTextFactor = 0.8f; //start slope rate max case (abnormal case)
        Log.d("position", "extentionTextFactor init "+extentionTextFactor); // for debug
        Log.d("position", "mWidth[6] init "+mWidth[6]); // for debug
        space = (int)(mWidthPixels - CoordConverter.fromDp_to_px(15)*2 + 0.5f);
        space /= 2;
        avgSizeObj = (mWidth[5] + mWidth[6]*extentionTextFactor)/2.0f;
        mScaleWorst = space/avgSizeObj;
        Log.d("position", "mScaleWorst init "+mScaleWorst); // for debug

        //space = mHeightPixels - mHeight[0] - mHeight[3] - mHeight[5] - mHeight[7] - mHeight[9];
        //space /= 6;
        space = (int)(mWidthPixels - CoordConverter.fromDp_to_px(50)*6 + 0.5f);
        space /= 5;
        float avgSizeObjy = (mHeight[0] + mHeight[3] + mHeight[5] + mHeight[7] + mHeight[9])/5.0f;
        float scaley = space/avgSizeObjy;
        Log.d("position", "scaley init "+scaley); // for debug
        avgSizeObjy = CoordConverter.fromPx_to_dp((int)avgSizeObjy);
        Log.d("position", "avgSizeObjy init "+avgSizeObjy); // for debug
        if(scaley < mScale)
            mScale = scaley;
        if(mScale < 1.0f)
            mScale = 1.0f;
        float heightMax = 75.0f;
        if(mScale*avgSizeObjy > heightMax)
            mScale = heightMax/avgSizeObjy;
        //mScale = 1.0f;
        Log.d("position", "mScale init final "+mScale); // for debug
        if(scaley < mScaleWorst)
            mScaleWorst = scaley;
        if(mScaleWorst*avgSizeObjy > heightMax)
            mScaleWorst = heightMax/avgSizeObjy;
        //mScaleWorst = 1.0f;
        Log.d("position", "mScaleWorst init final "+mScaleWorst); // for debug

        // the margin is the same also after scale, but scale changes the object size
        // so the margin after scale = margin desidered = margin before scale - (scale-1)size*0.5
        // so margin before scale = margin desidered + (scale-1)size*0.5
        // so margin desidered = first space

        // the next margin needs an offset
        // margin desidered = Sum (previous margin desidered) + current margin desidered respect the previous object
        // so margin desidered = Sum (previous object size) + Sum (previous+current margin desidered)
        // so margin before scale = Sum (previous object size) + Sum (previous+current space desidered) + (scale-1)size*0.5
        float offset = 0;
        int numSpace = 1;
        int i = 0;
        space = (int) (mWidthPixels - (mWidth[0] + mWidth[1] + mWidth[2])*mScale+ 0.5f);
        space /= 4; // num space between objects = num objects + 1
        // so margin before scale = Sum (previous object size) + Sum (previous+current space desidered) + (scale-1)size*0.5
        mLeft[i] = (int) (offset+mWidth[i]*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
        offset += mWidth[i]*mScale;
        numSpace++;
        i++;
        mLeft[i] = (int) (offset+mWidth[i]*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
        offset += mWidth[i]*mScale;
        numSpace++;
        i++;
        mLeft[i] = (int) (offset+mWidth[i]*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
        i++;

        offset = 0;
        numSpace = 1;
        int specialSpace = CoordConverter.fromDp_to_px(0);
        space = (int) (mWidthPixels - (mWidth[i] + mWidth[i+1]*extentionTextFactor+specialSpace)*mScaleWorst+ 0.5f);
        space /= (3-1); // num space between objects less specialSpace = num objects + 1 - 1 = num objects
        mLeft[i] = (int) (offset+mWidth[i]*(mScaleWorst-1.0f)*0.5 +numSpace*space + 0.5f);
        offset += mWidth[i]*mScaleWorst;
        numSpace++;
        i++;
        // so margin before scale = Sum (previous object size) + Sum (previous space desidered) + specialSpace + (scale-1)size*0.5
        mLeft[i] = (int) (offset+mWidth[i]*extentionTextFactor*(mScaleWorst-1.0f)*0.5 +(numSpace-1)*space+specialSpace + 0.5f);
        i++;

        offset = 0;
        numSpace = 1;
        space = (int) (mWidthPixels - (mWidth[i] + mWidth[i+1]*extentionTextFactor+specialSpace)*mScaleWorst+ 0.5f);
        space /= (3-1);
        mLeft[i] = (int) (offset+mWidth[i]*(mScaleWorst-1.0f)*0.5 +numSpace*space + 0.5f);
        offset += mWidth[i]*mScaleWorst;
        numSpace++;
        i++;
        mLeft[i] = (int) (offset+mWidth[i]*extentionTextFactor*(mScaleWorst-1.0f)*0.5 +(numSpace-1)*space+specialSpace + 0.5f);
        i++;

        offset = 0;
        numSpace = 1;
        space = (int) (mWidthPixels - (mWidth[i] + mWidth[i+1]*extentionTextFactor+specialSpace)*mScaleWorst+ 0.5f);
        space /= (3-1);
        mLeft[i] = (int) (offset+mWidth[i]*(mScaleWorst-1.0f)*0.5 +numSpace*space + 0.5f);
        offset += mWidth[i]*mScaleWorst;
        numSpace++;
        i++;
        mLeft[i] = (int) (offset+mWidth[i]*extentionTextFactor*(mScaleWorst-1.0f)*0.5 +(numSpace-1)*space+specialSpace + 0.5f);
        i++;

        offset = 0;
        numSpace = 1;
        space = (int) (mWidthPixels - (mWidth[i])*mScale+ 0.5f);
        space /= 2;
        mLeft[i] = (int) (offset+mWidth[i]*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);

        FrameLayout selectedFrameLayout = findViewById(R.id.FrameLayout);
        int top = selectedFrameLayout.getHeight();
        mHeightPixels = top; // the top menu reduces the screen Height

        offset = 0;
        numSpace = 1;
        i = 0;
        space = (int) (mHeightPixels - (mHeight[i] + mHeight[i+9])*mScale - (mHeight[i+3] + mHeight[i+5] + mHeight[i+7])*mScaleWorst + 0.5f);
        space /= 6;
        mTop[i] = (int) (offset+mHeight[i]*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
        i++;
        mTop[i] = mTop[i-1];
        i++;
        mTop[i] = mTop[i-1];
        offset += mHeight[i-2]*mScale;
        numSpace++;
        i++;
        mTop[i] = (int) (offset+mHeight[i]*(mScaleWorst-1.0f)*0.5 +numSpace*space + 0.5f);
        i++;
        mTop[i] = mTop[i-1];
        offset += mHeight[i-1]*mScaleWorst;
        numSpace++;
        i++;
        mTop[i] = (int) (offset+mHeight[i]*(mScaleWorst-1.0f)*0.5 +numSpace*space + 0.5f);
        i++;
        mTop[i] = mTop[i-1];
        offset += mHeight[i-1]*mScaleWorst;
        numSpace++;
        i++;
        mTop[i] = (int) (offset+mHeight[i]*(mScaleWorst-1.0f)*0.5 +numSpace*space + 0.5f);
        i++;
        mTop[i] = mTop[i-1];
        offset += mHeight[i-1]*mScaleWorst;
        numSpace++;
        i++;
        mTop[i] = (int) (offset+mHeight[i]*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);

        setText(); // to restore true text

        movePos();

        mFirstTime = false;
    }

    void readSize4(){
        FrameLayout.LayoutParams params;

        int i = 0;

        Button selectedButton = findViewById(R.id.Home);
        params = (FrameLayout.LayoutParams) selectedButton.getLayoutParams();
        mWidth[i] = params.width;
        mHeight[i] = params.height;
        i++;

        selectedButton = findViewById(R.id.Setting);
        params = (FrameLayout.LayoutParams) selectedButton.getLayoutParams();
        mWidth[i] = params.width;
        mHeight[i] = params.height;
        i++;

        selectedButton = findViewById(R.id.Help);
        params = (FrameLayout.LayoutParams) selectedButton.getLayoutParams();
        mWidth[i] = params.width;
        mHeight[i] = params.height;
        i++;

        SeekBar selectedSeekBar = findViewById(R.id.seekBarYaw);
        params = (FrameLayout.LayoutParams) selectedSeekBar.getLayoutParams();
        mWidth[i] = params.width;
        mHeight[i] = params.height;
        i++;

        TextView selectedTextView = findViewById(R.id.textSeekBarYaw);
        params = (FrameLayout.LayoutParams) selectedTextView.getLayoutParams();
        mWidth[i] = params.width;
        mHeight[i] = params.height;
        i++;

        selectedSeekBar = findViewById(R.id.seekBarPitchRoll);
        params = (FrameLayout.LayoutParams) selectedSeekBar.getLayoutParams();
        mWidth[i] = params.width;
        mHeight[i] = params.height;
        i++;

        selectedTextView = findViewById(R.id.textSeekBarPitchRoll);
        params = (FrameLayout.LayoutParams) selectedTextView.getLayoutParams();
        mWidth[i] = params.width;
        mHeight[i] = params.height;
        i++;

        selectedSeekBar = findViewById(R.id.seekBarThrottle);
        params = (FrameLayout.LayoutParams) selectedSeekBar.getLayoutParams();
        mWidth[i] = params.width;
        mHeight[i] = params.height;
        i++;

        selectedTextView = findViewById(R.id.textSeekBarThrottle);
        params = (FrameLayout.LayoutParams) selectedTextView.getLayoutParams();
        mWidth[i] = params.width;
        mHeight[i] = params.height;
        i++;

        LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonSlope);
        params = (FrameLayout.LayoutParams) selectedLinearLayout.getLayoutParams();
        mWidth[i] = params.width;
        mHeight[i] = params.height;
        i++;

        initPos();
    }


    void readSize(){
        mIndex = 0;

        Button selectedb = findViewById(R.id.Home);
        selectedb.post(new Runnable() {
            @Override
            public void run() {
                Button selected = findViewById(R.id.Home);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedb = findViewById(R.id.Setting);
        selectedb.post(new Runnable() {
            @Override
            public void run() {
                Button selected = findViewById(R.id.Setting);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedb = findViewById(R.id.Help);
        selectedb.post(new Runnable() {
            @Override
            public void run() {
                Button selected = findViewById(R.id.Help);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        SeekBar selected = findViewById(R.id.seekBarYaw);
        selected.post(new Runnable() {
            @Override
            public void run() {
                SeekBar selected = findViewById(R.id.seekBarYaw);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        TextView selectedt = findViewById(R.id.textSeekBarYaw);
        selectedt.post(new Runnable() {
            @Override
            public void run() {
                TextView selected = findViewById(R.id.textSeekBarYaw);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selected = findViewById(R.id.seekBarPitchRoll);
        selected.post(new Runnable() {
            @Override
            public void run() {
                SeekBar selected = findViewById(R.id.seekBarPitchRoll);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedt = findViewById(R.id.textSeekBarPitchRoll);
        selectedt.post(new Runnable() {
            @Override
            public void run() {
                TextView selected = findViewById(R.id.textSeekBarPitchRoll);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });


        selected = findViewById(R.id.seekBarThrottle);
        selected.post(new Runnable() {
            @Override
            public void run() {
                SeekBar selected = findViewById(R.id.seekBarThrottle);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedt = findViewById(R.id.textSeekBarThrottle);
        selectedt.post(new Runnable() {
            @Override
            public void run() {
                TextView selected = findViewById(R.id.textSeekBarThrottle);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        LinearLayout selectedl = findViewById(R.id.LinearLayout_RadioButtonSlope);
        selectedl.post(new Runnable() {
            @Override
            public void run() {
                LinearLayout selected = findViewById(R.id.LinearLayout_RadioButtonSlope);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });
    }

    void readSize2(){
        mIndex = 0;

        Button selectedb = findViewById(R.id.Home);
        selectedb.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
            @Override
            public void onGlobalLayout() {
                Button selected = findViewById(R.id.Home);
                selected.getViewTreeObserver().removeOnGlobalLayoutListener(this);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedb = findViewById(R.id.Setting);
        selectedb.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
            @Override
            public void onGlobalLayout() {
                Button selected = findViewById(R.id.Setting);
                selected.getViewTreeObserver().removeOnGlobalLayoutListener(this);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedb = findViewById(R.id.Help);
        selectedb.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
            @Override
            public void onGlobalLayout() {
                Button selected = findViewById(R.id.Help);
                selected.getViewTreeObserver().removeOnGlobalLayoutListener(this);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        SeekBar selected = findViewById(R.id.seekBarYaw);
        selected.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
            @Override
            public void onGlobalLayout() {
                SeekBar selected = findViewById(R.id.seekBarYaw);
                selected.getViewTreeObserver().removeOnGlobalLayoutListener(this);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        TextView selectedt = findViewById(R.id.textSeekBarYaw);
        selectedt.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
            @Override
            public void onGlobalLayout() {
                TextView selected = findViewById(R.id.textSeekBarYaw);
                selected.getViewTreeObserver().removeOnGlobalLayoutListener(this);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selected = findViewById(R.id.seekBarPitchRoll);
        selected.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
            @Override
            public void onGlobalLayout() {
                SeekBar selected = findViewById(R.id.seekBarPitchRoll);
                selected.getViewTreeObserver().removeOnGlobalLayoutListener(this);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedt = findViewById(R.id.textSeekBarPitchRoll);
        selectedt.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
            @Override
            public void onGlobalLayout() {
                TextView selected = findViewById(R.id.textSeekBarPitchRoll);
                selected.getViewTreeObserver().removeOnGlobalLayoutListener(this);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selected = findViewById(R.id.seekBarThrottle);
        selected.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
            @Override
            public void onGlobalLayout() {
                SeekBar selected = findViewById(R.id.seekBarThrottle);
                selected.getViewTreeObserver().removeOnGlobalLayoutListener(this);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedt = findViewById(R.id.textSeekBarThrottle);
        selectedt.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
            @Override
            public void onGlobalLayout() {
                TextView selected = findViewById(R.id.textSeekBarThrottle);
                selected.getViewTreeObserver().removeOnGlobalLayoutListener(this);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        LinearLayout selectedl = findViewById(R.id.LinearLayout_RadioButtonSlope);
        selectedl.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
            @Override
            public void onGlobalLayout() {
                LinearLayout selected = findViewById(R.id.LinearLayout_RadioButtonSlope);
                selected.getViewTreeObserver().removeOnGlobalLayoutListener(this);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });
    }

    void readSize3(){
        mIndex = 0;

        Button selectedb = findViewById(R.id.Home);
        selectedb.post(new Runnable() {
            @Override
            public void run() {
                Button selected = findViewById(R.id.Home);
                mWidth[mIndex] = selected.getMeasuredWidth();
                mHeight[mIndex] = selected.getMeasuredHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedb = findViewById(R.id.Setting);
        selectedb.post(new Runnable() {
            @Override
            public void run() {
                Button selected = findViewById(R.id.Setting);
                mWidth[mIndex] = selected.getMeasuredWidth();
                mHeight[mIndex] = selected.getMeasuredHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedb = findViewById(R.id.Help);
        selectedb.post(new Runnable() {
            @Override
            public void run() {
                Button selected = findViewById(R.id.Help);
                mWidth[mIndex] = selected.getMeasuredWidth();
                mHeight[mIndex] = selected.getMeasuredHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        SeekBar selected = findViewById(R.id.seekBarYaw);
        selected.post(new Runnable() {
            @Override
            public void run() {
                SeekBar selected = findViewById(R.id.seekBarYaw);
                mWidth[mIndex] = selected.getMeasuredWidth();
                mHeight[mIndex] = selected.getMeasuredHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        TextView selectedt = findViewById(R.id.textSeekBarYaw);
        selectedt.post(new Runnable() {
            @Override
            public void run() {
                TextView selected = findViewById(R.id.textSeekBarYaw);
                mWidth[mIndex] = selected.getMeasuredWidth();
                mHeight[mIndex] = selected.getMeasuredHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selected = findViewById(R.id.seekBarPitchRoll);
        selected.post(new Runnable() {
            @Override
            public void run() {
                SeekBar selected = findViewById(R.id.seekBarPitchRoll);
                mWidth[mIndex] = selected.getMeasuredWidth();
                mHeight[mIndex] = selected.getMeasuredHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedt = findViewById(R.id.textSeekBarPitchRoll);
        selectedt.post(new Runnable() {
            @Override
            public void run() {
                TextView selected = findViewById(R.id.textSeekBarPitchRoll);
                mWidth[mIndex] = selected.getMeasuredWidth();
                mHeight[mIndex] = selected.getMeasuredHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });


        selected = findViewById(R.id.seekBarThrottle);
        selected.post(new Runnable() {
            @Override
            public void run() {
                SeekBar selected = findViewById(R.id.seekBarThrottle);
                mWidth[mIndex] = selected.getMeasuredWidth();
                mHeight[mIndex] = selected.getMeasuredHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedt = findViewById(R.id.textSeekBarThrottle);
        selectedt.post(new Runnable() {
            @Override
            public void run() {
                TextView selected = findViewById(R.id.textSeekBarThrottle);
                mWidth[mIndex] = selected.getMeasuredWidth();
                mHeight[mIndex] = selected.getMeasuredHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        LinearLayout selectedl = findViewById(R.id.LinearLayout_RadioButtonSlope);
        selectedl.post(new Runnable() {
            @Override
            public void run() {
                LinearLayout selected = findViewById(R.id.LinearLayout_RadioButtonSlope);
                mWidth[mIndex] = selected.getMeasuredWidth();
                mHeight[mIndex] = selected.getMeasuredHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });
    }

    void readSize5(){
        mIndex = 0;

        int delay = 100;

        Button selectedb = findViewById(R.id.Home);
        selectedb.postDelayed(new Runnable() {
            @Override
            public void run() {
                Button selected = findViewById(R.id.Home);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        },delay);

        selectedb = findViewById(R.id.Setting);
        selectedb.postDelayed(new Runnable() {
            @Override
            public void run() {
                Button selected = findViewById(R.id.Setting);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        },delay);

        selectedb = findViewById(R.id.Help);
        selectedb.postDelayed(new Runnable() {
            @Override
            public void run() {
                Button selected = findViewById(R.id.Help);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        },delay);

        SeekBar selected = findViewById(R.id.seekBarYaw);
        selected.postDelayed(new Runnable() {
            @Override
            public void run() {
                SeekBar selected = findViewById(R.id.seekBarYaw);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        },delay);

        TextView selectedt = findViewById(R.id.textSeekBarYaw);
        selectedt.postDelayed(new Runnable() {
            @Override
            public void run() {
                TextView selected = findViewById(R.id.textSeekBarYaw);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        },delay);

        selected = findViewById(R.id.seekBarPitchRoll);
        selected.postDelayed(new Runnable() {
            @Override
            public void run() {
                SeekBar selected = findViewById(R.id.seekBarPitchRoll);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        },delay);

        selectedt = findViewById(R.id.textSeekBarPitchRoll);
        selectedt.postDelayed(new Runnable() {
            @Override
            public void run() {
                TextView selected = findViewById(R.id.textSeekBarPitchRoll);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        },delay);


        selected = findViewById(R.id.seekBarThrottle);
        selected.postDelayed(new Runnable() {
            @Override
            public void run() {
                SeekBar selected = findViewById(R.id.seekBarThrottle);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        },delay);

        selectedt = findViewById(R.id.textSeekBarThrottle);
        selectedt.postDelayed(new Runnable() {
            @Override
            public void run() {
                TextView selected = findViewById(R.id.textSeekBarThrottle);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        },delay);

        LinearLayout selectedl = findViewById(R.id.LinearLayout_RadioButtonSlope);
        selectedl.postDelayed(new Runnable() {
            @Override
            public void run() {
                LinearLayout selected = findViewById(R.id.LinearLayout_RadioButtonSlope);
                mWidth[mIndex] = selected.getWidth();
                mHeight[mIndex] = selected.getHeight();
                mIndex++;
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        },delay);
    }

    @Override
    protected void onStart() {
        Log.d("notification", this+" onStart "); // for debug
        super.onStart();

        if(mFirstTime)
        {
            readSize();
        }
        //else  //Samsung requires always movePos to have a correct readSize()
            movePos();
    }

//    @Override
//    public void onWindowFocusChanged(boolean hasFocus) {
//        super.onWindowFocusChanged(hasFocus);
//        //Here you can get the size!
//        Log.d("notification", this+" onWindowFocusChanged "); // for debug
//        if(mFirstTime)
//        {
//            readSize();
//        }
//    }

    private SeekBar.OnSeekBarChangeListener mSeekBarListenerYaw = new SeekBar.OnSeekBarChangeListener() {
        @Override
        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
            //progress	int: The current progress level. This will be in the range min..max where min
            //and max were set by setMin(int) and setMax(int), respectively. (The default values for min is 0 and max is 100.)
            {
                Log.d("scrollText", "SeekBar : progress value: "+String.valueOf(progress)); // for debug
                mProgressSeekBar = progress;
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        TextView textSeekBar = findViewById(R.id.textSeekBarYaw);
                        if(RadioButtonSlope_standard)
                        {
                            mProgressSeekBar_Yaw = mProgressSeekBar;
                            NonLinearCoeffBuilding(mProgressSeekBar_Yaw, CASE_Yaw);
                            textSeekBar.setText("         Yaw non linear start slope         = " + String.format("%.2f",mRatio_r_Yaw));
                        }
                        else
                        {
                            int s;
                            if(mProgressSeekBar < 3)
                                mProgressSeekBar = 3;
                            mFactor_f_Yaw = (float) mProgressSeekBar;
                            if(mRatio_r_Yaw < mFactor_f_Yaw)
                                s = (int)((mRatio_r_Yaw - 1.0f)/(mFactor_f_Yaw-1f) * 100f+0.5f);
                            else {
                                mRatio_r_Yaw = mFactor_f_Yaw;
                                s = 100;
                            }
                            NonLinearCoeffBuilding(s, CASE_Yaw);
                            textSeekBar.setText("                 ratio Yaw start Max       = " + String.format("%.2f",mFactor_f_Yaw)+"             so Yaw non linear start slope = " + String.format("%.2f",mRatio_r_Yaw));
                        }
                    }
                });
            }
        }
        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {
            Log.d("scrollText", "SeekBar : start"); // for debug
        }
        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {
            Log.d("scrollText", "SeekBar : end"); // for debug
            mEditor.putFloat("mCoeff_a_Yaw",mCoeff_a_Yaw);
            mEditor.putFloat("mCoeff_b_Yaw",mCoeff_b_Yaw);
            mEditor.putFloat("mCoeff_c_Yaw",mCoeff_c_Yaw);
            mEditor.putFloat("mRatio_r_Yaw",mRatio_r_Yaw);
            mEditor.putFloat("mFactor_f_Yaw",mFactor_f_Yaw);
            mEditor.commit();
        }
    };

    private SeekBar.OnSeekBarChangeListener mSeekBarListenerPitchRoll = new SeekBar.OnSeekBarChangeListener() {
        @Override
        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
        //progress	int: The current progress level. This will be in the range min..max where min
        //and max were set by setMin(int) and setMax(int), respectively. (The default values for min is 0 and max is 100.)
            {
                Log.d("scrollText", "SeekBar : progress value: "+String.valueOf(progress)); // for debug
                mProgressSeekBar = progress;

                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                    TextView textSeekBar = findViewById(R.id.textSeekBarPitchRoll);
                    if(RadioButtonSlope_standard)
                    {
                        mProgressSeekBar_PitchRoll = mProgressSeekBar;
                        NonLinearCoeffBuilding(mProgressSeekBar_PitchRoll, CASE_PitchRoll);
                        textSeekBar.setText("Pitch and Roll non linear start slope = " + String.format("%.2f",mRatio_r_xz));
                    }
                    else
                    {
                        int s;
                        if(mProgressSeekBar < 3)
                            mProgressSeekBar = 3;
                        mFactor_f_xz = (float) mProgressSeekBar;
                        if(mRatio_r_xz < mFactor_f_xz)
                            s = (int)((mRatio_r_xz - 1.0f)/(mFactor_f_xz-1f) * 100f+0.5f);
                        else {
                            mRatio_r_xz = mFactor_f_xz;
                            s = 100;
                        }
                        NonLinearCoeffBuilding(s, CASE_PitchRoll);
                        textSeekBar.setText("ratio Pitch and Roll start Max = " + String.format("%.2f",mFactor_f_xz)+" so Pitch and Roll non linear start slope = " + String.format("%.2f",mRatio_r_xz));
                    }
                    }
                });

            }
        }
        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {
            Log.d("scrollText", "SeekBar : start"); // for debug
        }
        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {
            Log.d("scrollText", "SeekBar : end"); // for debug
            mEditor.putFloat("mCoeff_a_xz",mCoeff_a_xz);
            mEditor.putFloat("mCoeff_b_xz",mCoeff_b_xz);
            mEditor.putFloat("mCoeff_c_xz",mCoeff_c_xz);
            mEditor.putFloat("mRatio_r_xz",mRatio_r_xz);
            mEditor.putFloat("mFactor_f_xz",mFactor_f_xz);
            mEditor.commit();
        }
    };

    private SeekBar.OnSeekBarChangeListener mSeekBarListenerThrottle = new SeekBar.OnSeekBarChangeListener() {
        @Override
        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
            //progress	int: The current progress level. This will be in the range min..max where min
            //and max were set by setMin(int) and setMax(int), respectively. (The default values for min is 0 and max is 100.)
            {
                Log.d("scrollText", "SeekBar : progress value: "+String.valueOf(progress)); // for debug
                mProgressSeekBar = progress;

                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                    TextView textSeekBar = findViewById(R.id.textSeekBarThrottle);
                    if(RadioButtonSlope_standard)
                    {
                        mProgressSeekBar_Throttle = mProgressSeekBar;
                        NonLinearCoeffBuilding(mProgressSeekBar_Throttle, CASE_Throttle);
                        textSeekBar.setText("     Throttle non linear start slope       = " + String.format("%.2f",mRatio_r_y));
                    }
                    else
                    {
                        int s;
                        if(mProgressSeekBar < 3)
                            mProgressSeekBar = 3;
                        mFactor_f_y = (float) mProgressSeekBar;
                        if(mRatio_r_y < mFactor_f_y)
                            s = (int)((mRatio_r_y - 1.0f)/(mFactor_f_y-1f) * 100f+0.5f);
                        else {
                            mRatio_r_y = mFactor_f_y;
                            s = 100;
                        }
                        NonLinearCoeffBuilding(s, CASE_Throttle);
                        textSeekBar.setText("           ratio Throttle start Max      = " + String.format("%.2f",mFactor_f_y)+"      so Throttle non linear start slope = " + String.format("%.2f",mRatio_r_y));
                    }
                    }
                });

            }
        }
        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {
            Log.d("scrollText", "SeekBar : start"); // for debug
        }
        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {
            Log.d("scrollText", "SeekBar : end"); // for debug
            mEditor.putFloat("mCoeff_a_y",mCoeff_a_y);
            mEditor.putFloat("mCoeff_b_y",mCoeff_b_y);
            mEditor.putFloat("mCoeff_c_y",mCoeff_c_y);
            mEditor.putFloat("mRatio_r_y",mRatio_r_y);
            mEditor.putFloat("mFactor_f_y",mFactor_f_y);
            mEditor.commit();
        }
    };

    void NonLinearCoeffBuilding(int seekValue, int axis_selector)
    {
        float factor_f,ratio_r,coeff_a,coeff_b,coeff_c;
        float s = (float) (seekValue);
        factor_f = 0;
        switch(axis_selector) {
            case -1:
                factor_f = mFactor_f_Yaw;
                break;
            case 0:
                factor_f = mFactor_f_y;
                break;
            case 1:
                factor_f = mFactor_f_xz;
                break;
        }
        ratio_r = 1.0f + s * 0.01f * (factor_f-1); // mRatio_r = ratio between slope at starting point non linear and linear  = [1 ... f], s = [0 ... 100]
        coeff_a = (ratio_r-1)*(factor_f-1)/((factor_f-2)*(factor_f-2));
        coeff_b = 1/(factor_f-2)-coeff_a/(factor_f-1);
        coeff_c = coeff_a*factor_f/(factor_f-1)-1/(factor_f-2);
        switch(axis_selector) {
            case -1:
                mRatio_r_Yaw = ratio_r;
                mCoeff_a_Yaw = coeff_a;
                mCoeff_b_Yaw = coeff_b;
                mCoeff_c_Yaw = coeff_c;
                break;
            case 0:
                mRatio_r_y = ratio_r;
                mCoeff_a_y = coeff_a;
                mCoeff_b_y = coeff_b;
                mCoeff_c_y = coeff_c;
                break;
            case 1:
                mRatio_r_xz = ratio_r;
                mCoeff_a_xz = coeff_a;
                mCoeff_b_xz = coeff_b;
                mCoeff_c_xz = coeff_c;
                break;
        }
    }


}



