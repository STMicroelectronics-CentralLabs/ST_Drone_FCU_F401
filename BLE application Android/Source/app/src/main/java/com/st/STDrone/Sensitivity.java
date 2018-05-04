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
import android.support.annotation.NonNull;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.MenuItem;
import android.view.View;
import android.widget.RadioButton;
import android.widget.SeekBar;
import android.widget.TextView;

import com.st.BlueSTSDK.Manager;
import com.st.BlueSTSDK.Node;

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

    public void ProgressBarUpdate(){
        mProgressSeekBar_Yaw = (int)((mRatio_r_Yaw - 1.0f)/(mFactor_f_Yaw-1f) * 100f+0.5f);
        mProgressSeekBar_PitchRoll = (int)((mRatio_r_xz - 1.0f)/(mFactor_f_xz-1f) * 100f+0.5f);
        mProgressSeekBar_Throttle = (int)((mRatio_r_y - 1.0f)/(mFactor_f_y-1f) * 100f+0.5f);
        TextView textSeekBar = findViewById(R.id.textSeekBarYaw);
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



