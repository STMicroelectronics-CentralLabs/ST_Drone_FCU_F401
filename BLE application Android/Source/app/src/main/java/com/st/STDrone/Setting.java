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
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.RadioButton;

import com.st.BlueSTSDK.Manager;
import com.st.BlueSTSDK.Node;


import static com.st.STDrone.DefaultParameters.mDataDroneVisible_default;
import static com.st.STDrone.DefaultParameters.mFollowerCursor_default;
import static com.st.STDrone.DefaultParameters.mLogFile_default;
import static com.st.STDrone.DefaultParameters.mShapeCircle_default;
import static com.st.STDrone.DefaultParameters.mSwapCommandcode_default;
import static com.st.STDrone.DefaultParameters.mYawDisalble_default;

//public class Setting extends AppCompatActivity {
public class Setting  extends MyLogFeatureActivity {

//    private final static boolean majorButtonsDrawDisable = true;
//    private final static boolean sharedPreferencesEnable = false;
//    private final static boolean bleEnable = false;
    private final static boolean majorButtonsDrawDisable = false;
    private final static boolean sharedPreferencesEnable = true;
    private final static boolean bleEnable = true;

    private final static String NODE_TAG = Setting.class.getCanonicalName() + "" +
            ".NODE_TAG";

    private SettingParameter mInitParam;

    private RadioButton RadioButtonDataDrone_OFF;
    private RadioButton RadioButtonDataDrone_ON;
    private RadioButton RadioButtonLogFile_OFF;
    private RadioButton RadioButtonLogFile_ON;
    private RadioButton RadioButtonMobile_OFF;
    private RadioButton RadioButtonMobile_ON;
    private RadioButton RadioButtonYawDisable_OFF;
    private RadioButton RadioButtonYawDisable_ON;
    private RadioButton RadioButtonShape_OFF;
    private RadioButton RadioButtonShape_ON;

    private RadioButton RadioButtonSwapPlanes_down;
    private RadioButton RadioButtonSwapPlanes_zero;
    private RadioButton RadioButtonSwapPlanes_up;

    private final static int PLANE_XY = 0;
    private final static int PLANE_XZ = 1;
    private final static int PLANE_ZY = 2;
    private int mSwapCommandcode = PLANE_XZ; // default

    private Boolean mFollowerCursor = false;
    private Boolean mYawDisalble = false;
    private Boolean mDataDroneVisible = false;
    private Boolean mLogFile = false;
    private Boolean mShapeCircle = true;

    private static boolean mNode_status = false;

    private SharedPreferences.Editor mEditor;

    /**
     * create an intent for start this activity
     *
     * @param c    context used for create the intent
      * @return intent for start a demo activity that use the node as data source
     */
        public static Intent getStartIntent(Context c, @NonNull Node node) {
        Intent i = new Intent(c, Setting.class);
        if(bleEnable) {
            if (node == null) {
                mNode_status = false;
            } else {
                mNode_status = true;
                i.putExtra(NODE_TAG, node.getTag());
                i.putExtras(NodeContainerFragment.prepareArguments(node));
            }
        }
        else
            mNode_status = false;
        return i;
    }//getStartIntent


    void getParameters(SharedPreferences pref){
        mSwapCommandcode = pref.getInt("mSwapCommandcode", mSwapCommandcode_default);
        mDataDroneVisible = pref.getBoolean("mDataDroneVisible", mDataDroneVisible_default);
        mFollowerCursor = pref.getBoolean("mFollowerCursor", mFollowerCursor_default);
        mYawDisalble = pref.getBoolean("mYawDisalble", mYawDisalble_default);
        mLogFile = pref.getBoolean("mLogFile", mLogFile_default);
        mShapeCircle = pref.getBoolean("mShapeCircle", mShapeCircle_default);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_setting);

        if(sharedPreferencesEnable) {
            SharedPreferences pref = getApplicationContext().getSharedPreferences("MyPref", MODE_PRIVATE);
            mEditor = pref.edit();
            getParameters(pref);
        }

        if(majorButtonsDrawDisable) {
            LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonSensor);
            selectedLinearLayout.setVisibility(View.INVISIBLE);
            selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtons);
            selectedLinearLayout.setVisibility(View.INVISIBLE);
            selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtons2);
            selectedLinearLayout.setVisibility(View.INVISIBLE);
            selectedLinearLayout = findViewById(R.id.LinearLayout_Sensitivity);
            selectedLinearLayout.setVisibility(View.INVISIBLE);
            Button selectedButton = findViewById(R.id.Help);
            selectedButton.setVisibility(View.INVISIBLE);
        }

        if(!majorButtonsDrawDisable){
            LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonSensor);
            if(!mNode_status)
            {
                mLogFile = false;
                mEditor.putBoolean("mLogFile",mLogFile);
                mEditor.commit();
                selectedLinearLayout.setVisibility(View.INVISIBLE);
            }
            else{
                String nodeTag = getIntent().getStringExtra(NODE_TAG);
                final Node n = Manager.getSharedInstance().getNodeWithTag(nodeTag);
                if (n.getState() != Node.State.Connected){
                    mLogFile = false;
                    mEditor.putBoolean("mLogFile",mLogFile);
                    mEditor.commit();
                    selectedLinearLayout.setVisibility(View.INVISIBLE);
                }
                else
                {
                    selectedLinearLayout.setVisibility(View.VISIBLE);

                    RadioButtonDataDrone_OFF = findViewById(R.id.RadioButtonDataDrone_OFF);
                    RadioButtonDataDrone_ON = findViewById(R.id.RadioButtonDataDrone_ON);
                    if(mDataDroneVisible)
                        RadioButtonDataDrone_ON.setChecked(true);
                    else
                        RadioButtonDataDrone_OFF.setChecked(true);

                    RadioButtonLogFile_OFF = findViewById(R.id.RadioButtonLogFile_OFF);
                    RadioButtonLogFile_ON = findViewById(R.id.RadioButtonLogFile_ON);
                    if(mLogFile)
                        RadioButtonLogFile_ON.setChecked(true);
                    else
                        RadioButtonLogFile_OFF.setChecked(true);
                }
            }

            RadioButtonMobile_OFF = findViewById(R.id.RadioButtonMobile_OFF);
            RadioButtonMobile_ON = findViewById(R.id.RadioButtonMobile_ON);
            if(mFollowerCursor)
                RadioButtonMobile_ON.setChecked(true);
            else
                RadioButtonMobile_OFF.setChecked(true);

            RadioButtonYawDisable_OFF = findViewById(R.id.RadioButtonYawDisable_OFF);
            RadioButtonYawDisable_ON = findViewById(R.id.RadioButtonYawDisable_ON);
            if(mYawDisalble)
                RadioButtonYawDisable_ON.setChecked(true);
            else
                RadioButtonYawDisable_OFF.setChecked(true);

            RadioButtonSwapPlanes_down = findViewById(R.id.RadioButtonSwapPlanes_Down);
            RadioButtonSwapPlanes_zero = findViewById(R.id.RadioButtonSwapPlanes_zero);
            RadioButtonSwapPlanes_up = findViewById(R.id.RadioButtonSwapPlanes_UP);
            switch(mSwapCommandcode) {
                case PLANE_XZ:
                    RadioButtonSwapPlanes_down.setChecked(true);
                    break;
                case PLANE_XY:
                    RadioButtonSwapPlanes_zero.setChecked(true);
                    break;
                case PLANE_ZY:
                    RadioButtonSwapPlanes_up.setChecked(true);
                    break;
            }

            RadioButtonShape_OFF = findViewById(R.id.RadioButtonShape_OFF);
            RadioButtonShape_ON = findViewById(R.id.RadioButtonShape_ON);
            if(mShapeCircle)
                RadioButtonShape_OFF.setChecked(true);
            else
                RadioButtonShape_ON.setChecked(true);
        }


        if(!majorButtonsDrawDisable) {
            findViewById(R.id.Sensitivity).setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View view) {

                    final Node n;
                    if (!mNode_status) {
                        n = null;
                    } else {
                        String nodeTag = getIntent().getStringExtra(NODE_TAG);
                        n = Manager.getSharedInstance().getNodeWithTag(nodeTag);
                    }
                    Intent i = Sensitivity.getStartIntent(Setting.this, n);
                    startActivity(i);
                }
            });
        }

        findViewById(R.id.Home).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                final Node n;
                if(!mNode_status)
                {
                    n = null;
                }
                else
                {
                    String nodeTag = getIntent().getStringExtra(NODE_TAG);
                    n = Manager.getSharedInstance().getNodeWithTag(nodeTag);
                }
                Intent i = Joystick.getStartIntent(Setting.this,n);
                startActivity(i);
            }
        });


        if(!majorButtonsDrawDisable) {
            findViewById(R.id.Help).setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View view) {

                    final Node n;
                    if (!mNode_status) {
                        n = null;
                    } else {
                        String nodeTag = getIntent().getStringExtra(NODE_TAG);
                        n = Manager.getSharedInstance().getNodeWithTag(nodeTag);
                    }
                    Intent i = Help.getStartIntent(Setting.this, n);
                    startActivity(i);
                }
            });
        }
    }


    @Override
    protected void onPause() {
        Log.d("notification", this+" onPauseSetting "); // for debug
        super.onPause();
    }

    @Override
    protected void onStop() {
        Log.d("notification", this+" onStopSetting "); // for debug
        super.onStop();
    }

    @Override
    protected void onDestroy() {
        Log.d("notification", this+" onDestroySetting "); // for debug
        super.onDestroy();
    }

    public void onRadioButtonSwapPlanes(View view) {
        // Is the button now checked?
        boolean checked = ((RadioButton) view).isChecked();

        // Check which radio button was clicked
        switch(view.getId()) {
            case R.id.RadioButtonSwapPlanes_Down:
                if (checked) {
                    mSwapCommandcode = PLANE_XZ;
                }
                break;
            case R.id.RadioButtonSwapPlanes_zero:
                if (checked) {
                    mSwapCommandcode = PLANE_XY;
                }
                break;
            case R.id.RadioButtonSwapPlanes_UP:
                if (checked) {
                    mSwapCommandcode = PLANE_ZY;
                }
                break;
        }

        mEditor.putInt("mSwapCommandcode",mSwapCommandcode);
        mEditor.commit();
    }


    public void onRadioButtonDataDrone(View view) {
        // Is the button now checked?
        boolean checked = ((RadioButton) view).isChecked();

        // Check which radio button was clicked
        switch(view.getId()) {
            case R.id.RadioButtonDataDrone_OFF:
                if (checked) {
                    mDataDroneVisible = false;
                }
                break;
            case R.id.RadioButtonDataDrone_ON:
                if (checked) {
                    mDataDroneVisible = true;
                }
                break;
        }
        mEditor.putBoolean("mDataDroneVisible",mDataDroneVisible);
        mEditor.commit();
    }


    public void onRadioButtonLogFile(View view) {
        // Is the button now checked?
        boolean checked = ((RadioButton) view).isChecked();
        String nodeTag = getIntent().getStringExtra(NODE_TAG);
        final Node n = Manager.getSharedInstance().getNodeWithTag(nodeTag);
        putNode(n);
        // Check which radio button was clicked
        switch(view.getId()) {
            case R.id.RadioButtonLogFile_OFF:
                if (checked) {
                    mLogFile = false;
                    stopLogging();
                }
                break;
            case R.id.RadioButtonLogFile_ON:
                if (checked) {
                    mLogFile = true;
                    startLogging();
                }
                break;
        }
        mEditor.putBoolean("mLogFile",mLogFile);
        mEditor.commit();
    }


    public void onRadioButtonMobile(View view) {
        // Is the button now checked?
        boolean checked = ((RadioButton) view).isChecked();

        // Check which radio button was clicked
        switch(view.getId()) {
            case R.id.RadioButtonMobile_OFF:
                if (checked) {
                    mFollowerCursor = false;
                }
                break;
            case R.id.RadioButtonMobile_ON:
                if (checked) {
                    mFollowerCursor = true;
                }
                break;
        }
        mEditor.putBoolean("mFollowerCursor",mFollowerCursor);
        mEditor.commit();
    }


    public void onRadioButtonYawDisable(View view) {
        // Is the button now checked?
        boolean checked = ((RadioButton) view).isChecked();

        // Check which radio button was clicked
        switch(view.getId()) {
            case R.id.RadioButtonYawDisable_OFF:
                if (checked) {
                    mYawDisalble = false;
                }
                break;
            case R.id.RadioButtonYawDisable_ON:
                if (checked) {
                    mYawDisalble = true;
                }
                break;
        }
        mEditor.putBoolean("mYawDisalble",mYawDisalble);
        mEditor.commit();
    }


    public void onRadioButtonShape(View view) {
        // Is the button now checked?
        boolean checked = ((RadioButton) view).isChecked();

        // Check which radio button was clicked
        switch(view.getId()) {
            case R.id.RadioButtonShape_OFF:
                if (checked) {
                    mShapeCircle = true;
                }
                break;
            case R.id.RadioButtonShape_ON:
                if (checked) {
                    mShapeCircle = false;
                }
                break;
        }
        mEditor.putBoolean("mShapeCircle",mShapeCircle);
        mEditor.commit();
    }

}



