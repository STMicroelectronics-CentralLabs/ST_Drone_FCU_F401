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
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.FrameLayout;
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

import static com.st.STDrone.CoordConverter.mWidthPixels;
import static com.st.STDrone.CoordConverter.mHeightPixels;


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

    private static int mWidthHome = 0;
    private static int mHeightHome = 0;
    private static int mWidthHelp = 0;
    private static int mHeightHelp = 0;
    private static int mWidthRadioButtonMobile = 0;
    private static int mHeightRadioButtonMobile = 0;
    private static int mWidthRadioButtonYawDisable = 0;
    private static int mHeightRadioButtonYawDisable = 0;
    private static int mWidthRadioButtonSwapPlanes = 0;
    private static int mHeightRadioButtonSwapPlanes = 0;
    private static int mWidthRadioButtonShape = 0;
    private static int mHeightRadioButtonShape = 0;
    private static int mWidthSensitivity = 0;
    private static int mHeightSensitivity = 0;
    private static int mWidthRadioButtonDataDrone = 0;
    private static int mHeightRadioButtonDataDrone = 0;
    private static int mWidthRadioButtonLogFile = 0;
    private static int mHeightRadioButtonLogFile = 0;
    private static int mNumObjectsTot = 9;
    private static int mNumObjects = 0;
    private static boolean mFirstTime = true;
    private static int mLeftHome = 0;
    private static int mTopHome = 0;
    private static int mLeftHelp = 0;
    private static int mTopHelp = 0;
    private static int mLeftRadioButtonMobile = 0;
    private static int mTopRadioButtonMobile = 0;
    private static int mLeftRadioButtonYawDisable = 0;
    private static int mTopRadioButtonYawDisable = 0;
    private static int mLeftRadioButtonSwapPlanes = 0;
    private static int mTopRadioButtonSwapPlanes = 0;
    private static int mLeftRadioButtonShape = 0;
    private static int mTopRadioButtonShape = 0;
    private static int mLeftSensitivity = 0;
    private static int mTopSensitivity = 0;
    private static int mLeftRadioButtonDataDrone = 0;
    private static int mTopRadioButtonDataDrone = 0;
    private static int mLeftRadioButtonLogFile = 0;
    private static int mTopRadioButtonLogFile = 0;

    private static float mScale;


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

//        if(majorButtonsDrawDisable) {
//            LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonSensor);
//            selectedLinearLayout.setVisibility(View.INVISIBLE);
//            selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtons);
//            selectedLinearLayout.setVisibility(View.INVISIBLE);
//            selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtons2);
//            selectedLinearLayout.setVisibility(View.INVISIBLE);
//            selectedLinearLayout = findViewById(R.id.LinearLayout_Sensitivity);
//            selectedLinearLayout.setVisibility(View.INVISIBLE);
//            Button selectedButton = findViewById(R.id.Help);
//            selectedButton.setVisibility(View.INVISIBLE);
//        }

        if(!majorButtonsDrawDisable){
            LinearLayout selectedLinearLayoutLogFile = findViewById(R.id.LinearLayout_RadioButtonLogFile);
            LinearLayout selectedLinearLayoutDataDrone = findViewById(R.id.LinearLayout_RadioButtonDataDrone);
            if(!mNode_status)
            {
                mLogFile = false;
                mEditor.putBoolean("mLogFile",mLogFile);
                mEditor.commit();
                selectedLinearLayoutLogFile.setVisibility(View.INVISIBLE);
                selectedLinearLayoutDataDrone.setVisibility(View.INVISIBLE);
            }
            else{
                String nodeTag = getIntent().getStringExtra(NODE_TAG);
                final Node n = Manager.getSharedInstance().getNodeWithTag(nodeTag);
                if (n.getState() != Node.State.Connected){
                    mLogFile = false;
                    mEditor.putBoolean("mLogFile",mLogFile);
                    mEditor.commit();
                    selectedLinearLayoutLogFile.setVisibility(View.INVISIBLE);
                    selectedLinearLayoutDataDrone.setVisibility(View.INVISIBLE);
                }
                else
                {
                    selectedLinearLayoutLogFile.setVisibility(View.VISIBLE);
                    selectedLinearLayoutDataDrone.setVisibility(View.VISIBLE);

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

    void movePos(){
        FrameLayout.LayoutParams params;

        Button selectedButton = findViewById(R.id.Home);
        params = (FrameLayout.LayoutParams) selectedButton.getLayoutParams();
        params.setMargins(mLeftHome, mTopHome, params.rightMargin, params.bottomMargin);
        selectedButton.setLayoutParams(params);
        selectedButton.setScaleX(mScale);
        selectedButton.setScaleY(mScale);

        selectedButton = findViewById(R.id.Help);
        params = (FrameLayout.LayoutParams) selectedButton.getLayoutParams();
        params.setMargins(mLeftHelp, mTopHelp, params.rightMargin, params.bottomMargin);
        selectedButton.setLayoutParams(params);
        selectedButton.setScaleX(mScale);
        selectedButton.setScaleY(mScale);

        LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonMobile);
        params = (FrameLayout.LayoutParams) selectedLinearLayout.getLayoutParams();
        params.setMargins(mLeftRadioButtonMobile, mTopRadioButtonMobile, params.rightMargin, params.bottomMargin);
        selectedLinearLayout.setLayoutParams(params);
        selectedLinearLayout.setScaleX(mScale);
        selectedLinearLayout.setScaleY(mScale);

        selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonYawDisable);
        params = (FrameLayout.LayoutParams) selectedLinearLayout.getLayoutParams();
        params.setMargins(mLeftRadioButtonYawDisable, mTopRadioButtonYawDisable, params.rightMargin, params.bottomMargin);
        selectedLinearLayout.setLayoutParams(params);
        selectedLinearLayout.setScaleX(mScale);
        selectedLinearLayout.setScaleY(mScale);

        selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonSwapPlanes);
        params = (FrameLayout.LayoutParams) selectedLinearLayout.getLayoutParams();
        params.setMargins(mLeftRadioButtonSwapPlanes, mTopRadioButtonSwapPlanes, params.rightMargin, params.bottomMargin);
        selectedLinearLayout.setLayoutParams(params);
        selectedLinearLayout.setScaleX(mScale);
        selectedLinearLayout.setScaleY(mScale);

        selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonShape);
        params = (FrameLayout.LayoutParams) selectedLinearLayout.getLayoutParams();
        params.setMargins(mLeftRadioButtonShape, mTopRadioButtonShape, params.rightMargin, params.bottomMargin);
        selectedLinearLayout.setLayoutParams(params);
        selectedLinearLayout.setScaleX(mScale);
        selectedLinearLayout.setScaleY(mScale);

        selectedButton = findViewById(R.id.Sensitivity);
        params = (FrameLayout.LayoutParams) selectedButton.getLayoutParams();
        params.setMargins(mLeftSensitivity, mTopSensitivity, params.rightMargin, params.bottomMargin);
        selectedButton.setLayoutParams(params);
        selectedButton.setScaleX(mScale);
        selectedButton.setScaleY(mScale);

        selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonDataDrone);
        params = (FrameLayout.LayoutParams) selectedLinearLayout.getLayoutParams();
        params.setMargins(mLeftRadioButtonDataDrone, mTopRadioButtonDataDrone, params.rightMargin, params.bottomMargin);
        selectedLinearLayout.setLayoutParams(params);
        selectedLinearLayout.setScaleX(mScale);
        selectedLinearLayout.setScaleY(mScale);

        selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonLogFile);
        params = (FrameLayout.LayoutParams) selectedLinearLayout.getLayoutParams();
        params.setMargins(mLeftRadioButtonLogFile, mTopRadioButtonLogFile, params.rightMargin, params.bottomMargin);
        selectedLinearLayout.setLayoutParams(params);
        selectedLinearLayout.setScaleX(mScale);
        selectedLinearLayout.setScaleY(mScale);

    }

    void initPos(){

        int space;

        space = mWidthPixels - mWidthHome - mWidthHelp;
        space /= 3;
        float avgSizeObj = (mWidthHome+mWidthHelp)*0.5f;
        mScale = space/avgSizeObj;
        Log.d("position", "mScale init "+mScale); // for debug

        space = mHeightPixels - mHeightHome - mHeightRadioButtonMobile - mHeightRadioButtonSwapPlanes - mHeightSensitivity - mHeightRadioButtonDataDrone;
        space /= 6;
        float avgSizeObjy = (mHeightHome+mHeightRadioButtonMobile+mHeightRadioButtonSwapPlanes+mHeightSensitivity+mHeightRadioButtonDataDrone)/5.0f;
        float scaley = space/avgSizeObjy*1.5f;
        Log.d("position", "scaley init "+scaley); // for debug
        avgSizeObjy = CoordConverter.fromPx_to_dp((int)avgSizeObjy);
        Log.d("position", "avgSizeObjy init "+avgSizeObjy); // for debug
        if(scaley < mScale)
            mScale = scaley;
        if(mScale < 1.0f)
            mScale = 1.0f;
        float heightMax = 75.0f;
        if((mScale*avgSizeObjy) > heightMax)
            mScale = heightMax/avgSizeObjy;
        //mScale = 1.0f;
        Log.d("position", "mScale init final "+mScale); // for debug

        // the margin is the same also after scale, but scale changes the object size
        // so the margin after scale = margin desidered = margin before scale - (scale-1)size*0.5
        // so margin before scale = margin desidered + (scale-1)size*0.5
        // so margin desidered = first space
        // in fact
        // margin before scale = space + (scale-1)size*0.5
        // center:
        // center = margin before scale + size*0.5
        // or
        // center = space + scale*size*0.5
        // so margin before scale + size*0.5 = space + scale*size*0.5
        // so margin before scale = space + (scale-1)*size*0.5

        // the next margin needs an offset
        // margin desidered = Sum (previous margin desidered) + current margin desidered respect the previous object
        // so margin desidered = Sum (previous object size) + Sum (previous+current margin desidered)
        // so margin before scale = Sum (previous object size) + Sum (previous+current space desidered) + (scale-1)size*0.5
        float offset;
        int numSpace;

// to impose same space per each row, but each row has a different space
        // in general
        // offsetTrue = Sum(previous space)+scale*Sum (previous object size)
        // so next object n into same row becomes:
        // margin before scalen = offsetTrue + space + (scale-1)*sizen*0.5
        // so margin before scalen = Sum(previous space)+scale*Sum (previous object size) + space + (scale-1)*sizen*0.5
        // by convention offset = scale*Sum (previous object size)
        // so margin before scalen = Sum(previous space)+ offset + space + (scale-1)*sizen*0.5
        // and space are the same
        // so margin before scalen = offset + space*numSpace + (scale-1)*sizen*0.5

//        offset = 0;
//        numSpace = 1;
//        space = (int) (mWidthPixels - mWidthHome*mScale - mWidthHelp*mScale + 0.5f);
//        space /= 3;
//        mLeftHome = (int) (numSpace*space+mWidthHome*(mScale-1.0f)*0.5 + 0.5f);
//        offset += mWidthHome*mScale;
//        numSpace++;
//        mLeftHelp = (int) (offset+mWidthHelp*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
//
//        offset = 0;
//        numSpace = 1;
//        space = (int) (mWidthPixels - mWidthRadioButtonYawDisable*mScale - mWidthRadioButtonMobile*mScale + 0.5f);
//        space /= 3;
//        mLeftRadioButtonMobile = (int) (numSpace*space+mWidthRadioButtonMobile*(mScale-1.0f)*0.5 + 0.5f);
//        offset += mWidthRadioButtonMobile*mScale;
//        numSpace++;
//        mLeftRadioButtonYawDisable = (int) (offset+mWidthRadioButtonYawDisable*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
//
//        offset = 0;
//        numSpace = 1;
//        space = (int) (mWidthPixels - mWidthRadioButtonSwapPlanes*mScale - mWidthRadioButtonShape*mScale + 0.5f);
//        space /= 3;
//        mLeftRadioButtonSwapPlanes = (int) (offset+mWidthRadioButtonSwapPlanes*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
//        offset += mWidthRadioButtonSwapPlanes*mScale;
//        numSpace++;
//        mLeftRadioButtonShape = (int) (offset+mWidthRadioButtonShape*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
//
//        offset = 0;
//        numSpace = 1;
//        space = (int) (mWidthPixels - mWidthSensitivity*mScale + 0.5f);
//        space /= 2;
//        mLeftSensitivity = (int) (offset+mWidthSensitivity*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
//
//        offset = 0;
//        numSpace = 1;
//        space = (int) (mWidthPixels - mWidthRadioButtonDataDrone*mScale - mWidthRadioButtonLogFile*mScale + 0.5f);
//        space /= 3;
//        mLeftRadioButtonDataDrone = (int) (offset+mWidthRadioButtonDataDrone*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
//        offset += mWidthRadioButtonDataDrone*mScale;
//        numSpace++;
//        mLeftRadioButtonLogFile = (int) (offset+mWidthRadioButtonLogFile*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);


// to impose same center of object1 to object2 (also space2 is unknown)

        // margin before scale1 = space1 + (scale-1)size1*0.5
        // margin before scale2 = space2 + (scale-1)size2*0.5
        // center:
        // center1 = margin before scale1 + size1*0.5
        // center2 = margin before scale2 + size2*0.5
        // or
        // center1 = space1 + scale*size1*0.5
        // center2 = space2 + scale*size2*0.5
        // so space2 = space1 +  scale*(size1-size2)*0.5
        // and margin before scale2 = space2 + (scale-1)*size2*0.5
        // by convention offset = offsetTrue
        // offset = space1+scale*size1  // it is like the offsetTrue of the reference row
        // so next object3 into same row becomes:
        // space3 = space1 +  scale*(size1-size3)*0.5
        // and margin before scale3 = offset + space3 + (scale-1)*size3*0.5
        // in general
        // offset = Sum(previous space of row1)+scale*Sum(previous object size of row1)  // it is like the offsetTrue of the reference row
        // so next object n into same row becomes:
        // spacen = space1 +  scale*(size1-sizen)*0.5
        // and margin before scalen = offset + spacen + (scale-1)*sizen*0.5
        int spacen;

        //worst case with lower space
        offset = 0;
        numSpace = 1;
        space = (int) (mWidthPixels - mWidthRadioButtonSwapPlanes*mScale - mWidthRadioButtonShape*mScale + 0.5f);
        space /= 3;
        mLeftRadioButtonSwapPlanes = (int) (offset+mWidthRadioButtonSwapPlanes*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
        offset += mWidthRadioButtonSwapPlanes*mScale;
        numSpace++;
        mLeftRadioButtonShape = (int) (offset+mWidthRadioButtonShape*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);

        offset = 0;
        spacen = (int) (space +  mScale*(mWidthRadioButtonSwapPlanes-mWidthHome)*0.5f);
        mLeftHome = (int) (offset + spacen + (mScale-1.0f)*mWidthHome*0.5+ 0.5f);
        offset += mWidthRadioButtonSwapPlanes*mScale+space;
        spacen = (int) (space +  mScale*(mWidthRadioButtonShape-mWidthHelp)*0.5f);
        mLeftHelp = (int) (offset + spacen + (mScale-1.0f)*mWidthHelp*0.5+ 0.5f);

        offset = 0;
        spacen = (int) (space +  mScale*(mWidthRadioButtonSwapPlanes-mWidthRadioButtonMobile)*0.5f);
        mLeftRadioButtonMobile = (int) (offset + spacen + (mScale-1.0f)*mWidthRadioButtonMobile*0.5+ 0.5f);
        offset += mWidthRadioButtonSwapPlanes*mScale+space;
        spacen = (int) (space +  mScale*(mWidthRadioButtonShape-mWidthRadioButtonYawDisable)*0.5f);
        mLeftRadioButtonYawDisable = (int) (offset + spacen + (mScale-1.0f)*mWidthRadioButtonYawDisable*0.5+ 0.5f);

        offset = 0;
        spacen = (int) (space +  mScale*(mWidthRadioButtonSwapPlanes-mWidthRadioButtonDataDrone)*0.5f);
        mLeftRadioButtonDataDrone = (int) (offset + spacen + (mScale-1.0f)*mWidthRadioButtonDataDrone*0.5+ 0.5f);
        offset += mWidthRadioButtonSwapPlanes*mScale+space;
        spacen = (int) (space +  mScale*(mWidthRadioButtonShape-mWidthRadioButtonLogFile)*0.5f);
        mLeftRadioButtonLogFile = (int) (offset + spacen + (mScale-1.0f)*mWidthRadioButtonLogFile*0.5+ 0.5f);

        // space change
        offset = 0;
        numSpace = 1;
        spacen = (int) (mWidthPixels - mWidthSensitivity*mScale + 0.5f);
        spacen /= 2;
        mLeftSensitivity = (int) (offset+mWidthSensitivity*(mScale-1.0f)*0.5 +numSpace*spacen + 0.5f);


        FrameLayout selectedFrameLayout = findViewById(R.id.FrameLayout);
        int top = selectedFrameLayout.getHeight();
        //Log.d("position", "top init pixel "+top); // for debug
        //Log.d("position", "top init "+top); // for debug
        //Log.d("position", "mHeightPixels init "+mHeightPixels); // for debug
         mHeightPixels = top; // the top menu reduces the screen Height

        offset = 0;
        numSpace = 1;
        space = (int) (mHeightPixels - mHeightHome*mScale - mHeightRadioButtonMobile*mScale - mHeightRadioButtonSwapPlanes*mScale - mHeightSensitivity*mScale - mHeightRadioButtonDataDrone*mScale + 0.5f);
        space /= 6;
        mTopHome = (int) (offset+mHeightHome*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
        mTopHelp = mTopHome;
        offset += mHeightHome*mScale;
        numSpace++;
        mTopRadioButtonMobile = (int) (offset+mHeightRadioButtonMobile*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
        mTopRadioButtonYawDisable = mTopRadioButtonMobile;
        offset += mHeightRadioButtonMobile*mScale;
        numSpace++;
        mTopRadioButtonSwapPlanes = (int) (offset+mHeightRadioButtonSwapPlanes*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
        mTopRadioButtonShape = mTopRadioButtonSwapPlanes;
        offset += mHeightRadioButtonSwapPlanes*mScale;
        numSpace++;
        mTopSensitivity = (int) (offset+mHeightSensitivity *(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
        offset += mHeightSensitivity*mScale;
        numSpace++;
        mTopRadioButtonDataDrone = (int) (offset+mHeightRadioButtonDataDrone*(mScale-1.0f)*0.5 +numSpace*space + 0.5f);
        mTopRadioButtonLogFile = mTopRadioButtonDataDrone;

        movePos();

        mFirstTime = false;
    }

    void readSize(){
        Button selectedButton = findViewById(R.id.Home);
        selectedButton.post(new Runnable() {
            @Override
            public void run() {
                Button selectedButton = findViewById(R.id.Home);
                mWidthHome = selectedButton.getWidth();
                mHeightHome = selectedButton.getHeight();
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedButton = findViewById(R.id.Help);
        selectedButton.post(new Runnable() {
            @Override
            public void run() {
                Button selectedButton = findViewById(R.id.Help);
                mWidthHelp = selectedButton.getWidth();
                mHeightHelp = selectedButton.getHeight();
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonMobile);
        selectedLinearLayout.post(new Runnable() {
            @Override
            public void run() {
                LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonMobile);
                mWidthRadioButtonMobile = selectedLinearLayout.getWidth();
                mHeightRadioButtonMobile = selectedLinearLayout.getHeight();
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonYawDisable);
        selectedLinearLayout.post(new Runnable() {
            @Override
            public void run() {
                LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonYawDisable);
                mWidthRadioButtonYawDisable = selectedLinearLayout.getWidth();
                mHeightRadioButtonYawDisable = selectedLinearLayout.getHeight();
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonSwapPlanes);
        selectedLinearLayout.post(new Runnable() {
            @Override
            public void run() {
                LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonSwapPlanes);
                mWidthRadioButtonSwapPlanes = selectedLinearLayout.getWidth();
                mHeightRadioButtonSwapPlanes = selectedLinearLayout.getHeight();
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonShape);
        selectedLinearLayout.post(new Runnable() {
            @Override
            public void run() {
                LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonShape);
                mWidthRadioButtonShape = selectedLinearLayout.getWidth();
                mHeightRadioButtonShape = selectedLinearLayout.getHeight();
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });


        selectedButton = findViewById(R.id.Sensitivity);
        selectedButton.post(new Runnable() {
            @Override
            public void run() {
                Button selectedButton = findViewById(R.id.Sensitivity);
                mWidthSensitivity = selectedButton.getWidth();
                mHeightSensitivity = selectedButton.getHeight();
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonDataDrone);
        selectedLinearLayout.post(new Runnable() {
            @Override
            public void run() {
                LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonDataDrone);
                mWidthRadioButtonDataDrone = selectedLinearLayout.getWidth();
                mHeightRadioButtonDataDrone = selectedLinearLayout.getHeight();
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });

        selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonLogFile);
        selectedLinearLayout.post(new Runnable() {
            @Override
            public void run() {
                LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_RadioButtonLogFile);
                mWidthRadioButtonLogFile = selectedLinearLayout.getWidth();
                mHeightRadioButtonLogFile = selectedLinearLayout.getHeight();
                mNumObjects++;
                if(mNumObjects == mNumObjectsTot)
                    initPos();
            }
        });
    }

    @Override
    protected void onStart() {
        Log.d("notification", this+" onStartSetting "); // for debug
        super.onStart();

        if(mFirstTime)
        {
            readSize();
        }
        //else //Samsung requires always movePos to have a correct readSize()
            movePos();
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



