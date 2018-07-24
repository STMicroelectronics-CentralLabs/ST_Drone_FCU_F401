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
import android.content.res.Configuration;
import android.graphics.Rect;
import android.os.Build;
import android.os.Vibrator;
import android.os.Bundle;
import android.support.annotation.NonNull;
import android.support.v7.app.AppCompatActivity;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.TextView;

import com.st.BlueSTSDK.Feature;
import com.st.BlueSTSDK.Features.FeatureAcceleration;
import com.st.BlueSTSDK.Features.FeatureGyroscope;
import com.st.BlueSTSDK.Features.FeatureHumidity;
import com.st.BlueSTSDK.Features.FeatureMagnetometer;
import com.st.BlueSTSDK.Features.FeaturePressure;
import com.st.BlueSTSDK.Features.FeatureSwitch;
import com.st.BlueSTSDK.Features.FeatureTemperature;
import com.st.BlueSTSDK.Manager;
import com.st.BlueSTSDK.Node;
import com.st.BlueSTSDK.Utils.UUIDToFeatureMap;


import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.UUID;

import static android.graphics.Color.argb;
import static com.st.STDrone.DefaultParameters.mCoeff_a_Yaw_default;
import static com.st.STDrone.DefaultParameters.mCoeff_a_xz_default;
import static com.st.STDrone.DefaultParameters.mCoeff_a_y_default;
import static com.st.STDrone.DefaultParameters.mCoeff_b_Yaw_default;
import static com.st.STDrone.DefaultParameters.mCoeff_b_xz_default;
import static com.st.STDrone.DefaultParameters.mCoeff_b_y_default;
import static com.st.STDrone.DefaultParameters.mCoeff_c_Yaw_default;
import static com.st.STDrone.DefaultParameters.mCoeff_c_xz_default;
import static com.st.STDrone.DefaultParameters.mCoeff_c_y_default;
import static com.st.STDrone.DefaultParameters.mDataDroneVisible_default;
import static com.st.STDrone.DefaultParameters.mFactor_f_Yaw_default;
import static com.st.STDrone.DefaultParameters.mFactor_f_xz_default;
import static com.st.STDrone.DefaultParameters.mFactor_f_y_default;
import static com.st.STDrone.DefaultParameters.mFollowerCursor_default;
import static com.st.STDrone.DefaultParameters.mLogFile_default;
import static com.st.STDrone.DefaultParameters.mRatio_r_Yaw_default;
import static com.st.STDrone.DefaultParameters.mRatio_r_xz_default;
import static com.st.STDrone.DefaultParameters.mRatio_r_y_default;
import static com.st.STDrone.DefaultParameters.mShapeCircle_default;
import static com.st.STDrone.DefaultParameters.mSwapCommandcode_default;
import static com.st.STDrone.DefaultParameters.mYawDisalble_default;
import static java.lang.StrictMath.sqrt;

public class Joystick extends AppCompatActivity {

    private int mVersion = 37;

    private final static boolean graphicsEnable = true;
    private final static boolean lastFeatureEnable = false;
//    private final static boolean cursorDrawEnable = false;
//    private final static boolean majorButtonsDrawDisable = true;
//    private final static boolean bleEnable = false;
    private final static boolean cursorDrawEnable = true;
    private final static boolean majorButtonsDrawDisable = false;
    private final static boolean bleEnable = true;

    //========================================================================================
    // joyistick BLE

    /**
     * tag used for retrieve the NodeContainerFragment
     */
    private final static String NODE_FRAGMENT = Joystick.class.getCanonicalName() + "" +
            ".NODE_FRAGMENT";

    /**
     * tag used for store the node id that permit us to find the node selected by the user
     */
    private final static String NODE_TAG = Joystick.class.getCanonicalName() + "" +
            ".NODE_TAG";

    /**
     * node that will stream the data
     */
    private Node mNode;

    /**
     * fragment that manage the node connection and avoid a re connection each time the activity
     * is recreated
     */
    private NodeContainerFragment mNodeContainer;


    private void StatusConnectionVisualization(){
        if(!mFirstTime)
        {
            if (mNode.getState() == Node.State.Connected) {

                Joystick.this.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Button textViewConnect = findViewById(R.id.Button_Connection);
                        textViewConnect.setText(" Connected ");
                        mySetColorButton(textViewConnect,argb(255,0,255,0));
                    }//run
                });

                mFeatureJoystick = mNode.getFeature(FeatureJoystick.class);
                if(mFeatureJoystick == null){
                    Joystick.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Button textViewConnect = findViewById(R.id.Button_Connection);
                            textViewConnect.setText(" Joystick not found Press me again ");
                            mySetColorButton(textViewConnect,argb(255,255,0,0));
                        }//run
                    });
                }
                else
                {
                    mAnalogBLEQuequeTask.cancel();

                    mAnalogBLEQuequeTask = new myAnalogBLEQuequeTask();

                    mScrollTimer.scheduleAtFixedRate(mAnalogBLEQuequeTask,0, mTimerAnalogBLEQueque); // msec

                    TextView textViewDataSensors = findViewById(R.id.textViewData);
                    listenerAllocatorDataSensors(textViewDataSensors);
                    visualizeDataSensors(textViewDataSensors);
                    Feature featureSelected = mNode.getFeature(FeatureSwitch.class);
                    if ((featureSelected != null) && featureSelected.isEnabled())
                    {
                        SwitchAddListener(featureSelected);
                    }
                }
            }
            else{
                if (mNode.getState() == Node.State.Connecting) {
                    Joystick.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Button textViewConnect = findViewById(R.id.Button_Connection);
                            textViewConnect.setText(" Connecting Wait ");
                            mySetColorButton(textViewConnect,argb(255,255,255,0));
                        }//run
                    });
                }else{
                    Joystick.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Button textViewConnect = findViewById(R.id.Button_Connection);
                            textViewConnect.setText(" Disconnected Press me ");
                            mySetColorButton(textViewConnect,argb(255,255,0,0));
                        }//run
                    });
                }
            }
        }
        else
        {
            Joystick.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Button textViewConnect = findViewById(R.id.Button_Connection);
                    textViewConnect.setText(" Start Connection ");
                    mySetColorButton(textViewConnect,argb(255,0,255,255));
                }//run
            });
        }
    }

    /**
     * listener that will be used for enable the notification when the node is connected
     */
    private Node.NodeStateListener mNodeStatusListener = new Node.NodeStateListener() {
        @Override
        public void onStateChange(final Node node, Node.State newState, Node.State prevState) {
            Log.d("notification", "NodeStateListener "+newState); // for debug
            StatusConnectionVisualization();
        }//onStateChange
    };

    /**
     * listener that will update the displayed feature data
     */
    private Feature.FeatureListener mGenericUpdate = null;
    private Feature.FeatureListener mSwitchUpdate = null;


    //========================================================================================
    // joyistick without BLE

    /** parameters from SettingParameter */
    private SettingParameter mInitParam;
    /** parameters from SensitivityParameter */
    private SensitivityParameter mInitSensitivityParam;

    private static boolean mFirstTime = true;
    //private boolean mFirstTime = true;

    private static boolean mBuildParameter = true;

    private FeatureJoystick mFeatureJoystick;

    private int mColor;

    private Button mSelectedButton;

    private int mTimerValue = 100; //msec

    //private int mTimerAnalogBLEQueque = 34; //msec (30 fps)
    private int mTimerAnalogBLEQueque = 20; //msec (50Hz)

    private int mWidthPixels;
    private int mHeightPixels;
    private float mDensityDpi;

    private int mWidth_dp;

    private int mHeightDirectionL;
    private int mWidthDirectionL;
    private int mStartXDirectionL;
    private int mStartYDirectionL;
    private int mStartSmallYDirectionL;
    private int mHeightDirectionR;
    private int mWidthDirectionR;
    private int mStartXDirectionR;
    private int mStartYDirectionR;

    private int mStartXL;
    private int mStartYL;
    private int mStartXR;
    private int mStartYR;

    private int mHeightSmallDirectionL;
    private int mHeightSmallDirectionR;
    private int mWidthSmallDirectionL;
    private int mWidthSmallDirectionR;

    private boolean mCorrectObjectL;
    private boolean mCorrectObjectR;
    private int mCenterX;
    private int mCenterY;
    private int mDeltaY;
    private int mRemapCenterX;
    private int mRemapCenterY;

    private int mLeftL = 0;
    private int mRightL = 0;
    private int mLeftR = 0;
    private int mRightR = 0;
    private int mTop = 0;
    private int mBottom = 0;

    private int mLenghtL = 0;
    private int mHeightData = 0;
    private int mHeightDal = 0;

    private int mHeightAvailable = 0;

    private boolean mCorrectSmallObject;
    private int mLeftSmallL = 0;
    private int mRightSmallL = 0;
    private int mBottomSmallL = 0;
    private int mTopSmallL = 0;
    private int mLeftSmallR = 0;
    private int mRightSmallR = 0;
    private int mBottomSmallR = 0;
    private int mTopSmallR = 0;

    private boolean mAnalogDirectionL = true;
    private boolean mAnalogDirectionR = true;
    private boolean mInitAnalogBoundsDone = false;

    private boolean mAnalogActivatorL = true;
    private boolean mAnalogActivatorR = true;

    private int mValuexL = 0;
    private int mValueyL = 0;
    private int mValuexR = 0;
    private int mValueyR = 0;
    private boolean mValueChangedL = false;
    private boolean mValueChangedR = false;

    private SeekBar mSeekBar;
    private int mProgressSeekBar = 100;

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

    private final static int PLANE_XY = 0;
    private final static int PLANE_XZ = 1;
    private final static int PLANE_ZY = 2;

    private int mSwapCommandcode = PLANE_XZ; // default

    private Boolean mFollowerCursor = false;
    private Boolean mYawDisalble = false;
    private Boolean mDataDroneVisible = false;
    private Boolean mLogFile = false;
    private Boolean mShapeCircle = true;


    private Boolean mButton_Take_off_status = false;
    private long mLastDown = 0;
    private Boolean mButton_Arm_status = false;
    private  boolean mButton_Setting_status = false;
    private  boolean mButton_Help_status = false;

    private int mOffsetRotY = 0; // default
    private int mOffsetAxisX = 0; // default
    private int mOffsetAxisY = 0; // default
    private int mOffsetAxisZ = 0; // default
    private int mAxisY = 0;

    private String mStringDataSensorsAcceleration = "";
    private String mStringDataSensorsGyroscope = "";
    private String mStringDataSensorsMagnetometer = "";
    private String mStringDataSensorsPressure = "";
    private String mStringDataSensorsHumidity = ""; // altitude instead

    private double mAltitude = 0;
    private static double mPressureRef = 1013.25f; // 1 atm in mBar

    private static boolean mArmedStatus = false;

    private boolean mAnalogOnGoing = false;

    private int mNumTouchOld = 0;
    private boolean mCursorLeftOld = true;

    /**
     * create an intent for start this activity
     *
     * @param c    context used for create the intent
     * @param node node to use for the demo
     * @return intent for start a demo activity that use the node as data source
     */
    //public static Intent getStartIntent(Context c, @NonNull Node node, SettingParameter initParam, SensitivityParameter initSensitivityParam) {
    public static Intent getStartIntent(Context c, @NonNull Node node) {
        Intent i = new Intent(c, Joystick.class);
        if(bleEnable) {
            if (node != null) {
                i.putExtra(NODE_TAG, node.getTag());
                i.putExtras(NodeContainerFragment.prepareArguments(node));
            }
        }
        return i;
    }//getStartIntent

    AllAnalogDrawable mAllAnalogDrawable;

//    private int fromDp_to_px(int dp) {
//        int px = (int)(dp * (mDensityDpi / 160));
//        return px;
//    }
//
//    private int fromPx_to_dp(int px) {
//        int dp = (int)(px*160/mDensityDpi);
//        return dp;
//    }

    private void drawAnalogActivatorL() {
            mAllAnalogDrawable.setBoundColorL(mStartXDirectionL,mStartYDirectionL,mHeightDirectionL,mWidthDirectionL,argb(255,0,221,255));
            mAllAnalogDrawable.setVisibleL(true);

            if(mSwapCommandcode == PLANE_XZ)
                mAllAnalogDrawable.setBoundColorSmallL(mStartXDirectionL,mStartSmallYDirectionL-mAxisY,mHeightSmallDirectionL,mWidthSmallDirectionL,argb(255,0,100,128));
            else
                mAllAnalogDrawable.setBoundColorSmallL(mStartXDirectionL,mStartYDirectionR,mHeightSmallDirectionL,mWidthSmallDirectionL,argb(255,0,100,128));
            mAllAnalogDrawable.setVisibleSmallL(true);

            mAllAnalogDrawable.setVisibility(View.VISIBLE);
            mAllAnalogDrawable.invalidate();

        mStartXL = mStartXDirectionL;
        mStartYL = mStartYDirectionL;
    }
    private void drawAnalogActivatorR() {
            mAllAnalogDrawable.setBoundColorR(mStartXDirectionR,mStartYDirectionR,mHeightDirectionR,mWidthDirectionR,argb(255,0,221,255));
            mAllAnalogDrawable.setVisibleR(true);

            if(mSwapCommandcode == PLANE_XZ)
                mAllAnalogDrawable.setBoundColorSmallR(mStartXDirectionR,mStartYDirectionR,mHeightSmallDirectionR,mWidthSmallDirectionR,argb(255,0,100,128));
            else
                mAllAnalogDrawable.setBoundColorSmallR(mStartXDirectionR,mStartSmallYDirectionL-mAxisY,mHeightSmallDirectionR,mWidthSmallDirectionR,argb(255,0,100,128));
            mAllAnalogDrawable.setVisibleSmallR(true);

            mAllAnalogDrawable.setVisibility(View.VISIBLE);
            mAllAnalogDrawable.invalidate();

        mStartXR = mStartXDirectionR;
        mStartYR = mStartYDirectionR;
    }

    private void drawAnalog() {

        if(mInitAnalogBoundsDone)
        {
            if((mAnalogDirectionL)&&(mAnalogActivatorL))
            {
                Log.d("drawAnalog", "drawAnalogActivatorL"); // for debug
                drawAnalogActivatorL();
            }
            if((mAnalogDirectionR)&&(mAnalogActivatorR))
            {
                Log.d("drawAnalog", "drawAnalogActivatorR"); // for debug
                drawAnalogActivatorR();
            }
        }
        else
        {
            mAllAnalogDrawable.setVisibility(View.INVISIBLE);
            mAllAnalogDrawable.invalidate();
        }
    }

    private void initAnalogBounds() {

        // the y = 0 starts after menuInflate + initial margin of activity_joystick
        // the menuInflate ends where LinearLayout_MainButtons starts

        int heightMenu = mHeightPixels - mHeightAvailable;
        mBottom -= heightMenu;
        mBottom += CoordConverter.fromDp_to_px(8); // coming from initial margin of activity_joystick
        mTop += CoordConverter.fromDp_to_px(8); // coming from initial margin of activity_joystick
        int marginCursorY = CoordConverter.fromDp_to_px(10); // to have free space between buttons
        int availableSpace = (mBottom-mTop)-marginCursorY;
        if(availableSpace > (mRightL-mLeftL))
            availableSpace = (mRightL-mLeftL);
        mWidthDirectionL = availableSpace/2; // horizontal half lenght of big analog left cursor
        int lenghtdp = CoordConverter.fromPx_to_dp(mWidthDirectionL);
        if (lenghtdp > 200) {
            mWidthDirectionL = CoordConverter.fromDp_to_px(200); // horizontal half lenght of big analog left cursor 200dp as maximum
        }
        mHeightDirectionL = mWidthDirectionL; // vertical half lenght of big analog left cursor
        mStartXDirectionL = mLeftL+mWidthDirectionL; // center x of big analog left cursor
        mStartYDirectionL = (mTop+mBottom)/2;// center y of big analog left cursor
        mHeightDirectionR = mHeightDirectionL;// vertical half lenght of big analog right cursor
        mWidthDirectionR = mWidthDirectionL;// horizontal half lenght of big analog right cursor
        mStartXDirectionR = mWidthPixels-mStartXDirectionL;
        mStartYDirectionR = mStartYDirectionL;
        mHeightSmallDirectionL = mHeightDirectionL/4;
        mWidthSmallDirectionL = mWidthDirectionL/4;
        if(mYawDisalble)
        {
            mWidthDirectionL = mWidthSmallDirectionL;
            if(mAllAnalogDrawable != null)
                mAllAnalogDrawable.setYaw(mYawDisalble);
        }
        mDeltaY = (mHeightDirectionL-mHeightSmallDirectionL);
        mStartSmallYDirectionL = mStartYDirectionL+mDeltaY;
        mHeightSmallDirectionR = mHeightSmallDirectionL;
        mWidthSmallDirectionR = mWidthSmallDirectionL;

        mLeftR = mWidthPixels-mRightL;
        mRightR = mWidthPixels;

        // todo: fix uncorrect buttonw or mStartXDirectionL values
        Button selectedButton2 = findViewById(R.id.Button_L);
        int buttonw = CoordConverter.fromPx_to_dp(selectedButton2.getWidth());
        LinearLayout layout = findViewById(R.id.tableLayout_DirectionL);
        LinearLayout.LayoutParams params = (LinearLayout.LayoutParams)layout.getLayoutParams();
        int centerLdp = CoordConverter.fromPx_to_dp(mStartXDirectionL);
        //int centerLdp = (mStartXDirectionL);
        //int margin = centerLdp-(params.width/2);
        int margin = centerLdp-(buttonw/2+buttonw);
        params.setMargins(margin, params.topMargin, params.rightMargin, params.bottomMargin);
        layout.setLayoutParams(params);

        layout = findViewById(R.id.tableLayout_DirectionR);
        params = (LinearLayout.LayoutParams)layout.getLayoutParams();
        params.setMargins(params.leftMargin, params.topMargin, margin, params.bottomMargin);
        layout.setLayoutParams(params);


        if(!mInitAnalogBoundsDone)
        {
            boolean enableFactor = false;
            if ((getResources().getConfiguration().screenLayout & Configuration.SCREENLAYOUT_SIZE_MASK) >= Configuration.SCREENLAYOUT_SIZE_LARGE) {
                // on a large screen device ...
                enableFactor = true;
            }

            float factor = availableSpace /(float)(mLenghtL);
            if((factor > 1)&&enableFactor) //if large screen and space available > of big lenght analog cursor
            {
                if(!graphicsEnable)
                {
                    // Digital Buttons text rescaling
                    Button selectedButton = findViewById(R.id.Button_UP);
                    float sizeText = selectedButton.getTextSize();
                    sizeText = sizeText*factor;
                    selectedButton.setTextSize(sizeText);
                    selectedButton = findViewById(R.id.Button_UP2);
                    selectedButton.setTextSize(sizeText);
                    selectedButton = findViewById(R.id.Button_Down);
                    selectedButton.setTextSize(sizeText);
                    selectedButton = findViewById(R.id.Button_Down2);
                    selectedButton.setTextSize(sizeText);
                    selectedButton = findViewById(R.id.Button_L);
                    selectedButton.setTextSize(sizeText);
                    selectedButton = findViewById(R.id.Button_Left2);
                    selectedButton.setTextSize(sizeText);
                    selectedButton = findViewById(R.id.Button_R);
                    selectedButton.setTextSize(sizeText);
                    selectedButton = findViewById(R.id.Button_Right2);
                    selectedButton.setTextSize(sizeText);
                }
            }
            else
            {
                // small Button_Connection, small Buttons for analog/digital and seekBar with small height
                Button selectedButton = findViewById(R.id.Button_Connection);
                int  pixelHeight = CoordConverter.fromDp_to_px(35);
                ViewGroup.LayoutParams LayoutParamButton = selectedButton.getLayoutParams();
                if(LayoutParamButton.height > pixelHeight)
                    LayoutParamButton.height = pixelHeight;
                selectedButton = findViewById(R.id.Button_DAL);
                LayoutParamButton = selectedButton.getLayoutParams();
                if(LayoutParamButton.height > pixelHeight)
                    LayoutParamButton.height = pixelHeight;
                selectedButton = findViewById(R.id.Button_DAR);
                LayoutParamButton = selectedButton.getLayoutParams();
                if(LayoutParamButton.height > pixelHeight)
                    LayoutParamButton.height = pixelHeight;
                pixelHeight = CoordConverter.fromDp_to_px(25);
                ViewGroup.LayoutParams LayoutParamSeek = mSeekBar.getLayoutParams();
                if(LayoutParamSeek.height > pixelHeight)
                    LayoutParamSeek.height = pixelHeight;
            }
        }

        mInitAnalogBoundsDone = true;

        if(cursorDrawEnable)
        {
            drawAnalog();
        }
    }

    void mySetColorButton(Button selectedButton, int color)
    {
        if (mWidth_dp >= 820) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
                selectedButton.getBackground().setTint(color);
            } else
                selectedButton.setBackgroundColor(color);
        }
        else
            selectedButton.setBackgroundColor(color);
    }

    void getParameters(SharedPreferences pref){
        mSwapCommandcode = pref.getInt("mSwapCommandcode", mSwapCommandcode_default);
        mDataDroneVisible = pref.getBoolean("mDataDroneVisible", mDataDroneVisible_default);
        mFollowerCursor = pref.getBoolean("mFollowerCursor", mFollowerCursor_default);
        mYawDisalble = pref.getBoolean("mYawDisalble", mYawDisalble_default);
        mLogFile = pref.getBoolean("mLogFile", mLogFile_default);
        mShapeCircle = pref.getBoolean("mShapeCircle", mShapeCircle_default);

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

        setContentView(R.layout.activity_joystick);

        if(!majorButtonsDrawDisable)
        {
            findViewById(R.id.Button_UP).setOnClickListener(mOnClickListener);
            findViewById(R.id.Button_L).setOnClickListener(mOnClickListener);
            findViewById(R.id.Button_R).setOnClickListener(mOnClickListener);
            findViewById(R.id.Button_Down).setOnClickListener(mOnClickListener);
            findViewById(R.id.Button_UP2).setOnClickListener(mOnClickListener);
            findViewById(R.id.Button_Left2).setOnClickListener(mOnClickListener);
            findViewById(R.id.Button_Right2).setOnClickListener(mOnClickListener);
            findViewById(R.id.Button_Down2).setOnClickListener(mOnClickListener);

            findViewById(R.id.Button_Connection).setOnClickListener(mOnClickListener);

            findViewById(R.id.Button_Take_off).setOnClickListener(mOnClickListener);
            findViewById(R.id.Button_Sensor_Calibration).setOnTouchListener(mOnTouchListener_Button_Sensor_Calibration);
            findViewById(R.id.Button_Arm).setOnClickListener(mOnClickListener);
            findViewById(R.id.Button_Help).setOnClickListener(mOnClickListener);
        }
        findViewById(R.id.Button_Setting).setOnClickListener(mOnClickListener);

        // how to generate an imageButton with 3 states instead of radioButton (switch and toggleButton are only 2 states)
        // see: https://stackoverflow.com/questions/23558156/how-can-i-add-third-state-in-android-switch

        mProgressSeekBar = 100;

        if(mBuildParameter) {
            DefaultParameters.BuildDefaultParameters();
         }
        mBuildParameter = false;

        mSeekBar = findViewById(R.id.seekBar);
        if(!majorButtonsDrawDisable)
        {
            mSeekBar.setProgress(mProgressSeekBar);
            mSeekBar.setOnSeekBarChangeListener(mSeekBarListener);

            findViewById(R.id.Button_DAL).setOnClickListener(mOnClickListener);
            findViewById(R.id.Button_DAR).setOnClickListener(mOnClickListener);

            mColor = argb(255,0,0x99,0xcc);
            Button selectedButton = findViewById(R.id.Button_UP);
            mySetColorButton(selectedButton,mColor);
            selectedButton = findViewById(R.id.Button_Down);
            mySetColorButton(selectedButton,mColor);
            selectedButton = findViewById(R.id.Button_L);
            mySetColorButton(selectedButton,mColor);
            selectedButton = findViewById(R.id.Button_R);
            mySetColorButton(selectedButton,mColor);
            mColor = argb(255,0,0x99,0xcc);
            selectedButton = findViewById(R.id.Button_UP2);
            mySetColorButton(selectedButton,mColor);
            selectedButton = findViewById(R.id.Button_Down2);
            mySetColorButton(selectedButton,mColor);
            mColor = argb(255,0,0x99,0xcc);
            selectedButton = findViewById(R.id.Button_Left2);
            mySetColorButton(selectedButton,mColor);
            selectedButton = findViewById(R.id.Button_Right2);
            mySetColorButton(selectedButton,mColor);
        }

        if(majorButtonsDrawDisable)
        {
            Button selectedButton = findViewById(R.id.Button_Connection);
            selectedButton.setVisibility(View.INVISIBLE);
            selectedButton = findViewById(R.id.Button_Take_off);
            selectedButton.setVisibility(View.INVISIBLE);
            selectedButton = findViewById(R.id.Button_Sensor_Calibration);
            selectedButton.setVisibility(View.INVISIBLE);
            selectedButton = findViewById(R.id.Button_Arm);
            selectedButton.setVisibility(View.INVISIBLE);
            selectedButton = findViewById(R.id.Button_Help);
            selectedButton.setVisibility(View.INVISIBLE);
            selectedButton = findViewById(R.id.Button_DAL);
            selectedButton.setVisibility(View.INVISIBLE);
            selectedButton = findViewById(R.id.Button_DAR);
            selectedButton.setVisibility(View.INVISIBLE);
            mSeekBar.setVisibility(View.INVISIBLE);
        }

        DisplayMetrics metrics = getResources().getDisplayMetrics();
        mWidthPixels  = metrics.widthPixels;
        mHeightPixels = metrics.heightPixels;
        mDensityDpi = metrics.densityDpi;
        mWidth_dp  = CoordConverter.fromPx_to_dp(mWidthPixels);

        CoordConverter.init(mWidthPixels,mHeightPixels,mDensityDpi);


        if(cursorDrawEnable)
        {
            ViewGroup.LayoutParams param = new ViewGroup.LayoutParams(mWidthPixels,mHeightPixels);
            mAllAnalogDrawable = new AllAnalogDrawable(this);
            addContentView (mAllAnalogDrawable, param);
        }


        LinearLayout selectedLinearLayout = findViewById(R.id.tableLayout_DirectionL);
        selectedLinearLayout.setVisibility(View.INVISIBLE);
        selectedLinearLayout = findViewById(R.id.tableLayout_DirectionR);
        selectedLinearLayout.setVisibility(View.INVISIBLE);
        mInitAnalogBoundsDone = false;


        if(!lastFeatureEnable)
        {
            Button selectedButton = findViewById(R.id.Button_Take_off);
            selectedButton.setVisibility(View.INVISIBLE);
        }

        if(cursorDrawEnable)
            mAllAnalogDrawable.setOnTouchListener(mOnTouchListener);

        mOffsetRotY = 0; // default
        mOffsetAxisX = 0; // default
        mOffsetAxisY = 0; // default
        mOffsetAxisZ = 0; // default

        Log.d("mFirstTime", this+" "+mFirstTime); // for debug
        if(!mFirstTime)
        {

            //=======================================================================================
            // BLE
            //find the node
            String nodeTag = getIntent().getStringExtra(NODE_TAG);
            mNode = Manager.getSharedInstance().getNodeWithTag(nodeTag);

            if(mNode == null)
                mFirstTime = true;
            else
            {
                //mFirstTime = false;
                //registration feature
                UUIDToFeatureMap map = new UUIDToFeatureMap();
                map.put(UUID.fromString("00008000-0001-11e1-ac36-0002a5d5c51b") , FeatureJoystick.class); // new FeatureJoystick
                mNode.addExternalCharacteristics(map);

                //create or recover the NodeContainerFragment
                if (savedInstanceState == null) {
                    Intent i = getIntent();
//                    if (mNodeContainer != null)
//                        mNodeContainer.onDestroy();
                    mNodeContainer = new NodeContainerFragment();
                    mNodeContainer.setArguments(i.getExtras());

                    getFragmentManager().beginTransaction()
                            .add(mNodeContainer, NODE_FRAGMENT).commit();

                } else {
                    mNodeContainer = (NodeContainerFragment) getFragmentManager()
                            .findFragmentByTag(NODE_FRAGMENT);

                }//if-else
            }
        }
    }


    @Override
    protected void onStart(){
        super.onStart();

        StatusArmedVisualization(mArmedStatus);

        SharedPreferences pref = getApplicationContext().getSharedPreferences("MyPref", MODE_PRIVATE);
        getParameters(pref);
        if(cursorDrawEnable)
        {
            mAllAnalogDrawable.setShape(mShapeCircle);
            mAllAnalogDrawable.setDensity(mDensityDpi);
            mAllAnalogDrawable.setPlanes(mSwapCommandcode);
            mAllAnalogDrawable.setYaw(mYawDisalble);
        }

        if(!majorButtonsDrawDisable) {
            TextView textSeekBar = findViewById(R.id.textSeekBar2);
            textSeekBar.setText("Speed Scaler " + String.valueOf(mProgressSeekBar) + "%");
            if(!mDataDroneVisible)
            {
                TextView textViewDataSensors = findViewById(R.id.textViewData);
                String swVersion = "STDrone software version ";
                swVersion = swVersion + String.valueOf(mVersion);
                textViewDataSensors.setText(swVersion);
            }
        }

        mRightL = 0;
        mTop = 0;
        mBottom = 0;
        mHeightAvailable = 0;

        if(!majorButtonsDrawDisable)
        {
            LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_MainButtons);
            selectedLinearLayout.post(new Runnable() {
                @Override
                public void run() {
                    LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_MainButtons);
                    mTop = selectedLinearLayout.getBottom();
                    selectedLinearLayout = findViewById(R.id.tableLayout_DirectionL);
                    mLeftL = CoordConverter.fromDp_to_px(50); // margin to put the finger out cursor
                    mLenghtL = selectedLinearLayout.getRight() - mLeftL;
                    if((mTop>0)&&(mRightL>0)&&(mBottom>0)&&(mHeightAvailable>0))
                        initAnalogBounds();
                }
            });

            selectedLinearLayout = findViewById(R.id.LinearLayout_for_measures);
            selectedLinearLayout.post(new Runnable() {
                @Override
                public void run() {
                    LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_for_measures);
                    mHeightAvailable = selectedLinearLayout.getHeight();
                    if((mTop>0)&&(mRightL>0)&&(mBottom>0)&&(mHeightAvailable>0))
                        initAnalogBounds();
                }
            });

            selectedLinearLayout = findViewById(R.id.LinearLayout_textViewData);
            selectedLinearLayout.post(new Runnable() {
                @Override
                public void run() {
                    LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_textViewData);
                    mHeightData = selectedLinearLayout.getHeight();
                    if(mHeightDal > 0)
                        mBottom = mHeightPixels - mHeightData - mHeightDal;
                    if((mTop>0)&&(mRightL>0)&&(mBottom>0)&&(mHeightAvailable>0))
                        initAnalogBounds();
                }
            });

            selectedLinearLayout = findViewById(R.id.LinearLayout_Button_DAL);
            selectedLinearLayout.post(new Runnable() {
                @Override
                public void run() {
                    LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_Button_DAL);
                    mHeightDal = selectedLinearLayout.getHeight();
                    if(mHeightData > 0)
                        mBottom = mHeightPixels - mHeightData - mHeightDal;
                    if((mTop>0)&&(mRightL>0)&&(mBottom>0)&&(mHeightAvailable>0))
                        initAnalogBounds();
                }
            });

            Button selectedButton = findViewById(R.id.Button_DAL);
            selectedButton.post(new Runnable() {
                @Override
                public void run() {
                    Button selectedButton = findViewById(R.id.Button_DAL);
                    Rect rect = selectedButton.getClipBounds();

                    if((mTop>0)&&(mRightL>0)&&(mBottom>0)&&(mHeightAvailable>0))
                        initAnalogBounds();
                }
            });

            selectedLinearLayout = findViewById(R.id.LinearLayout_SeekBar);
            selectedLinearLayout.post(new Runnable() {
                @Override
                public void run() {
                    LinearLayout selectedLinearLayout = findViewById(R.id.LinearLayout_SeekBar);
                    //mRightL = selectedLinearLayout.getLeft(); // outLocation is better because it includes also initial margin of activity_joystick
                    int[] outLocation = new int[2];
                    selectedLinearLayout.getLocationOnScreen (outLocation); // y doesn't work , because it starts from previous LinearLayout
                    mRightL = outLocation[0]-CoordConverter.fromDp_to_px(5);
                    if((mTop>0)&&(mRightL>0)&&(mBottom>0)&&(mHeightAvailable>0))
                        initAnalogBounds();
                }
            });

        }

//        if(mFirstTime == false)  // linked to LogFeatureActivity
//            stopLogging();
    }

    /**
     * stop all the enabled notification
     */
    private void disableNeedNotification() {
        List<Feature> features = mNode.getFeatures();
        for (Feature f : features) {
            if (mNode.isEnableNotification(f))
                mNode.disableNotification(f);
            if(mGenericUpdate != null)
                f.removeFeatureListener(mGenericUpdate);
        }//for sTestFeature

        Feature featureSelected = mNode.getFeature(FeatureSwitch.class);
        if (featureSelected != null){
            if(mSwitchUpdate != null)
                featureSelected.removeFeatureListener(mSwitchUpdate);
        }
    }//disableNeedNotification

    private void StatusBatteryVoltagePercentageVisualization(final int batteryVoltagePercentage){
        Joystick.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                TextView textViewConnect = findViewById(R.id.textViewBattery);
                textViewConnect.setText(" Battery "+String.valueOf(batteryVoltagePercentage)+"% ");
                if(batteryVoltagePercentage >= 85)
                    textViewConnect.setBackgroundColor(argb(255,0x99,0xcc,0));
                else
                    textViewConnect.setBackgroundColor(argb(255,255,0xbb,0x33));
            }//run
        });
    }

    private void StatusRSSIdroneVisualization(final int RSSIdrone){
        Joystick.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                TextView textViewConnect = findViewById(R.id.textViewRSSI);
                textViewConnect.setText(" RSSI "+String.valueOf(RSSIdrone)+"db ");
            }//run
        });
    }

    private void StatusArmedVisualization(boolean armedStatus){
        if (armedStatus) {
            Joystick.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    TextView textViewConnect = findViewById(R.id.textViewARMED);
                    textViewConnect.setText("   Armed   ");
                    textViewConnect.setBackgroundColor(argb(255,0,255,0));
                }//run
            });
        }
        else{
            Joystick.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    TextView textViewConnect = findViewById(R.id.textViewARMED);
                    textViewConnect.setText(" Not Armed ");
                    textViewConnect.setBackgroundColor(argb(255,255,0,0));
                }//run
            });
        }
    }

    /**
     * if the node is connected enable the gui, otherwise set a listener that will do that
     */
    @Override
    protected void onResume() {
        super.onResume();
        mButton_Setting_status = false;
        mButton_Help_status = false;
        mAnalogOnGoing = false;
        mAnalogActivatorL = true;
        mAnalogActivatorR = true;
        if(mNode == null)
            mFirstTime = true;
//        else
//            mFirstTime = false;
        Log.d("notification", this+" onResume"); // for debug

        if(mScrollTimer == null)
            mScrollTimer = new Timer();
        StatusConnectionVisualization();
        if(!mFirstTime)
            mNode.addNodeStateListener(mNodeStatusListener);

        // Drone_Sensors:
        {
            if(mDataDroneVisible)
            {
                TextView textViewDataSensors = findViewById(R.id.textViewData);

                visualizeDataSensors(textViewDataSensors);
            }
            else
            {
                // the button for mDataDroneVisible was in joystick, it is in setting now
                // todo: check if it is yet useful
                if(mGenericUpdate !=null)
                {
                    Feature featureSelected = mNode.getFeature(MyFeatureAcceleration.class);
                    if ((featureSelected != null) && featureSelected.isEnabled()) {
                        featureSelected.removeFeatureListener(mGenericUpdate);
                        mNode.disableNotification(featureSelected);
                    }
                    featureSelected = mNode.getFeature(MyFeatureGyroscope.class);
                    if ((featureSelected != null) && featureSelected.isEnabled()) {
                        featureSelected.removeFeatureListener(mGenericUpdate);
                        mNode.disableNotification(featureSelected);
                    }
                    featureSelected = mNode.getFeature(MyFeatureMagnetometer.class);
                    if ((featureSelected != null) && featureSelected.isEnabled()) {
                        featureSelected.removeFeatureListener(mGenericUpdate);
                        mNode.disableNotification(featureSelected);
                    }
                    featureSelected = mNode.getFeature(MyFeaturePressure.class);
                    if ((featureSelected != null) && featureSelected.isEnabled()) {
                        featureSelected.removeFeatureListener(mGenericUpdate);
                        mNode.disableNotification(featureSelected);
                    }
                    activeBatteryRSSIlistener(); // pressure must be before else disable also temperature and humidity
                }
            }
        }

    }//onResume


    @Override
    protected void onPause() {

        mScrollTimer.cancel();
        mScrollTimer.purge();
        mScrollTimer = null;

        Log.d("notification", this+" onPause "); // for debug
        //if((mNode != null)&&(!mButton_Setting_status)&&(!mButton_Help_status))
        if(mNode != null)
        {
            Log.d("notification", this+" onPauseIfNode "+mNode.getState()); // for debug
            //it is safe remove also if we didn't add it
            mNode.removeNodeStateListener(mNodeStatusListener);

            //if the node is already disconnected we don't care of disable the notification
            if (mNode.isConnected()) {
                disableNeedNotification();
            }//if
        }

        super.onPause();
    }//stopDemo

    @Override
    protected void onStop() {
        super.onStop();
        Log.d("notification", this+" onStop "); // for debug
        if((mNodeContainer!=null)&&(!mButton_Setting_status)&&(!mButton_Help_status)) {
            Log.d("notification", "mButton_Setting_status "+mButton_Setting_status); // for debug
            Log.d("notification", "mButton_Help_status "+mButton_Help_status); // for debug
            Log.d("notification", this+" onStopIfNode "+mNode.getState()); // for debug
            mNodeContainer.onDisconnection();
        }
    }

    @Override
    protected void onDestroy() {
        Log.d("notification", this+" onDestroy "); // for debug
        super.onDestroy();
    }

    @Override
    public void onBackPressed() {
        if (mNode != null)
            mNodeContainer.keepConnectionOpen(true);
        super.onBackPressed();
    }//onBackPressed

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {

        switch (item.getItemId()) {
            // Respond to the action bar's Up/Home button, we don't return for permit to
            // do the standard action of this button
            case android.R.id.home:
                if (mNode != null)
                    mNodeContainer.keepConnectionOpen(true);
        }//switch

        return super.onOptionsItemSelected(item);
    }//onOptionsItemSelected


    private Timer mScrollTimer = new Timer();

    private TimerTask mScrollTask = new myScrollTask();

    private class myScrollTask extends TimerTask {
        @Override
        public void run() {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    mySetColorButton(mSelectedButton,mColor);
                }
            });
        }
    };


    private TimerTask mAnalogBLEQuequeTask = new myAnalogBLEQuequeTask();

    private class myAnalogBLEQuequeTask extends TimerTask {
        @Override
        public void run() {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    mFeatureJoystick.enableWriteBLEQueque();
                }
            });
        }
    };

    private TimerTask mButton_Sensor_Calibration_Task = new myButton_Sensor_Calibration_Task();

    private class myButton_Sensor_Calibration_Task extends TimerTask {
        @Override
        public void run() {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                    {
                        mFeatureJoystick.setStatusJoystick(FeatureJoystick.Calib_Mask,FeatureJoystick.Calib_OFF);
                    }
                    Button selectedButton = findViewById(R.id.Button_Sensor_Calibration);
                    mySetColorButton(selectedButton,argb(255,0,0x99,0xcc)); //"@android:color/holo_blue_dark"
                    Log.d("Sensor_Calibration","Sensor_Calibration good cmd");
                }
            });
        }
    };

    private TimerTask mButton_Sensor_Calibration_Fail_Task = new myButton_Sensor_Calibration_Fail_Task();

    private class myButton_Sensor_Calibration_Fail_Task extends TimerTask {
        @Override
        public void run() {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Button selectedButton = findViewById(R.id.Button_Sensor_Calibration);
                    mySetColorButton(selectedButton,argb(255,0,0x99,0xcc)); //"@android:color/holo_blue_dark"
                    Log.d("Sensor_Calibration","Sensor_Calibration fail cmd");
                }
            });
        }
    };


    /**
     * class used for update the feature display data
     */
    private class SwitchFragmentUpdate implements Feature.FeatureListener {

        /**
         * set the text view text with the feature toString value
         *
         * @param f      feature that has received an update
         * @param sample new data received from the feature
         */
        @Override
        public void onUpdate(Feature f, Feature.Sample sample) {
            final Feature.Sample samplef = sample;
            Joystick.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    mArmedStatus= (samplef.data[0].byteValue()!=0); // true = ON
                    StatusArmedVisualization(mArmedStatus);
                }
            });
        }//onUpdate

    }//SwitchFragmentUpdate

    private void SwitchAddListener (Feature selectedFeature) {

        if(selectedFeature==null)
            return;
        //create a listener that will update the selected view
        mSwitchUpdate = new SwitchFragmentUpdate();
        selectedFeature.addFeatureListener(mSwitchUpdate);

        mNode.enableNotification(selectedFeature);
    }

    /**
     * class used for update the feature display data
     */
    private class GenericFragmentUpdate implements Feature.FeatureListener {

        /**
         * text view that will contain the data/name
         */
        final private TextView mTextView;

         GenericFragmentUpdate(TextView textView) {
             mTextView = textView;
        }//GenericFragmentUpdate

        /**
         * set the text view text with the feature toString value
         *
         * @param f      feature that has received an update
         * @param sample new data received from the feature
         */
        @Override
        public void onUpdate(Feature f, Feature.Sample sample) {
            final Feature.Sample samplef = sample;
            if (f instanceof FeatureHumidity) { // battery % instead
                int batteryVoltagePercentage = samplef.data[0].intValue();
                //Log.d("notification", "drawBattery"); // for debug
                StatusBatteryVoltagePercentageVisualization(batteryVoltagePercentage);
            }
            if (f instanceof FeatureTemperature) { // RSSI % instead
                int RSSIdrone = samplef.data[0].intValue();
                Log.d("notification", "drawRSSI: "+String.valueOf(RSSIdrone)); // for debug
                StatusRSSIdroneVisualization(RSSIdrone);
            }
            if(mDataDroneVisible) {
                if (f instanceof FeatureAcceleration)
                    mStringDataSensorsAcceleration = f.toString();
                if (f instanceof FeatureGyroscope)
                    mStringDataSensorsGyroscope = f.toString();
                if (f instanceof FeatureMagnetometer)
                    mStringDataSensorsMagnetometer = f.toString();
                if (f instanceof FeaturePressure){
                    mStringDataSensorsPressure = f.toString();
                    MyFeatureHumidity myf = new MyFeatureHumidity(f.getParentNode());
                    double pressure = samplef.data[0].floatValue();
                    if(mButton_Arm_status)
                    {
                        mAltitude = 44330*100*(1- Math.pow(pressure/mPressureRef,(double)(1.0f/5.255f))); // *100 to have cm instead of m
//                        Log.d("notification", "PressureRef : "+String.valueOf(mPressureRef)); // for debug
//                        Log.d("notification", "Pressure : "+String.valueOf(pressure)); // for debug
//                        Log.d("notification", "mAltitude : "+String.valueOf(mAltitude)); // for debug
                    }
                    else
                    {
                        mPressureRef = pressure;
                        mAltitude = 0;
                       // Log.d("notification", "PressureRef : "+String.valueOf(mPressureRef)); // for debug
                    }
                    myf.ReplaceValue((float)mAltitude);
                    if(lastFeatureEnable)
                    {
                        mStringDataSensorsHumidity = myf.toString(); // altitude instead
                    }
                    else
                    {
                        mStringDataSensorsHumidity = ""; // altitude instead
                    }
                }
                final String featureDump = mStringDataSensorsAcceleration + mStringDataSensorsGyroscope + mStringDataSensorsMagnetometer + mStringDataSensorsPressure + mStringDataSensorsHumidity;
                if (f instanceof FeatureAcceleration) {
                    Joystick.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mTextView.setText(featureDump);
                            //if(mAnalogOnGoing == false)
                            {
                                //Log.d("notification", "mDataDroneVisible"); // for debug
                            }
                        }
                    });
                }
            }
        }//onUpdate
    }//GenericFragmentUpdate

    private void CaracteristicVisualization(Feature selectedFeature) {

             selectedFeature.addFeatureListener(mGenericUpdate);

            //if (!mNode.isEnableNotification(selectedFeature))
                mNode.enableNotification(selectedFeature);
    }

    private SeekBar.OnSeekBarChangeListener mSeekBarListener = new SeekBar.OnSeekBarChangeListener() {
        @Override
        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
//            progress	int: The current progress level. This will be in the range min..max where min
//            and max were set by setMin(int) and setMax(int), respectively. (The default values for min is 0 and max is 100.)
            {
                Log.d("scrollText", "SeekBar : progress value: "+String.valueOf(progress)); // for debug
                mProgressSeekBar =  progress;

                if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                {
                    int valuey = 0;
                    if (mSwapCommandcode != PLANE_XZ)
                    {
                        valuey = (int) (((float) mValueyR * mProgressSeekBar) * 0.01 + 0.5);
                        mFeatureJoystick.writeAnalogY(FeatureJoystick.YAXIS_ANALOG_INDEX, valuey, mNode,false);
                    }
                    else
                    {
                        valuey = (int) (((float) mValueyL * mProgressSeekBar) * 0.01 + 0.5);
                        mFeatureJoystick.writeAnalogY(FeatureJoystick.YAXIS_ANALOG_INDEX, valuey, mNode,true);
                    }
                }


                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        TextView textSeekBar = findViewById(R.id.textSeekBar2);
                        if(mAnalogDirectionL && mAnalogDirectionR)
                        {
                            textSeekBar.setText("Speed Scaler "+ String.valueOf(mProgressSeekBar)+"%");
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
        }
    };

    void listenerAllocatorDataSensors(TextView textViewDataSensors)
    {
        boolean noFeatures = true;
        Feature featureSelected = mNode.getFeature(MyFeatureAcceleration.class);
        if ((featureSelected != null) && featureSelected.isEnabled())
        {
            noFeatures = false;
        }
        featureSelected = mNode.getFeature(MyFeatureGyroscope.class);
        if ((featureSelected != null) && featureSelected.isEnabled())
        {
            noFeatures = false;
        }
        featureSelected = mNode.getFeature(MyFeatureMagnetometer.class);
        if ((featureSelected != null) && featureSelected.isEnabled())
        {
            noFeatures = false;
        }
        featureSelected = mNode.getFeature(MyFeaturePressure.class);
        if ((featureSelected != null) && featureSelected.isEnabled())
        {
            noFeatures = false;
        }
        featureSelected = mNode.getFeature(MyFeatureHumidity.class); // receives battery % instead
        if ((featureSelected != null) && featureSelected.isEnabled())
        {
            noFeatures = false;
        }
        //featureSelected = mNode.getFeatures(MyFeatureTemperature.class).get(0); // RSSI  instead
        featureSelected = mNode.getFeature(MyFeatureTemperature.class); // RSSI  instead
        if ((featureSelected != null) && featureSelected.isEnabled())
        {
            noFeatures = false;
        }

        if (!noFeatures)
        {
            mGenericUpdate = new GenericFragmentUpdate(textViewDataSensors);
        }
    }

    void visualizeDataSensors(TextView textViewDataSensors)
    {
        boolean noFeatures = true;
        if(!mFirstTime)
        {
            Feature featureSelected = mNode.getFeature(MyFeatureAcceleration.class);
            if ((featureSelected != null) && featureSelected.isEnabled())
            {
                CaracteristicVisualization(featureSelected);
                noFeatures = false;
            }
            featureSelected = mNode.getFeature(MyFeatureGyroscope.class);
            if ((featureSelected != null) && featureSelected.isEnabled())
            {
                CaracteristicVisualization(featureSelected);
                noFeatures = false;
            }
            featureSelected = mNode.getFeature(MyFeatureMagnetometer.class);
            if ((featureSelected != null) && featureSelected.isEnabled())
            {
                CaracteristicVisualization(featureSelected);
                noFeatures = false;
            }
            featureSelected = mNode.getFeature(MyFeaturePressure.class);
            if ((featureSelected != null) && featureSelected.isEnabled())
            {
                CaracteristicVisualization(featureSelected);
                noFeatures = false;
            }
            featureSelected = mNode.getFeature(MyFeatureHumidity.class); // battery %  instead
            if ((featureSelected != null) && featureSelected.isEnabled())
            {
                CaracteristicVisualization(featureSelected);
                noFeatures = false;
            }
            //featureSelected = mNode.getFeatures(MyFeatureTemperature.class).get(0); // RSSI  instead
            featureSelected = mNode.getFeature(MyFeatureTemperature.class); // RSSI  instead
            if ((featureSelected != null) && featureSelected.isEnabled())
            {
                CaracteristicVisualization(featureSelected);
                noFeatures = false;
            }
        }

        if (noFeatures)
        {
            String swVersion = "STDrone software version ";
            swVersion = swVersion + String.valueOf(mVersion);
            swVersion += "    Data Drone not available !!!";
            textViewDataSensors.setText(swVersion);
        }
    }

    void activeBatteryRSSIlistener()
    {
        if(!mFirstTime)
        {
            Feature featureSelected = mNode.getFeature(MyFeatureHumidity.class); // battery %  instead
            if ((featureSelected != null) && featureSelected.isEnabled())
            {
                CaracteristicVisualization(featureSelected);
            }
            //featureSelected = mNode.getFeatures(MyFeatureTemperature.class).get(0); // RSSI  instead
            featureSelected = mNode.getFeature(MyFeatureTemperature.class); // RSSI  instead
            if ((featureSelected != null) && featureSelected.isEnabled())
            {
                CaracteristicVisualization(featureSelected);
            }
        }
    }

    private View.OnClickListener mOnClickListener = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            switch (v.getId()) {

                case R.id.Button_DAL:
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Button selectedButton = findViewById(R.id.Button_DAL);
                            mColor = argb(255,255,255,0);
                            mySetColorButton(selectedButton,mColor);
                            mSelectedButton = selectedButton;
                            mScrollTask.cancel();
                            mScrollTask = new myScrollTask();
                            if(graphicsEnable)
                            {
                                mColor = argb(0,255,255,255);
                            }
                            else
                            {
                                mColor = argb(255,0,0x99,0xcc);
                            }
                            mScrollTimer.schedule(mScrollTask, mTimerValue); // msec

                            //digital/analog
                            LinearLayout selectedLinearLayout = findViewById(R.id.tableLayout_DirectionL);
                            if(!mAnalogDirectionL)
                            {
                                selectedButton.setText(" Offset Left ");
                                selectedLinearLayout.setVisibility(View.INVISIBLE);

                                drawAnalogActivatorL();

                                if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                                {
                                    mFeatureJoystick.setStatusJoystick(FeatureJoystick.Left_offset_Mask,FeatureJoystick.Left_offset_OFF);
                                }
                            }
                            else
                            {
                                selectedButton.setText(" Analog Left ");
                                selectedLinearLayout.setVisibility(View.VISIBLE);
                                mAllAnalogDrawable.setVisibleL(false);
                                mAllAnalogDrawable.setVisibleSmallL(false);
                                if(!mAnalogDirectionR)
                                {
                                    mAllAnalogDrawable.setVisibility(View.INVISIBLE);
                                }

                                if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                                {
                                    mFeatureJoystick.setStatusJoystick(FeatureJoystick.Left_offset_Mask,FeatureJoystick.Left_offset_ON);
                                }
                            }

                            mAnalogDirectionL = !mAnalogDirectionL;

                            if(mAnalogDirectionR)
                            {
                                drawAnalogActivatorR();
                            }
                        }
                    });
                    break;
                case R.id.Button_DAR:
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Button selectedButton = findViewById(R.id.Button_DAR);
                            mColor = argb(255,255,255,0);
                            mySetColorButton(selectedButton,mColor);
                            mSelectedButton = selectedButton;
                            mScrollTask.cancel();
                            mScrollTask = new myScrollTask();
                            if(graphicsEnable)
                            {
                                mColor = argb(0,255,255,255);
                            }
                            else
                            {
                                mColor = argb(255,0,0x99,0xcc);
                            }
                            mScrollTimer.schedule(mScrollTask, mTimerValue); // msec

                            //digital/analog
                            LinearLayout selectedLinearLayout = findViewById(R.id.tableLayout_DirectionR);
                            if(!mAnalogDirectionR)
                            {
                                selectedButton.setText(" Offset Right ");
                                selectedLinearLayout.setVisibility(View.INVISIBLE);

                                drawAnalogActivatorR();

                                if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                                {
                                    mFeatureJoystick.setStatusJoystick(FeatureJoystick.Right_offset_Mask,FeatureJoystick.Right_offset_OFF);
                                }
                            }
                            else
                            {
                                selectedButton.setText(" Analog Right ");
                                selectedLinearLayout.setVisibility(View.VISIBLE);
                                mAllAnalogDrawable.setVisibleR(false);
                                mAllAnalogDrawable.setVisibleSmallR(false);
                                if(!mAnalogDirectionL)
                                {
                                    mAllAnalogDrawable.setVisibility(View.INVISIBLE);
                                }

                                if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                                {
                                    mFeatureJoystick.setStatusJoystick(FeatureJoystick.Right_offset_Mask,FeatureJoystick.Right_offset_ON);
                                }
                            }
                            mAnalogDirectionR = !mAnalogDirectionR;

                            if(mAnalogDirectionL)
                            {
                                drawAnalogActivatorL();
                            }
                        }
                    });
                    break;
                case R.id.Button_UP:
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Button selectedButton = findViewById(R.id.Button_UP);
                            mColor = argb(255,0,0xdd,0xff);
                            mySetColorButton(selectedButton,mColor);
                            mSelectedButton = selectedButton;
                            mScrollTask.cancel();
                            mScrollTask = new myScrollTask();
                            mColor = argb(255,0,0x99,0xcc);
                            mScrollTimer.schedule(mScrollTask, mTimerValue); // msec
                            if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                            {
                                mOffsetAxisY++;
                                if(mOffsetAxisY> 255)
                                    mOffsetAxisY = 255;
                                mFeatureJoystick.setOffsetAxisY(mOffsetAxisY);
                                mFeatureJoystick.writeDigitalButtomJoystick(FeatureJoystick.YPOS);
                            }
                        }
                    });
                    break;
                case R.id.Button_Down:
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Button selectedButton = findViewById(R.id.Button_Down);
                            mColor = argb(255,0,0xdd,0xff);
                            mySetColorButton(selectedButton,mColor);
                            mSelectedButton = selectedButton;
                            mScrollTask.cancel();
                            mScrollTask = new myScrollTask();
                            mColor = argb(255,0,0x99,0xcc);
                            mScrollTimer.schedule(mScrollTask, mTimerValue); // msec
                            if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                            {
                                mOffsetAxisY--;
                                if(mOffsetAxisY< 0)
                                    mOffsetAxisY = 0;
                                mFeatureJoystick.setOffsetAxisY(mOffsetAxisY);
                                mFeatureJoystick.writeDigitalButtomJoystick(FeatureJoystick.YNEG);
                            }
                        }
                    });
                    break;
                case R.id.Button_L:
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Button selectedButton = findViewById(R.id.Button_L);
                            mColor = argb(255,0,0xdd,0xff);
                            mySetColorButton(selectedButton,mColor);
                            mSelectedButton = selectedButton;
                            mScrollTask.cancel();
                            mScrollTask = new myScrollTask();
                            mColor = argb(255,0,0x99,0xcc);
                            mScrollTimer.schedule(mScrollTask, mTimerValue); // msec
                            if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                            {
                                mOffsetRotY--;
                                if(mOffsetRotY< -128)
                                    mOffsetRotY = -128;
                                mFeatureJoystick.setOffsetRotY(mOffsetRotY);
                                mFeatureJoystick.writeDigitalButtomJoystick(FeatureJoystick.ROTYNEG);
                            }
                        }
                    });
                    break;
                case R.id.Button_R:
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Button selectedButton = findViewById(R.id.Button_R);
                            mColor = argb(255,0,0xdd,0xff);
                            mySetColorButton(selectedButton,mColor);
                            mSelectedButton = selectedButton;
                            mScrollTask.cancel();
                            mScrollTask = new myScrollTask();
                            mColor = argb(255,0,0x99,0xcc);
                            mScrollTimer.schedule(mScrollTask, mTimerValue); // msec
                            if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                            {
                                mOffsetRotY++;
                                if(mOffsetRotY> 127)
                                    mOffsetRotY = 127;
                                mFeatureJoystick.setOffsetRotY(mOffsetRotY);
                                mFeatureJoystick.writeDigitalButtomJoystick(FeatureJoystick.ROTYPOS);
                            }
                        }
                    });
                    break;
                case R.id.Button_UP2:
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Button selectedButton = findViewById(R.id.Button_UP2);
                            mColor = argb(255,0,0xdd,0xff);
                            mySetColorButton(selectedButton,mColor);
                            mSelectedButton = selectedButton;
                            mScrollTask.cancel();
                            mScrollTask = new myScrollTask();
                            mColor = argb(255,0,0x99,0xcc);
                            mScrollTimer.schedule(mScrollTask, mTimerValue); // msec
                            if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                            {
                                mOffsetAxisZ++;
                                if(mOffsetAxisZ> 127)
                                    mOffsetAxisZ = 127;
                                mFeatureJoystick.setOffsetAxisZ(mOffsetAxisZ);
                                mFeatureJoystick.writeDigitalButtomJoystick(FeatureJoystick.ZPOS);
                            }
                        }
                    });
                    break;
                case R.id.Button_Down2:
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Button selectedButton = findViewById(R.id.Button_Down2);
                            mColor = argb(255,0,0xdd,0xff);
                            mySetColorButton(selectedButton,mColor);
                            mSelectedButton = selectedButton;
                            mScrollTask.cancel();
                            mScrollTask = new myScrollTask();
                            mColor = argb(255,0,0x99,0xcc);
                            mScrollTimer.schedule(mScrollTask, mTimerValue); // msec
                            if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                            {
                                mOffsetAxisZ--;
                                if(mOffsetAxisZ< -128)
                                    mOffsetAxisZ = -128;
                                mFeatureJoystick.setOffsetAxisZ(mOffsetAxisZ);
                                mFeatureJoystick.writeDigitalButtomJoystick(FeatureJoystick.ZNEG);
                            }
                        }
                    });
                    break;
                case R.id.Button_Left2:
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Button selectedButton = findViewById(R.id.Button_Left2);
                            mColor = argb(255,0,0xdd,0xff);
                            mySetColorButton(selectedButton,mColor);
                            mSelectedButton = selectedButton;
                            mScrollTask.cancel();
                            mScrollTask = new myScrollTask();
                            mColor = argb(255,0,0x99,0xcc);
                            mScrollTimer.schedule(mScrollTask, mTimerValue); // msec
                            if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                            {
                                mOffsetAxisX--;
                                if(mOffsetAxisX< -128)
                                    mOffsetAxisX = -128;
                                mFeatureJoystick.setOffsetAxisX(mOffsetAxisX);
                                mFeatureJoystick.writeDigitalButtomJoystick(FeatureJoystick.XNEG);
                            }
                        }
                    });
                    break;
                case R.id.Button_Right2:
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Button selectedButton = findViewById(R.id.Button_Right2);
                            mColor = argb(255,0,0xdd,0xff);
                            mySetColorButton(selectedButton,mColor);
                            mSelectedButton = selectedButton;
                            mScrollTask.cancel();
                            mScrollTask = new myScrollTask();
                            mColor = argb(255,0,0x99,0xcc);
                            mScrollTimer.schedule(mScrollTask, mTimerValue); // msec
                            if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                            {
                                mOffsetAxisX++;
                                if(mOffsetAxisX> 127)
                                    mOffsetAxisX = 127;
                                mFeatureJoystick.setOffsetAxisX(mOffsetAxisX);
                                mFeatureJoystick.writeDigitalButtomJoystick(FeatureJoystick.XPOS);
                            }
                        }
                    });
                    break;
                case R.id.Button_Connection:
                    //if ((mNode == null)||(mNode.getState() != Node.State.Connected))
                    {
                        mFirstTime = false;
                        Intent i = ScanActivity.getStartIntent(Joystick.this);
                        startActivity(i);
                    }
                    break;
                case R.id.Button_Take_off:
                if ((mNode != null)&&(mNode.getState() == Node.State.Connected))
                {
                    Button selectedButton = findViewById(R.id.Button_Take_off);
                    if(!mButton_Take_off_status)
                    {
                        if(!graphicsEnable)
                            selectedButton.setText("   Land   ");
                        selectedButton.setCompoundDrawablesWithIntrinsicBounds(0,R.drawable.software_drone_land,0,0);
                        if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                        {
                            mFeatureJoystick.setStatusJoystick(FeatureJoystick.Takeoff_Mask,FeatureJoystick.Takeoff_ON);
                        }
                    }
                    else
                    {
                        if(!graphicsEnable)
                            selectedButton.setText(" Take off ");
                        selectedButton.setCompoundDrawablesWithIntrinsicBounds(0,R.drawable.software_drone_take_off,0,0);
                        if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                        {
                            mFeatureJoystick.setStatusJoystick(FeatureJoystick.Takeoff_Mask,FeatureJoystick.Takeoff_OFF);
                        }
                    }

                    mButton_Take_off_status = !mButton_Take_off_status;

                }
                break;
                case R.id.Button_Arm:
                if ((mNode != null)&&(mNode.getState() == Node.State.Connected))
                {
                    Button selectedButton = findViewById(R.id.Button_Arm);
                    if(!mButton_Arm_status)
                    {
                        if(!graphicsEnable)
                            selectedButton.setText(" Stop ");
                        mySetColorButton(selectedButton,argb(255,255,0,0));

                        if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                        {
                            mFeatureJoystick.setStatusJoystick(FeatureJoystick.Arm_Mask,FeatureJoystick.Arm_ON);
                        }
                    }
                    else
                    {
                        if(!graphicsEnable)
                            selectedButton.setText(" Arm  ");
                        mySetColorButton(selectedButton,argb(255,0,0x99,0xcc)); //"@android:color/holo_blue_dark"
                        if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                        {
                            mFeatureJoystick.setStatusJoystick(FeatureJoystick.Arm_Mask,FeatureJoystick.Arm_OFF);
                        }
                    }

                    mButton_Arm_status = !mButton_Arm_status;
                }
                break;
                case R.id.Button_Setting:
                //if ((mNode == null)||(mNode.getState() != Node.State.Connected))
                {
                    mButton_Setting_status = true;
                    Intent i = Setting.getStartIntent(Joystick.this,mNode);
                    if (mNode != null)
                        mNodeContainer.keepConnectionOpen(true);
                    Log.d("notification", this+" Button_Setting "+mButton_Setting_status); // for debug
                    startActivity(i);
                }
                break;
                case R.id.Button_Help:
                //if ((mNode == null)||(mNode.getState() != Node.State.Connected))
                {
                    mButton_Help_status = true;
                    Intent i = Help.getStartIntent(Joystick.this,mNode);
                    if (mNode != null)
                        mNodeContainer.keepConnectionOpen(true);
                    startActivity(i);
                }
                break;
            }
        }
    };




    private Button.OnTouchListener  mOnTouchListener_Button_Sensor_Calibration =   new Button.OnTouchListener() {
        @Override
        public boolean onTouch(View v, MotionEvent event) {
            long lastDuration = 0;
            if(event.getAction() == MotionEvent.ACTION_DOWN) {
                mLastDown = System.currentTimeMillis();
            } else if (event.getAction() == MotionEvent.ACTION_UP) {
                lastDuration = System.currentTimeMillis() - mLastDown;
                if(lastDuration > 1000)
                {
                    if ((mNode != null)&&(mNode.getState() == Node.State.Connected))
                    {
                        if ((mFeatureJoystick != null)&&mFeatureJoystick.isEnabled())
                        {
                            mFeatureJoystick.setStatusJoystick(FeatureJoystick.Calib_Mask,FeatureJoystick.Calib_ON);
                        }
                    }
                    Button selectedButton = findViewById(R.id.Button_Sensor_Calibration);
                    mySetColorButton(selectedButton,argb(255,0,255,0)); // green
                    mButton_Sensor_Calibration_Task.cancel();
                    mButton_Sensor_Calibration_Task = new myButton_Sensor_Calibration_Task();
                    mScrollTimer.schedule(mButton_Sensor_Calibration_Task,2000); // msec
                }
                else
                {
                    Button selectedButton = findViewById(R.id.Button_Sensor_Calibration);
                    mySetColorButton(selectedButton,argb(255,255,0,0)); // red
                    mButton_Sensor_Calibration_Fail_Task.cancel();
                    mButton_Sensor_Calibration_Fail_Task = new myButton_Sensor_Calibration_Fail_Task();
                    mScrollTimer.schedule(mButton_Sensor_Calibration_Fail_Task, 2000); // msec
                }
            }

            return true;
        }
    };

    private boolean displayAnalogL(float x,float y,int centerx, int centery){

        boolean handled = true;
        mCenterX = centerx;
        mCenterY = centery;

        mCorrectObjectL = true;
        float yori = y;
        if(mSwapCommandcode == PLANE_XZ) {
            y -= (mDeltaY-mAxisY);
        }

       if(mFollowerCursor)
        {
            if((x >= mLeftL)&&(x <= mRightL)&&(y >= mTop)&&(yori >= mTop)&&(yori <= mBottom))
            {
                mLeftSmallL = mCenterX-mWidthDirectionL+mWidthSmallDirectionL;
                mRightSmallL = mCenterX+mWidthDirectionL-mWidthSmallDirectionL;
                mBottomSmallL = mCenterY+mHeightDirectionL-mHeightSmallDirectionL;
                mTopSmallL = mCenterY-mHeightDirectionL+mHeightSmallDirectionL;
                if(mSwapCommandcode == PLANE_XZ) {
                    mBottomSmallL -= (mDeltaY-mAxisY);
                    mTopSmallL -= (mDeltaY-mAxisY);
                }
            }
            else
            {
                mCorrectObjectL = false;
            }
        }
        else
        {
            if((x >= (mStartXL-mWidthDirectionL))&&(x <= (mStartXL+mWidthDirectionL))&&(yori >= (mStartYL-mHeightDirectionL))&&(yori <= (mStartYL+mHeightDirectionL)))
            {
                mLeftSmallL = mStartXL-mWidthDirectionL+mWidthSmallDirectionL;
                mRightSmallL = mStartXL+mWidthDirectionL-mWidthSmallDirectionL;
                mBottomSmallL = mStartYL+mHeightDirectionL-mHeightSmallDirectionL;
                mTopSmallL = mStartYL-mHeightDirectionL+mHeightSmallDirectionL;
            }
            else
            {
                mCorrectObjectL = false;
            }
        }

        if(mCorrectObjectL){
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                if(mSwapCommandcode == PLANE_XZ) {
                    if(mFollowerCursor)
                    {
                        mAllAnalogDrawable.setBoundColorL(mCenterX,mCenterY-(mDeltaY-mAxisY),mHeightDirectionL,mWidthDirectionL,argb(255,0,221,255));
                    }
                    else
                    {
                        mAllAnalogDrawable.setBoundColorL(mStartXL,mStartYL,mHeightDirectionL,mWidthDirectionL,argb(255,0,221,255));
                    }
                }
                else
                {
                    if(mFollowerCursor)
                    {
                        mAllAnalogDrawable.setBoundColorL(mCenterX,mCenterY,mHeightDirectionL,mWidthDirectionL,argb(255,0,221,255));
                    }
                    else
                    {
                        mAllAnalogDrawable.setBoundColorL(mStartXL,mStartYL,mHeightDirectionL,mWidthDirectionL,argb(255,0,221,255));
                    }
                }
                mAllAnalogDrawable.setVisibleL(true);

                mAllAnalogDrawable.setBoundColorSmallL(mCenterX,mCenterY,mHeightSmallDirectionL,mWidthSmallDirectionL,argb(255,0,100,128));
                mAllAnalogDrawable.setVisibleSmallL(true);

                mAllAnalogDrawable.invalidate();
                }
            });
            if(mSwapCommandcode == PLANE_XZ) {
                mAxisY = mBottomSmallL-mCenterY;
            }
        }
        else
        {
            handled = false;
        }
        return handled;
    }

    private boolean displayAnalogR(float x,float y,int centerx, int centery){

        boolean handled = true;
        mCenterX = centerx;
        mCenterY = centery;

        mCorrectObjectR = true;
        float yori = y;
        if(mSwapCommandcode != PLANE_XZ) {
            y -= (mDeltaY-mAxisY);
        }
        if(mFollowerCursor)
        {
            if((x >= mLeftR)&&(x <= mRightR)&&(y >= mTop)&&(yori >= mTop)&&(yori <= mBottom))
            {
                mLeftSmallR = mCenterX-mWidthDirectionR+mWidthSmallDirectionR;
                mRightSmallR = mCenterX+mWidthDirectionR-mWidthSmallDirectionR;
                mBottomSmallR = mCenterY+mHeightDirectionR-mHeightSmallDirectionR;
                mTopSmallR = mCenterY-mHeightDirectionR+mHeightSmallDirectionR;
                if(mSwapCommandcode != PLANE_XZ) {
                    mBottomSmallR -= (mDeltaY-mAxisY);
                    mTopSmallR -= (mDeltaY-mAxisY);
                }
            }
            else
            {
                mCorrectObjectR = false;
            }
        }
        else
        {
            if((x >= (mStartXR-mWidthDirectionR))&&(x <= (mStartXR+mWidthDirectionR))&&(yori >= (mStartYR-mHeightDirectionR))&&(yori <= (mStartYR+mHeightDirectionR)))
            {
                mLeftSmallR = mStartXR-mWidthDirectionR+mWidthSmallDirectionR;
                mRightSmallR = mStartXR+mWidthDirectionR-mWidthSmallDirectionR;
                mBottomSmallR = mStartYR+mHeightDirectionR-mHeightSmallDirectionR;
                mTopSmallR = mStartYR-mHeightDirectionR+mHeightSmallDirectionR;
            }
            else
            {
                mCorrectObjectR = false;
            }
        }

        if(mCorrectObjectR){
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                if(mSwapCommandcode != PLANE_XZ) {
                    if(mFollowerCursor)
                    {
                        mAllAnalogDrawable.setBoundColorR(mCenterX,mCenterY-(mDeltaY-mAxisY),mHeightDirectionR,mWidthDirectionR,argb(255,0,221,255));
                    }
                    else
                    {
                        mAllAnalogDrawable.setBoundColorR(mStartXR,mStartYR,mHeightDirectionR,mWidthDirectionR,argb(255,0,221,255));
                    }
                }
                else
                {
                    if(mFollowerCursor)
                    {
                        mAllAnalogDrawable.setBoundColorR(mCenterX,mCenterY,mHeightDirectionR,mWidthDirectionR,argb(255,0,221,255));
                    }
                    else
                    {
                        mAllAnalogDrawable.setBoundColorR(mStartXR,mStartYR,mHeightDirectionR,mWidthDirectionR,argb(255,0,221,255));
                    }
                }
                mAllAnalogDrawable.setVisibleR(true);

                mAllAnalogDrawable.setBoundColorSmallR(mCenterX,mCenterY,mHeightSmallDirectionR,mWidthSmallDirectionR,argb(255,0,100,128));
                mAllAnalogDrawable.setVisibleSmallR(true);

                mAllAnalogDrawable.invalidate();
                }
            });
            if(mSwapCommandcode != PLANE_XZ) {
                mAxisY = mBottomSmallR-mCenterY;
            }
        }
        else
        {
            handled = false;
        }
        return handled;
    }

    void coordShapeRemapping(float x, float y, int rightSmall, int leftSmall, int bottomSmall, int topSmall)
    {
        double remapx;
        double remapy;
        if(mShapeCircle)
        {
            float cx = (float)(rightSmall+leftSmall)*0.5f;
            float cy = (float)(bottomSmall+topSmall)*0.5f;
            float dx = (x-cx);
            float dy = (y-cy);
            float dx2 = (dx*dx);
            float dy2 = (dy*dy);
            double diag = sqrt(dx2+dy2);
            float radius = (float)(rightSmall-leftSmall)*0.5f;
            if(diag > radius){
                // clipping
                dx = radius*dx/(float)diag;
                dy = radius*dy/(float)diag;
                dx2 = (dx*dx);
                dy2 = (dy*dy);
                diag = sqrt(dx2+dy2);
                mCenterX = (int)(cx+dx+0.5f);
                mCenterY = (int)(cy+dy+0.5f);
            }

            // dx/(radius*cos) = remapx/radius, so remapx/dx = 1/cos, so remapy must be with same factor remapy/dy = 1/cos
            // where cos = dx/diag
            // after 45degree x and y are swapped
            if(dx2 > dy2)
            {
                if(dx > 0){
                    remapy = dy/dx*diag;
                    remapx = cx+diag;
                }
                else{
                    remapy = -dy/dx*diag;
                    remapx = cx-diag;
                }
                remapy += cy;
            }
            else
            {
                if(dy > 0){
                    remapx = dx/dy*diag;
                    remapy = cy+diag;
                }
                else{
                    remapx = -dx/dy*diag;
                    remapy = cy-diag;
                }
                remapx += cx;
            }
            mRemapCenterX = (int) (remapx+0.5f);
            mRemapCenterY = (int) (remapy+0.5f);
        }
        else
        {
            remapx = x;
            remapy = y;
            if(remapx < leftSmall)
                mCenterX = leftSmall;
            if(remapx > rightSmall)
                mCenterX = rightSmall;
            if(remapy < topSmall)
                mCenterY = topSmall;
            if(remapy > bottomSmall)
                mCenterY = bottomSmall;
            mRemapCenterX = mCenterX;
            mRemapCenterY = mCenterY;
        }
    }

    private AllAnalogDrawable.OnTouchListener  mOnTouchListener =   new AllAnalogDrawable.OnTouchListener() {
        @Override
        public boolean onTouch(View v, MotionEvent event) {

            boolean handled = false;
            boolean handledL = false;
            boolean handledR = false;
            int visibiltyStatus = v.getVisibility();
            if(visibiltyStatus == View.VISIBLE) {

                boolean cursorLeft;
                int mNumTouch = event.getPointerCount();
                Log.d("multiple_touch", "numTouch = "+String.valueOf(mNumTouch)); // for debug
                for(int i=0; i < mNumTouch; i++) // for multiple touch
                {
                    float x = event.getX(i);
                    float y = event.getY(i);
                    mCenterX = (int) x;
                    mCenterY = (int) y;

                    Log.d("multiple_touch", "index = "+String.valueOf(event.getPointerId(i))); // for debug
                    if(mCenterX <= mWidthPixels/2)
                    {
                        cursorLeft = true;

                        if(mAllAnalogDrawable.getVisibleSmallL()) {
                            // MotionEvent.ACTION_POINTER_DOWN doesn't work need this check
                            if ((event.getAction() == MotionEvent.ACTION_DOWN) || (event.getAction() == MotionEvent.ACTION_POINTER_DOWN) || ((mNumTouch == 2) && (mNumTouchOld == 1) && (!mCursorLeftOld))) {
                                // first touch
                                handledL = displayAnalogL(x, y, mCenterX, mCenterY);
                                mAnalogOnGoing = true;

                                if (handledL)
                                    mAnalogActivatorL = false;
                            }

                            if (event.getAction() == MotionEvent.ACTION_MOVE) {
                                handledL = true;
                                if (mFollowerCursor) {
                                    // keep touch
                                    boolean vibrationOn = false;
                                    if ((mCenterX <= 20) && (mCenterX > mLeftSmallL))
                                        vibrationOn = true;
                                    if (mCenterX >= (mWidthPixels / 2 - 20))
                                        vibrationOn = true;
                                    if (((mCenterY + mTop) >= (mHeightPixels - 100)) && ((mCenterY + mTop) < mBottomSmallL))
                                        vibrationOn = true;
                                    if (((mCenterY + mTop) <= 20) && ((mCenterY + mTop) > mTopSmallL))
                                        vibrationOn = true;
                                    if (vibrationOn) {
                                        Vibrator vibration = (Vibrator) getSystemService(Context.VIBRATOR_SERVICE);
                                        // Vibrate for 500 milliseconds
                                        vibration.vibrate(500);
                                    }
                                }

                                 boolean shapeCircle = mShapeCircle;
                                if (mYawDisalble)
                                    mShapeCircle = false;
                                coordShapeRemapping(x, y, mRightSmallL, mLeftSmallL, mBottomSmallL, mTopSmallL);
                                if (mYawDisalble)
                                    mShapeCircle = shapeCircle;
                                mCorrectSmallObject = true;

                                if (mCorrectSmallObject) {
                                    runOnUiThread(new Runnable() {
                                        @Override
                                        public void run() {
                                            Log.d("drawAnalog", "mAnalogSmallDirectionDrawableViewL"); // for debug
                                            mAllAnalogDrawable.setBoundColorSmallL(mCenterX, mCenterY, mHeightSmallDirectionL, mWidthSmallDirectionL, argb(255, 0, 100, 128));
                                            mAllAnalogDrawable.setVisibleSmallL(true);
                                            mAllAnalogDrawable.invalidate();

                                            if ((mFeatureJoystick != null) && mFeatureJoystick.isEnabled()) {
                                                int valuexLin = (int) (255.0 * (mRemapCenterX - mLeftSmallL) / (float) (mRightSmallL - mLeftSmallL) + 0.5);
                                                int valueyLin = (int) (255.0 * (mBottomSmallL - mRemapCenterY) / (float) (mBottomSmallL - mTopSmallL) + 0.5);

                                                //non linear values
                                                int valuex = 0;
                                                int valuey = 0;
                                                switch (mSwapCommandcode) {
                                                    case PLANE_XY:
                                                        valuex = NonLinerarValueCentralCalculation(mRemapCenterX, mLeftSmallL, mRightSmallL, -1);
                                                        valuey = NonLinerarValueCentralCalculation(-mRemapCenterY, -mBottomSmallL, -mTopSmallL, 1);
                                                        break;
                                                    case PLANE_XZ:
                                                        valuex = NonLinerarValueCentralCalculation(mRemapCenterX, mLeftSmallL, mRightSmallL, -1);
                                                        valuey = NonLinerarValueYCalculation(-mRemapCenterY, -mBottomSmallL, -mTopSmallL, 0);
                                                        break;
                                                    case PLANE_ZY:
                                                        valuex = NonLinerarValueCentralCalculation(mRemapCenterX, mLeftSmallL, mRightSmallL, -1);
                                                        valuey = NonLinerarValueCentralCalculation(-mRemapCenterY, -mBottomSmallL, -mTopSmallL, 1);
                                                        break;
                                                }

                                                valuex = (int) (((float) (valuex-128) * mProgressSeekBar) * 0.01+128 + 0.5);
                                                if (mSwapCommandcode == PLANE_XZ)
                                                    valuey = (int) (((float) valuey * mProgressSeekBar) * 0.01 + 0.5);
                                                else
                                                    valuey = (int) (((float) (valuey-128) * mProgressSeekBar) * 0.01+128 + 0.5);

                                                mValuexL = valuex;
                                                mValueyL = valuey;
                                                mValueChangedL = true;

                                                Log.d("DEBUG_TAG", "The linear valuex L is " + String.valueOf(valuexLin - 128));
                                                Log.d("DEBUG_TAG", "The non linear valuex L is " + String.valueOf(valuex - 128));
                                                Log.d("DEBUG_TAG", "ratio non linear/linear valuex L is " + String.valueOf((float) (valuex - 128) / (float) (valuexLin - 128)));
                                                if (mSwapCommandcode == PLANE_XZ) {
                                                    Log.d("DEBUG_TAG", "The linear valuey L is " + String.valueOf(valueyLin));
                                                    Log.d("DEBUG_TAG", "The non linear valuey L is " + String.valueOf(valuey));
                                                    Log.d("DEBUG_TAG", "ratio non linear/linear L valuey is " + String.valueOf((float) valuey / (float) valueyLin));
                                                } else {
                                                    Log.d("DEBUG_TAG", "The linear valuey L is " + String.valueOf(valueyLin - 128));
                                                    Log.d("DEBUG_TAG", "The non linear valuey L is " + String.valueOf(valuey - 128));
                                                    Log.d("DEBUG_TAG", "ratio non linear/linear valuey L is " + String.valueOf((float) (valuey - 128) / (float) (valueyLin - 128)));
                                                }

                                                if ((!mAnalogActivatorL) && (!mAnalogActivatorR)) {

                                                    if (mValueChangedL && mValueChangedR) {
                                                        Log.d("DEBUG_TAG", " left both"); // for debug
                                                        switch (mSwapCommandcode) {
                                                            case PLANE_XY:
                                                                mFeatureJoystick.writeAnalogJoystickFull(FeatureJoystick.ROTY_ANALOG_INDEX, mValuexL, FeatureJoystick.ZAXIS_ANALOG_INDEX, mValueyL,
                                                                        FeatureJoystick.XAXIS_ANALOG_INDEX, mValuexR, FeatureJoystick.YAXIS_ANALOG_INDEX, mValueyR, mNode);
                                                                break;
                                                            case PLANE_XZ:
                                                                mFeatureJoystick.writeAnalogJoystickFull(FeatureJoystick.ROTY_ANALOG_INDEX, mValuexL, FeatureJoystick.YAXIS_ANALOG_INDEX, mValueyL,
                                                                        FeatureJoystick.XAXIS_ANALOG_INDEX, mValuexR, FeatureJoystick.ZAXIS_ANALOG_INDEX, mValueyR, mNode);
                                                                break;
                                                            case PLANE_ZY:
                                                                mFeatureJoystick.writeAnalogJoystickFull(FeatureJoystick.ROTY_ANALOG_INDEX, mValuexL, FeatureJoystick.XAXIS_ANALOG_INDEX, mValueyL,
                                                                        FeatureJoystick.ZAXIS_ANALOG_INDEX, mValuexR, FeatureJoystick.YAXIS_ANALOG_INDEX, mValueyR, mNode);
                                                                break;
                                                        }
                                                        mValueChangedL = false;
                                                        mValueChangedR = false;
                                                    }
                                                } else {
                                                    Log.d("DEBUG_TAG", " left single"); // for debug

                                                    switch (mSwapCommandcode) {
                                                        case PLANE_XY:
                                                            mFeatureJoystick.writeAnalogJoystickDouble(FeatureJoystick.ROTY_ANALOG_INDEX, valuex, FeatureJoystick.ZAXIS_ANALOG_INDEX, valuey, mNode, true);
                                                            break;
                                                        case PLANE_XZ:
                                                            mFeatureJoystick.writeAnalogJoystickDouble(FeatureJoystick.ROTY_ANALOG_INDEX, valuex, FeatureJoystick.YAXIS_ANALOG_INDEX, valuey, mNode, true);
                                                            break;
                                                        case PLANE_ZY:
                                                            mFeatureJoystick.writeAnalogJoystickDouble(FeatureJoystick.ROTY_ANALOG_INDEX, valuex, FeatureJoystick.XAXIS_ANALOG_INDEX, valuey, mNode, true);
                                                            break;
                                                    }
                                                }
                                            }

                                            if (mSwapCommandcode == PLANE_XZ)
                                                mAxisY = mBottomSmallL - mCenterY;
                                        }
                                    });
                                }
                            }
                            if ((event.getAction() == MotionEvent.ACTION_UP) || (event.getAction() == MotionEvent.ACTION_POINTER_UP)) {
                                Log.d("multiple_touch", "end with index = " + String.valueOf(event.getPointerId(i))); // for debug
                                // last touch
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {

                                        drawAnalogActivatorL();

                                        if ((mFeatureJoystick != null) && mFeatureJoystick.isEnabled()) {
                                            int valuex = 128;
                                            int valuey = 0;

                                            switch (mSwapCommandcode) {
                                                case PLANE_XY:
                                                    mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.ROTY_ANALOG_INDEX, valuex);
                                                    //mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.ZAXIS_ANALOG_INDEX,valuey);
                                                    break;
                                                case PLANE_XZ:
                                                    mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.ROTY_ANALOG_INDEX, valuex);
                                                    //mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.YAXIS_ANALOG_INDEX,valuey);
                                                    break;
                                                case PLANE_ZY:
                                                    mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.ROTY_ANALOG_INDEX, valuex);
                                                    //mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.XAXIS_ANALOG_INDEX,valuey);
                                                    break;
                                            }
                                        }
                                    }
                                });
                                handledL = false;
                                mAnalogOnGoing = false;

                                mAnalogActivatorL = true;
                                mValueChangedL = false;
                                mValueChangedR = false;
                            }
                        }

                        if(mAllAnalogDrawable.getVisibleSmallR()) {
                            // MotionEvent.ACTION_POINTER_UP doesn't work need this check
                            if ((event.getAction() == MotionEvent.ACTION_UP) || (event.getAction() == MotionEvent.ACTION_POINTER_UP) || ((mNumTouch == 1) && (mNumTouchOld == 2))) {
                                //it was right
                                Log.d("multiple_touch", "end with index = " + String.valueOf(event.getPointerId(i))); // for debug
                                // last touch
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {

                                        drawAnalogActivatorR();

                                        if ((mFeatureJoystick != null) && mFeatureJoystick.isEnabled()) {
                                            int valuex = 128;
                                            int valuey = 128;

                                            switch (mSwapCommandcode) {
                                                case PLANE_XY:
                                                    mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.XAXIS_ANALOG_INDEX, valuex);
                                                    //mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.YAXIS_ANALOG_INDEX,valuey);
                                                    break;
                                                case PLANE_XZ:
                                                    mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.XAXIS_ANALOG_INDEX, valuex);
                                                    //mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.ZAXIS_ANALOG_INDEX,valuey);
                                                    break;
                                                case PLANE_ZY:
                                                    mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.ZAXIS_ANALOG_INDEX, valuex);
                                                    //mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.YAXIS_ANALOG_INDEX,valuey);
                                                    break;
                                            }
                                        }
                                    }
                                });
                                handledR = false;
                                mAnalogOnGoing = false;

                                mAnalogActivatorR = true;
                                mValueChangedL = false;
                                mValueChangedR = false;
                            }
                        }
                    }
                    else
                    {
                        cursorLeft = false;

                        if(mAllAnalogDrawable.getVisibleSmallR()) {
                            // MotionEvent.ACTION_POINTER_DOWN doesn't work need this check
                            if ((event.getAction() == MotionEvent.ACTION_DOWN) || (event.getAction() == MotionEvent.ACTION_POINTER_DOWN) || ((mNumTouch == 2) && (mNumTouchOld == 1) && (mCursorLeftOld))) {
                                // first touch
                                handledR = displayAnalogR(x, y, mCenterX, mCenterY);
                                mAnalogOnGoing = true;

                                if (handledR)
                                    mAnalogActivatorR = false;
                            }

                            if (event.getAction() == MotionEvent.ACTION_MOVE) {
                                handledR = true;
                                if (mFollowerCursor) {
                                    // keep touch
                                    boolean vibrationOn = false;
                                    if ((mCenterX >= (mWidthPixels - 20)) && (mCenterX < mRightSmallR))
                                        vibrationOn = true;
                                    if (mCenterX <= (mWidthPixels / 2 - 20))
                                        vibrationOn = true;
                                    if (((mCenterY + mTop) >= (mHeightPixels - 100)) && ((mCenterY + mTop) < mBottomSmallR))
                                        vibrationOn = true;
                                    if (((mCenterY + mTop) <= 20) && ((mCenterY + mTop) > mTopSmallR))
                                        vibrationOn = true;
                                    if (vibrationOn) {
                                        Vibrator vibration = (Vibrator) getSystemService(Context.VIBRATOR_SERVICE);
                                        // Vibrate for 500 milliseconds
                                        vibration.vibrate(500);
                                    }
                                }
                                coordShapeRemapping(x, y, mRightSmallR, mLeftSmallR, mBottomSmallR, mTopSmallR);
                                mCorrectSmallObject = true;

                                if (mCorrectSmallObject) {
                                    runOnUiThread(new Runnable() {
                                        @Override
                                        public void run() {
                                            mAllAnalogDrawable.setBoundColorSmallR(mCenterX, mCenterY, mHeightSmallDirectionR, mWidthSmallDirectionR, argb(255, 0, 100, 128));
                                            mAllAnalogDrawable.setVisibleSmallR(true);
                                            mAllAnalogDrawable.invalidate();
                                            if ((mFeatureJoystick != null) && mFeatureJoystick.isEnabled()) {
                                                // linear values
                                                int valuexLin = (int) (255.0 * (mRemapCenterX - mLeftSmallR) / (float) (mRightSmallR - mLeftSmallR) + 0.5);
                                                int valueyLin = (int) (255.0 * (mBottomSmallR - mRemapCenterY) / (float) (mBottomSmallR - mTopSmallR) + 0.5);

                                                //non linear values
                                                int valuex = 0;
                                                int valuey = 0;
                                                switch (mSwapCommandcode) {
                                                    case PLANE_XY:
                                                        valuex = NonLinerarValueCentralCalculation(mRemapCenterX, mLeftSmallR, mRightSmallR, 1);
                                                        valuey = NonLinerarValueYCalculation(-mRemapCenterY, -mBottomSmallR, -mTopSmallR, 0);
                                                        break;
                                                    case PLANE_XZ:
                                                        valuex = NonLinerarValueCentralCalculation(mRemapCenterX, mLeftSmallR, mRightSmallR, 1);
                                                        valuey = NonLinerarValueCentralCalculation(-mRemapCenterY, -mBottomSmallR, -mTopSmallR, 1);
                                                        break;
                                                    case PLANE_ZY:
                                                        valuex = NonLinerarValueCentralCalculation(mRemapCenterX, mLeftSmallR, mRightSmallR, 1);
                                                        valuey = NonLinerarValueYCalculation(-mRemapCenterY, -mBottomSmallR, -mTopSmallR, 0);
                                                        break;
                                                }

                                                valuex = (int) (((float) (valuex-128) * mProgressSeekBar) * 0.01+128 + 0.5);
                                                if (mSwapCommandcode != PLANE_XZ)
                                                    valuey = (int) (((float) valuey * mProgressSeekBar) * 0.01 + 0.5);
                                                else
                                                    valuey = (int) (((float) (valuey-128) * mProgressSeekBar) * 0.01+128 + 0.5);
                                                mValuexR = valuex;
                                                mValueyR = valuey;
                                                mValueChangedR = true;

                                                Log.d("DEBUG_TAG", "The linear valuex R is " + String.valueOf(valuexLin - 128));
                                                Log.d("DEBUG_TAG", "The non linear valuex R  is " + String.valueOf(valuex - 128));
                                                Log.d("DEBUG_TAG", "ratio non linear/linear valuex R  is " + String.valueOf((float) (valuex - 128) / (float) (valuexLin - 128)));
                                                if (mSwapCommandcode != PLANE_XZ) {
                                                    Log.d("DEBUG_TAG", "The linear valuey R is " + String.valueOf(valueyLin));
                                                    Log.d("DEBUG_TAG", "The non linear valuey R  is " + String.valueOf(valuey));
                                                    Log.d("DEBUG_TAG", "ratio non linear/linear valuey R  is " + String.valueOf((float) valuey / (float) valueyLin));
                                                } else {
                                                    Log.d("DEBUG_TAG", "The linear valuey R  is " + String.valueOf(valueyLin - 128));
                                                    Log.d("DEBUG_TAG", "The non linear valuey R  is " + String.valueOf(valuey - 128));
                                                    Log.d("DEBUG_TAG", "ratio non linear/linear valuey R  is " + String.valueOf((float) (valuey - 128) / (float) (valueyLin - 128)));
                                                }

                                                if ((!mAnalogActivatorL) && (!mAnalogActivatorR)) {

                                                    if (mValueChangedL && mValueChangedR) {
                                                        Log.d("DEBUG_TAG", " right both"); // for debug
                                                        switch (mSwapCommandcode) {
                                                            case PLANE_XY:
                                                                mFeatureJoystick.writeAnalogJoystickFull(FeatureJoystick.ROTY_ANALOG_INDEX, mValuexL, FeatureJoystick.ZAXIS_ANALOG_INDEX, mValueyL,
                                                                        FeatureJoystick.XAXIS_ANALOG_INDEX, mValuexR, FeatureJoystick.YAXIS_ANALOG_INDEX, mValueyR, mNode);
                                                                break;
                                                            case PLANE_XZ:
                                                                mFeatureJoystick.writeAnalogJoystickFull(FeatureJoystick.ROTY_ANALOG_INDEX, mValuexL, FeatureJoystick.YAXIS_ANALOG_INDEX, mValueyL,
                                                                        FeatureJoystick.XAXIS_ANALOG_INDEX, mValuexR, FeatureJoystick.ZAXIS_ANALOG_INDEX, mValueyR, mNode);
                                                                break;
                                                            case PLANE_ZY:
                                                                mFeatureJoystick.writeAnalogJoystickFull(FeatureJoystick.ROTY_ANALOG_INDEX, mValuexL, FeatureJoystick.XAXIS_ANALOG_INDEX, mValueyL,
                                                                        FeatureJoystick.ZAXIS_ANALOG_INDEX, mValuexR, FeatureJoystick.YAXIS_ANALOG_INDEX, mValueyR, mNode);
                                                                break;
                                                        }
                                                        mValueChangedL = false;
                                                        mValueChangedR = false;
                                                    }
                                                } else {
                                                    Log.d("DEBUG_TAG", " right single"); // for debug

                                                    switch (mSwapCommandcode) {
                                                        case PLANE_XY:
                                                            mFeatureJoystick.writeAnalogJoystickDouble(FeatureJoystick.XAXIS_ANALOG_INDEX, valuex, FeatureJoystick.YAXIS_ANALOG_INDEX, valuey, mNode, false);
                                                            break;
                                                        case PLANE_XZ:
                                                            mFeatureJoystick.writeAnalogJoystickDouble(FeatureJoystick.XAXIS_ANALOG_INDEX, valuex, FeatureJoystick.ZAXIS_ANALOG_INDEX, valuey, mNode, false);
                                                            break;
                                                        case PLANE_ZY:
                                                            mFeatureJoystick.writeAnalogJoystickDouble(FeatureJoystick.ZAXIS_ANALOG_INDEX, valuex, FeatureJoystick.YAXIS_ANALOG_INDEX, valuey, mNode, false);
                                                            break;
                                                    }
                                                }
                                            }

                                            if (mSwapCommandcode != PLANE_XZ)
                                                mAxisY = mBottomSmallR - mCenterY;
                                        }
                                    });
                                }
                            }
                            if ((event.getAction() == MotionEvent.ACTION_UP) || (event.getAction() == MotionEvent.ACTION_POINTER_UP)) {
                                Log.d("multiple_touch", "end with index = " + String.valueOf(event.getPointerId(i))); // for debug
                                // last touch
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {

                                        drawAnalogActivatorR();

                                        if ((mFeatureJoystick != null) && mFeatureJoystick.isEnabled()) {
                                            int valuex = 128;
                                            int valuey = 128;

                                            switch (mSwapCommandcode) {
                                                case PLANE_XY:
                                                    mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.XAXIS_ANALOG_INDEX, valuex);
                                                    //mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.YAXIS_ANALOG_INDEX,valuey);
                                                    break;
                                                case PLANE_XZ:
                                                    mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.XAXIS_ANALOG_INDEX, valuex);
                                                    //mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.ZAXIS_ANALOG_INDEX,valuey);
                                                    break;
                                                case PLANE_ZY:
                                                    mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.ZAXIS_ANALOG_INDEX, valuex);
                                                    //mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.YAXIS_ANALOG_INDEX,valuey);
                                                    break;
                                            }
                                        }
                                    }
                                });
                                handledR = false;
                                mAnalogOnGoing = false;

                                mAnalogActivatorR = true;
                                mValueChangedL = false;
                                mValueChangedR = false;
                            }
                        }

                        if(mAllAnalogDrawable.getVisibleSmallL()) {
                            // MotionEvent.ACTION_POINTER_UP doesn't work need this check
                            if ((event.getAction() == MotionEvent.ACTION_UP) || (event.getAction() == MotionEvent.ACTION_POINTER_UP) || ((mNumTouch == 1) && (mNumTouchOld == 2))) {
                                //it was left
                                Log.d("multiple_touch", "end with index = " + String.valueOf(event.getPointerId(i))); // for debug
                                // last touch
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {

                                        drawAnalogActivatorL();

                                        if ((mFeatureJoystick != null) && mFeatureJoystick.isEnabled()) {
                                            int valuex = 128;
                                            int valuey = 0;

                                            switch (mSwapCommandcode) {
                                                case PLANE_XY:
                                                    mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.ROTY_ANALOG_INDEX, valuex);
                                                    //mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.ZAXIS_ANALOG_INDEX,valuey);
                                                    break;
                                                case PLANE_XZ:
                                                    mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.ROTY_ANALOG_INDEX, valuex);
                                                    //mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.YAXIS_ANALOG_INDEX,valuey);
                                                    break;
                                                case PLANE_ZY:
                                                    mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.ROTY_ANALOG_INDEX, valuex);
                                                    //mFeatureJoystick.writeAnalogJoystick(FeatureJoystick.XAXIS_ANALOG_INDEX,valuey);
                                                    break;
                                            }
                                        }
                                    }
                                });
                                handledL = false;
                                mAnalogOnGoing = false;

                                mAnalogActivatorL = true;
                                mValueChangedL = false;
                                mValueChangedR = false;
                            }
                        }
                    }
                    mCursorLeftOld = cursorLeft;
                }
                mNumTouchOld = mNumTouch;
            }
            handled = handledL || handledR;
            return handled;
        }
    };


    int NonLinerarValueYCalculation(int v, int v0, int v1, int axis_selector)
    {
        float factor_f = 0,coeff_a = 0,coeff_b = 0,coeff_c = 0;
        switch(axis_selector) {
            case -1:
                factor_f = mFactor_f_Yaw;
                coeff_a = mCoeff_a_Yaw;
                coeff_b = mCoeff_b_Yaw;
                coeff_c = mCoeff_c_Yaw;
                break;
            case 0:
                factor_f = mFactor_f_y;
                coeff_a = mCoeff_a_y;
                coeff_b = mCoeff_b_y;
                coeff_c = mCoeff_c_y;
                break;
            case 1:
                factor_f = mFactor_f_xz;
                coeff_a = mCoeff_a_xz;
                coeff_b = mCoeff_b_xz;
                coeff_c = mCoeff_c_xz;
                break;
        }
        float t = (float)(v - v0)/(float)(v1-v0);
        float x = (1-t+t*(factor_f-1));
        float y = -coeff_a/x+coeff_b*x+coeff_c; // 0 ... 1
        int value = (int)(255.0*y+0.5);

        return value;
    }

    int NonLinerarValueCentralCalculation(int v, int v0, int v1, int axis_selector)
    {
        int value;
        float factor_f = 0,coeff_a = 0,coeff_b = 0,coeff_c = 0;
        switch(axis_selector) {
            case -1:
                factor_f = mFactor_f_Yaw;
                coeff_a = mCoeff_a_Yaw;
                coeff_b = mCoeff_b_Yaw;
                coeff_c = mCoeff_c_Yaw;
                break;
            case 0:
                factor_f = mFactor_f_y;
                coeff_a = mCoeff_a_y;
                coeff_b = mCoeff_b_y;
                coeff_c = mCoeff_c_y;
                break;
            case 1:
                factor_f = mFactor_f_xz;
                coeff_a = mCoeff_a_xz;
                coeff_b = mCoeff_b_xz;
                coeff_c = mCoeff_c_xz;
                break;
        }
        float v0f = (float) (v0+v1)*0.5f;
        float v1f = (float) (v1-v0)*0.5f+v0f;
        float t = (v - v0f)/(v1f-v0f);
        boolean signNeg = (t < 0);
        if (signNeg)
            t = -t;
        float x = (1-t+t*(factor_f-1));
        float y = -coeff_a/x+coeff_b*x+coeff_c; // 0 ... 1
        if (signNeg)
        {
            value = (int)(-127.0*y+128-0.5);
        }
        else
        {
            value = (int)(127.0*y+128+0.5);
        }

        return value;
    }

}



