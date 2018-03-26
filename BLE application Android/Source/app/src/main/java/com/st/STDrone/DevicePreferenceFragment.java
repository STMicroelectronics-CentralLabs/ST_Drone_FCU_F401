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

import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.preference.Preference;
import android.preference.PreferenceFragment;

import com.st.BlueSTSDK.Config.Command;
import com.st.BlueSTSDK.ConfigControl;
import com.st.BlueSTSDK.Config.Register;
import com.st.BlueSTSDK.Config.STWeSU.RegisterDefines;
import com.st.BlueSTSDK.Features.Field;
import com.st.BlueSTSDK.Manager;
import com.st.BlueSTSDK.Node;

/**
 * fragment that contains the log preference -> where and how store the data
 */
public class DevicePreferenceFragment extends PreferenceFragment {

    private static final String NODE_TAG = DevicePreferenceFragment.class.getCanonicalName()+"" +
            ".NODE_TAG";
    private static final String NODE_BUNDLE = DevicePreferenceFragment.class.getCanonicalName()
            +".NODE_BUNDLE";

    public static void appendArguments(Intent i,Node n){
        Bundle args = new Bundle();
        args.putString(NODE_TAG, n.getTag());
        i.putExtra(NODE_BUNDLE,args);
    }


    private SharedPreferences.OnSharedPreferenceChangeListener prefListener = new SharedPreferences.OnSharedPreferenceChangeListener() {
        @Override
        public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
            setPreferenceSummaryText(key,sharedPreferences.toString());
        }
    };


    /**
     * node that we will configure
     */
    private Node mNode;

    /**
     * ble service used for configure the node
     */
    private ConfigControl mConfigService;

    private void initializeConfigService(Node n){
        if(n!=null && n.isConnected()) {
            mConfigService = n.getConfigRegister();
            if (mConfigService != null)
                mConfigService.addConfigListener(configControl);
            //if

            //start the read after 500ms
            new Handler().postDelayed(new Runnable() {
                @Override
                public void run() {
                    getActivity().runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            getRegValueFromDevicePersistent(RegisterDefines.RegistersName.FW_VER);
                        }
                    });
                }
            }, 500);
        }//if
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        addPreferencesFromResource(R.xml.pref_configuration);

        String nodeTag = getActivity().getIntent().getBundleExtra(NODE_BUNDLE).getString(NODE_TAG);
        mNode = Manager.getSharedInstance().getNodeWithTag(nodeTag);
    }

    private void getRegValueFromDevicePersistent(RegisterDefines.RegistersName reg){
        Command cmdVer = new Command(reg.getRegister(), Register.Target.PERSISTENT);
        mConfigService.read(cmdVer);
    }

    private Node.NodeStateListener mNodeStatusListener = new Node.NodeStateListener() {
        @Override
        public void onStateChange(Node node, Node.State newState, Node.State prevState) {
            if (newState == Node.State.Connected) {
                initializeConfigService(node);
            }
        }
    };

    @Override
    public void onResume(){
        super.onResume();
        if(mNode.isConnected())
            initializeConfigService(mNode);
        else
            mNode.addNodeStateListener(mNodeStatusListener);

        getPreferenceManager().getSharedPreferences().registerOnSharedPreferenceChangeListener(prefListener);

        getPreferenceManager().findPreference("DEVICE_LOCAL_NAME").setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
            @Override
            public boolean onPreferenceChange(Preference preference, Object newValue) {

                if (newValue != null) {
                    String text = (String) newValue;
                    if (text.length() > 0) {
                        byte[] toSend = new byte[16]; //to allign to two byte
                        toSend[0] = 0x09;
                        System.arraycopy(text.getBytes(), 0, toSend, 1, Math.min(text.length(), 15));
                        Command cmd = new Command(RegisterDefines.RegistersName.BLE_LOC_NAME.getRegister(),
                                Register.Target.PERSISTENT, toSend);
                        mConfigService.write(cmd);
                    }
                }
                return false;
            }
        });
        getPreferenceManager().findPreference("DEVICE_PUB_ADDR").setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
            @Override
            public boolean onPreferenceChange(Preference preference, Object newValue) {
                if (newValue != null) {
                    String text = ((String) newValue).toLowerCase();

                    if (text.matches("[0-9a-f][0-9a-f]:[0-9a-f][0-9a-f]:[0-9a-f][0-9a-f]:[0-9a-f][0-9a-f]:[0-9a-f][0-9a-f]:[0-9a-f][0-9a-f]")) {

                        String [] strArr = text.split(":");
                        byte [] toSend = new byte[strArr.length];

                        if (toSend.length > 0) {
                            for (int i = 0; i < toSend.length; i++)
                                toSend[toSend.length -i-1] = (byte)(Short.parseShort(strArr[i],16) & 0xFF);
                            Command cmd = new Command(RegisterDefines.RegistersName.BLE_PUB_ADDR
                                    .getRegister(), Register.Target.PERSISTENT, toSend);
                            mConfigService.write(cmd);

                        }
                    }
                }
                return false;
            }
        });

        getPreferenceManager().findPreference("DEVICE_LED_CONFIG_S").setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
            @Override
            public boolean onPreferenceChange(Preference preference, Object newValue) {
                short val = Short.parseShort((String) newValue, 16);
                Command cmd = new Command(RegisterDefines.RegistersName.LED_CONFIG.getRegister(),
                        Register.Target.PERSISTENT, val, Field.Type.Int16);
                mConfigService.write(cmd);

                return false;
            }
        });
        getPreferenceManager().findPreference("DEVICE_LOW_POWER_S").setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
            @Override
            public boolean onPreferenceChange(Preference preference, Object newValue) {
                short val = Short.parseShort((String) newValue);
                Command cmd = new Command(RegisterDefines.RegistersName.PWR_MODE_CONFIG.getRegister(),
                        Register.Target.PERSISTENT, val, Field.Type.Int16);
                mConfigService.write(cmd);

                return false;
            }
        });
        getPreferenceManager().findPreference("DEVICE_DFU_S").setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
            @Override
            public boolean onPreferenceChange(Preference preference, Object newValue) {
                short val = Short.parseShort((String) newValue);
                Command cmd = new Command(RegisterDefines.RegistersName.DFU_REBOOT.getRegister(),
                        Register.Target.SESSION, val, Field.Type.Int16);
                mConfigService.write(cmd);

                return false;
            }
        });
        //if is null the node is not created -> the configService is initialized by the callback
        initializeConfigService(mNode);

    }

    @Override
    public void onPause(){

        if(mConfigService!=null)
            mConfigService.removeConfigListener(configControl);

        mNode.removeNodeStateListener(mNodeStatusListener);

        getPreferenceManager().getSharedPreferences().unregisterOnSharedPreferenceChangeListener(prefListener);
        super.onPause();
    }

    private void setPreferenceSummaryText(final String key, final String txt){
        getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Preference pref = getPreferenceManager().findPreference(key);
                if (pref != null) {
                    pref.setSummary(txt);
                    pref.setDefaultValue(txt);
                }
            }
        });
    }
    private ConfigControl.ConfigControlListener configControl = new ConfigControl.ConfigControlListener() {
        @Override
        public void onRegisterReadResult(ConfigControl config,Command cmd, int error) {
            if (error == 0) {
                if (cmd.getRegister().equals(RegisterDefines.RegistersName.FW_VER.getRegister())) {
                    String strVer = String.format("Version: %X.%X.%02X",
                            ((cmd.getData()[1] >> 4) & 0x0F),
                            ((cmd.getData()[1]) & 0x0F),
                            (cmd.getData()[0] & 0xFF));
                    setPreferenceSummaryText("DEVICE_FW_VERSION", strVer);

                    getRegValueFromDevicePersistent(RegisterDefines.RegistersName.BLE_LOC_NAME);
                }
                if (cmd.getRegister().equals((RegisterDefines.RegistersName.BLE_LOC_NAME
                        .getRegister()))) {

                    setPreferenceSummaryText("DEVICE_LOCAL_NAME", String.copyValueOf(cmd.getDataChar(), 1, Math.min(15, cmd.getDataChar().length)));

                    getRegValueFromDevicePersistent(RegisterDefines.RegistersName.BLE_PUB_ADDR);
                }
                if (cmd.getRegister().equals((RegisterDefines.RegistersName.BLE_PUB_ADDR.getRegister()))) {
                    String strAddr = "";
                    for (int i = 0; i < cmd.getData().length; i++) {
                        strAddr += String.format(i < cmd.getData().length - 1 ? "%02X:" : "%02X", cmd.getData()[cmd.getData().length - i - 1]);
                    }
                    setPreferenceSummaryText("DEVICE_PUB_ADDR", strAddr);

                    //getRegValueFromDevicePersistent(RegisterDefines.RegistersName.FW_VER);
                }
            }
        }

        @Override
        public void onRegisterWriteResult(ConfigControl config,Command cmd, int error) {
            if (error == 0)
                mConfigService.read(cmd);

        }

        @Override
        public void onRequestResult(ConfigControl config,Command cmd, boolean success) {

        }

    };
}//LogPreferenceFragment