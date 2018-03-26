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

import android.os.Bundle;
import android.preference.PreferenceActivity;
import android.support.annotation.NonNull;

import com.st.BlueSTSDK.Node;

import java.util.List;


/**
 * A {@link PreferenceActivity} that presents a set of application settings. On
 * handset devices, settings are presented as a single list. On tablets,
 * settings are split by category, with category headers shown to the left of
 * the list of settings.
 * <p/>
 * See <a href="http://developer.android.com/design/patterns/settings.html">
 * Android Design: Settings</a> for design guidelines and the <a
 * href="http://developer.android.com/guide/topics/ui/settings.html">Settings
 * API Guide</a> for more information on developing a Settings UI.
 */
public class SettingsActivity extends PreferenceActivity {

    private final static String NODE_FRAGMENT = SettingsActivity.class.getCanonicalName()
            +".NODE_FRAGMENT";

    private final static String NODE_CONTAINER_ARGS = SettingsActivity.class.getCanonicalName()
            +".NODE_CONTAINER_ARGS";

    private final static String NODE_HAS_REGISTER = SettingsActivity.class.getCanonicalName()
            +".NODE_HAS_REGISTER";

    /**
     * fragment that will contain the node and start the connection with it
     */
    private NodeContainerFragment mNodeContainer;

    /** create an intent for start the activity that contains the configuration option for the app
     *
     * @param c context used for create the intent
     * @param node note that will be used by the activity
     * @return intent for start this activity
     */
    public static Intent getStartIntent(Context c,@NonNull Node node){
        Intent i = new Intent(c,SettingsActivity.class);
        //DevicePreferenceFragment.appendArguments(i,node);
        i.putExtra(NODE_CONTAINER_ARGS,NodeContainerFragment.prepareArguments(node));
        DevicePreferenceFragment.appendArguments(i,node);
        i.putExtra(NODE_HAS_REGISTER,node.getConfigRegister()!=null);
        return  i;
    }

    /**
     * is is not already created it instantiate a new node container,
     * but the fragment will not be added to the activity
     */
    private void initializeNodeContainer(){
        //try to recover an already created fragment otherwise we create a new one

    }

    /**
     * create a new node container
     * @param savedInstanceState
     */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (savedInstanceState == null) {
            Intent i = getIntent();
            mNodeContainer = new NodeContainerFragment();
            mNodeContainer.setArguments(i.getBundleExtra(NODE_CONTAINER_ARGS));

            getFragmentManager().beginTransaction()
                    .add(mNodeContainer, NODE_FRAGMENT).commit();

        } else {
            mNodeContainer = (NodeContainerFragment) getFragmentManager()
                    .findFragmentByTag(NODE_FRAGMENT);

        }//if-else

    }

    /**
     * in case of small screen this activity will be recreate for show a new fragment with the
     * selected options, we propagate the useful information to the fragment

     * @return intent that will re start the the activity with the proper fragment
     */
    @Override
    public Intent onBuildStartFragmentIntent (String fragmentName, Bundle args, int titleRes, int
            shortTitleRes){
        Intent i = super.onBuildStartFragmentIntent(fragmentName,args,titleRes,shortTitleRes);
        mNodeContainer.keepConnectionOpen(true);
        if(fragmentName.equals(DevicePreferenceFragment.class.getName())) {
            DevicePreferenceFragment.appendArguments(i, mNodeContainer.getNode());
            i.putExtra(NODE_CONTAINER_ARGS, getIntent().getBundleExtra(NODE_CONTAINER_ARGS));
            i.putExtra(NODE_HAS_REGISTER,getIntent().getBooleanExtra(NODE_HAS_REGISTER,false));
        }

        return i;
    }

    /**
     * Populate the activity with the top-level headers.
     */
    @Override
    public void onBuildHeaders(List<Header> target) {
        //create the register configuration only if needed
        if(getIntent().getBooleanExtra(NODE_HAS_REGISTER, false))
            loadHeadersFromResource(R.xml.pref_headers_configuration, target);
    }

    @Override
    protected  boolean isValidFragment (String fragmentName){
        return fragmentName.equals(DevicePreferenceFragment.class.getName()) ;
    }


    /**
     * if we have to leave this activity, we force the disconnection of the node
     */
    @Override
    public void onBackPressed(){
        mNodeContainer.keepConnectionOpen(true);
        super.onBackPressed();
    }

}
