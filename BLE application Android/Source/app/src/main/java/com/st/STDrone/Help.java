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
import android.support.annotation.NonNull;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.MenuItem;
import android.view.View;

import com.st.BlueSTSDK.Manager;
import com.st.BlueSTSDK.Node;

public class Help  extends AppCompatActivity {

    private final static String NODE_TAG = Help.class.getCanonicalName() + "" +
            ".NODE_TAG";

    private static boolean mNode_status = false;

    /**
     * create an intent for start this activity
     *
     * @param c    context used for create the intent
      * @return intent for start a demo activity that use the node as data source
     */
        public static Intent getStartIntent(Context c, @NonNull Node node) {
        Intent i = new Intent(c, Help.class);
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

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_help);

        findViewById(R.id.Button_Home).setOnClickListener(new View.OnClickListener() {
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
            Intent i = Joystick.getStartIntent(Help.this,n);

            startActivity(i);
            }
        });


        findViewById(R.id.Button_Setting).setOnClickListener(new View.OnClickListener() {
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
            Intent i = Setting.getStartIntent(Help.this,n);
            startActivity(i);
            }
        });
    }

 }



