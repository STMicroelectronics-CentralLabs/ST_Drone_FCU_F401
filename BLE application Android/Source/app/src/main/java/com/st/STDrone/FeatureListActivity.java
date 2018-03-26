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
import android.support.annotation.NonNull;
import android.support.v7.app.AppCompatActivity;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.TextView;

import com.st.BlueSTSDK.Feature;
import com.st.BlueSTSDK.Manager;
import com.st.BlueSTSDK.Node;

import java.util.List;

/**
 * This simple activity show all the features available in a node.
 * When the user select one feature we request to receive the update notification and
 * we display it
 */
public class FeatureListActivity extends AppCompatActivity implements AdapterView.OnItemClickListener {

    /**
     * tag used for retrieve the NodeContainerFragment
     */
    private final static String NODE_FRAGMENT = FeatureListActivity.class.getCanonicalName() + "" +
            ".NODE_FRAGMENT";

    /**
     * tag used for store the node id that permit us to find the node selected by the user
     */
    private final static String NODE_TAG = FeatureListActivity.class.getCanonicalName() + "" +
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

    /**
     * list view where we display the available features exported by the node
     */
    private ListView mFeatureList;

    /**
     * adapter that will build the feature item
     */
    private ArrayAdapter<Feature> mFeatureListAdapter;

    /**
     * listener that will be used for enable the notification when the node is connected
     */
    private Node.NodeStateListener mNodeStatusListener = new Node.NodeStateListener() {
        @Override
        public void onStateChange(final Node node, Node.State newState, Node.State prevState) {
            if (newState == Node.State.Connected) {
                FeatureListActivity.this.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        populateFeatureList();
                        invalidateOptionsMenu(); //enable/disable the settings options
                    }//run
                });
            }//if
        }//onStateChange
    };

    /**
     * listener that will update the displayed feature data
     */
    private Feature.FeatureListener mGenericUpdate;

    /**
     * create an intent for start this activity
     *
     * @param c    context used for create the intent
     * @param node node to use for the demo
     * @return intent for start a demo activity that use the node as data source
     */
    public static Intent getStartIntent(Context c, @NonNull Node node) {
        Intent i = new Intent(c, FeatureListActivity.class);
        i.putExtra(NODE_TAG, node.getTag());
        i.putExtras(NodeContainerFragment.prepareArguments(node));
        return i;
    }//getStartIntent

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_demo);

        //load the gui
        mFeatureList = (ListView) findViewById(R.id.featureList);
        mFeatureList.setOnItemClickListener(this);

        //find the node
        String nodeTag = getIntent().getStringExtra(NODE_TAG);
        mNode = Manager.getSharedInstance().getNodeWithTag(nodeTag);

        //create or recover the NodeContainerFragment
        if (savedInstanceState == null) {
            Intent i = getIntent();
            mNodeContainer = new NodeContainerFragment();
            mNodeContainer.setArguments(i.getExtras());

            getFragmentManager().beginTransaction()
                    .add(mNodeContainer, NODE_FRAGMENT).commit();

        } else {
            mNodeContainer = (NodeContainerFragment) getFragmentManager()
                    .findFragmentByTag(NODE_FRAGMENT);

        }//if-else
    }//onCreate

    /**
     * build the menu and show the item only if the service is available in the node
     *
     * @param menu menu where add the items
     */
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_demo, menu);

        menu.findItem(R.id.menu_showDebug).setVisible(mNode.getDebug() != null);
        menu.findItem(R.id.menu_showRegister).setVisible(mNode.getConfigRegister() != null);

        return true;
    }//onCreateOptionMenu

    /**
     * start the activity with the debug console or for manage the configuration register
     *
     * @param item item selected by the user
     * @return true if the item is handle by this method
     */
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();

        //we call keepConnectionOpen for skip the node disconnection when the activity is destroyed
        //in this way we avoid fast connection/disconnection call

        //noinspection SimplifiableIfStatement
        if (id == R.id.menu_showRegister) {
            mNodeContainer.keepConnectionOpen(true);
            startActivity(SettingsActivity.getStartIntent(this, mNode));
            return true;
        }//else
        if (id == R.id.menu_showDebug) {
            mNodeContainer.keepConnectionOpen(true);
            startActivity(DebugConsoleActivity.getStartIntent(this, mNode));
            return true;
        }//else

        return super.onOptionsItemSelected(item);
    }

    /**
     * stop all the enabled notification
     */
    private void disableNeedNotification() {
        List<Feature> features = mNode.getFeatures();
        for (Feature f : features) {
            if (mNode.isEnableNotification(f))
                mNode.disableNotification(f);
        }//for sTestFeature
    }//disableNeedNotification

    /**
     * create and populate the adapter with only the enabled features.
     */
    private void populateFeatureList() {
        if (mNode != null) {
            mFeatureListAdapter = new FeatureAdapter(this,
                    R.layout.feature_list_item);
            List<Feature> features = mNode.getFeatures();
            for (Feature f : features) {
                if (f.isEnabled()) {
                    mFeatureListAdapter.add(f);
                }//if
            }//for

            //set the adapter as data source for the adapter
            mFeatureList.setAdapter(mFeatureListAdapter);

        }//if
    }//populateFeatureList


    /**
     * if the node is connected enable the gui, otherwise set a listener that will do that
     */
    @Override
    protected void onResume() {
        super.onResume();
        if (mNode.isConnected()) {
            populateFeatureList();
            invalidateOptionsMenu(); //enable/disable the settings options
        } else
            mNode.addNodeStateListener(mNodeStatusListener);

    }//onResume


    @Override
    protected void onPause() {

        //it is safe remove also if we didn't add it
        mNode.removeNodeStateListener(mNodeStatusListener);

        //if the node is already disconnected we don't care of disable the notification
        if (mNode.isConnected()) {
            disableNeedNotification();
        }//if

        super.onPause();
    }//stopDemo

    /**
     * When a user select a row we enable/disable the notification for that feature
     *
     * @param adapterView list view
     * @param view        selected view
     * @param position    selected row
     * @param l           selected id
     */
    @Override
    public void onItemClick(AdapterView<?> adapterView, View view, int position, long l) {
        Feature selectedFeature = mFeatureListAdapter.getItem(position);
        if(selectedFeature==null)
            return;

        if (mNode.isEnableNotification(selectedFeature)) {

            selectedFeature.removeFeatureListener(mGenericUpdate);
            mNode.disableNotification(selectedFeature);

            ((TextView) view).setText(selectedFeature.getName()); //reset the cell name
        } else {
            //create a listener that will update the selected view
            mGenericUpdate = new GenericFragmentUpdate((TextView) view);
            selectedFeature.addFeatureListener(mGenericUpdate);

            mNode.enableNotification(selectedFeature);
        }//if-else
    }//onItemClick

    /**
     * extend an array adapter for change the view content, instead of used the toString result
     * we use the feature name
     */
    private static class FeatureAdapter extends ArrayAdapter<Feature> {

        /**
         * @see ArrayAdapter#ArrayAdapter(Context, int)
         */
        FeatureAdapter(Context c, int resourceId) {
            super(c, resourceId);
        }

        /**
         * create a text view and initialize it with the equivalent feature name
         */
        @NonNull
        @Override
        public View getView(int position, View v, @NonNull ViewGroup parent) {

            if (v == null) {
                LayoutInflater inflater = (LayoutInflater) getContext().getSystemService(Context.LAYOUT_INFLATER_SERVICE);
                v = inflater.inflate(R.layout.feature_list_item, parent, false);
            }

            Feature f = getItem(position);

            if(f!=null)
                ((TextView) v).setText(f.getName());

            return v;

        }//getView

    }//FeatureAdapter

    /**
     * class used for update the feature display data
     */
    private class GenericFragmentUpdate implements Feature.FeatureListener {

        /**
         * text view that will contain the data/name
         */
        final private TextView mTextView;

        /**
         * @param text text view that will show the name/values
         */
        GenericFragmentUpdate(TextView text) {
            mTextView = text;
        }//GenericFragmentUpdate

        /**
         * set the text view text with the feature toString value
         *
         * @param f      feature that has received an update
         * @param sample new data received from the feature
         */
        @Override
        public void onUpdate(Feature f, Feature.Sample sample) {
            final String featureDump = f.toString();
            FeatureListActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    mTextView.setText(featureDump);
                }
            });
        }//onUpdate

    }//GenericFragmentUpdate
}
