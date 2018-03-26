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

import android.app.Activity;
import android.content.Context;
import android.content.res.Resources;
import android.graphics.drawable.Drawable;
import android.support.annotation.NonNull;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.ImageView;
import android.widget.TextView;

import com.st.BlueSTSDK.Manager;
import com.st.BlueSTSDK.Node;

/**
 * class that map a node into a view with the layout defined in node_view_item.
 * this class can be set as a {@link com.st.BlueSTSDK.Manager.ManagerListener} for automatically add
 * a node when it is discovered
 * */
public class NodeArrayAdapter extends ArrayAdapter<Node> implements Manager.ManagerListener {

    /**
     * activity where this adapter is attached
     */
    private Activity mActivity;
    /**
     * image to show for the different boards
     */
    private Drawable mNucleoImage;
    private Drawable mSTEVAL_WESU1_Image;
    private Drawable mGenericImage;

    /**
     * build the adapter
     *
     * @param context context where the adapter will be used
     */
    public NodeArrayAdapter(Activity context) {
        super(context, R.layout.node_view_item);
        mActivity = context;
        Resources res = mActivity.getResources();
        mNucleoImage = res.getDrawable(R.drawable.board_nucleo);
        mSTEVAL_WESU1_Image = res.getDrawable(R.drawable.board_steval_wesu1);
        mGenericImage = res.getDrawable(R.drawable.board_generic);
    }//NodeArrayAdapter

    /** empty function */
    @Override
    public void onDiscoveryChange(Manager m, boolean enabled) {
    }

    /**
     * new node discovered -> we add it to the adapter
     *
     * @param m    manager that discover the node
     * @param node new node discovered
     */
    @Override
    public void onNodeDiscovered(Manager m, final Node node) {
        mActivity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                add(node);
            }//run
        });
    }

    /**
     * disconnect al connected node manage by this adapter
     */
    void disconnectAllNodes() {
        for (int i = 0; i < getCount(); i++) {
            Node n = getItem(i);
            if (n!=null && n.isConnected())
                n.disconnect();
        }//for
    }//disconnectAllNodes

    /**
     * create a view that describe a particular node
     *
     * @param position position that have to be build
     * @param v        where store the information
     * @param parent   group where the view will be stored
     * @return a view that contains the information about the node in position \code{position}
     */
    @NonNull
    @Override
    public View getView(int position, View v, @NonNull ViewGroup parent) {

        ViewHolderItem viewHolder;

        if (v == null) {
            LayoutInflater inflater = (LayoutInflater) getContext().getSystemService(Context.LAYOUT_INFLATER_SERVICE);
            v = inflater.inflate(R.layout.node_view_item, parent, false);
            viewHolder = new ViewHolderItem();
            viewHolder.sensorName = (TextView) v.findViewById(R.id.nodeName);
            viewHolder.sensorTag = (TextView) v.findViewById(R.id.nodeTag);
            viewHolder.boardType = (ImageView) v.findViewById(R.id.nodeBoard);
            v.setTag(viewHolder);
        } else {
            //else -> is a recycled view -> we have only to update the values
            viewHolder = (ViewHolderItem) v.getTag();
        }//if-else

        //get the corresponding sensor
        Node sensor = getItem(position);

        viewHolder.sensorName.setText(sensor.getName());
        viewHolder.sensorTag.setText(sensor.getTag());
        switch (sensor.getType()) {

            case STEVAL_WESU1:
                viewHolder.boardType.setImageDrawable(mSTEVAL_WESU1_Image);
                break;
            case NUCLEO:
                viewHolder.boardType.setImageDrawable(mNucleoImage);
                break;
            case GENERIC:
            default:
                viewHolder.boardType.setImageDrawable(mGenericImage);
        }//switch

        return v;
    }//getView

    /**
     * class that contains view that we have to change between different items
     */
    private static class ViewHolderItem {
        TextView sensorName;
        TextView sensorTag;
        ImageView boardType;
    }//ViewHolderItem

}// NodeArrayAdapter class
