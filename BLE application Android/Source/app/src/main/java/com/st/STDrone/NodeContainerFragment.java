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
import android.app.Fragment;
import android.app.ProgressDialog;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;

import com.st.BlueSTSDK.Manager;
import com.st.BlueSTSDK.Node;

/**
 * This is headless fragment that is used for store a connected node. This fragment will not be
 * destroyed when an activity is destroyed for change its configuration -> using this fragment you
 * avoid to connect/disconnect multiple times in a short time..
 * <p>
 *      This class will start the connection with the node inside the onCreate and close it inside the
 *      onDestroy.
 * </p>
 * <p>
 *     If you move in another activity that will use the same node you can avoid to disconnect calling
 *     the method {@link NodeContainerFragment#keepConnectionOpen(boolean)}
 * </p>
 * <p>
 *     This fragment will automatically restart the connection when an error status is detected
 * </p>
 */
public class NodeContainerFragment extends Fragment {
    /**
     * string used for store our data in the fragment args
     */
    final static String NODE_TAG = NodeContainerFragment.class.getCanonicalName() + ".NODE_TAG";

    /**
     * true if the user ask to skip the disconnect when the fragment is destroyed
     */
    boolean userAskToKeepConnection = false;
    /**
     * progress dialog to show when we wait that the node connection
     */
    //private ProgressDialog mConnectionWait;
    /**
     * node handle by this class
     */
    private Node mNode = null;

    /**
     * node listener that will manage the dialog + pass the data to the user listener if it is set
     */
    private Node.NodeStateListener mNodeStateListener = new Node.NodeStateListener() {
        @Override
        public void onStateChange(final Node node, Node.State newState, Node.State prevState) {
            final Activity activity = NodeContainerFragment.this.getActivity();
            Log.d("nodeContainer","State"+newState);
            //we connect -> hide the dialog
            if ((newState == Node.State.Connected) && activity != null) {
                activity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        //close the progress dialog
                        //mConnectionWait.dismiss();
                        //mConnectionWait = null;
                    }
                });
            //error state -> show a toast message and start a new connection
            } else if ((newState == Node.State.Unreachable ||
                    newState == Node.State.Dead ||
                    newState == Node.State.Lost) && activity != null) {
                final String msg;
                switch (newState) {
                    case Dead:
                        msg = String.format(getResources().getString(R.string.progressDialogConnMsgDeadNodeError),
                                node.getName());
                        break;
                    case Unreachable:
                        msg = String.format(getResources().getString(R.string.progressDialogConnMsgUnreachableNodeError),
                                node.getName());
                        break;
                    case Lost:
                    default:
                        msg = String.format(getResources().getString(R.string
                                        .progressDialogConnMsgLostNodeError),
                                node.getName());
                        break;
                }//switch

                activity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
//                        if(mConnectionWait == null) {
//                            setUpProgressDialog(node.getName());
//                            //mConnectionWait.show();
//                        }
//                        else if (!mConnectionWait.isShowing())
//                            mConnectionWait.show();
                        Toast.makeText(activity, msg, Toast.LENGTH_LONG).show();
                        Log.d("nodeContainer","connect");
                        mNode.connect(getActivity().getApplicationContext());
                    }
                });
            }
        }//onStateChange
    };

    /**
     * prepare the arguments to pass to this fragment
     *
     * @param n node that this fragment has to manage
     * @return bundle to pass as argument to a NodeContainerFragment
     */
    public static Bundle prepareArguments(Node n) {
        Bundle args = new Bundle();
        args.putString(NODE_TAG, n.getTag());
        return args;
    }

//    /**
//     * Prepare the progress dialog tho be shown setting the title and the message
//     *
//     * @param nodeName name of the node that we will use
//     */
//    private void setUpProgressDialog(String nodeName) {
//        mConnectionWait = new ProgressDialog(getActivity(), ProgressDialog.STYLE_SPINNER);
//        mConnectionWait.setTitle(R.string.progressDialogConnTitle);
//        mConnectionWait.setMessage(String.format(getResources().getString(R.string
//                        .progressDialogConnMsg),
//                nodeName));
//    }//setUpProgressDialog

    /**
     * return the node handle by this fragment
     *
     * @return return the node handle by this fragment
     */
    public Node getNode() {
        return mNode;
    }

    /**
     * set this fragment as retain state + recover the node from the manager
     *
     * @param savedInstanceState data stored from the previous instance (not used)
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setRetainInstance(true);
        String nodeTag = getArguments().getString(NODE_TAG);
        mNode = Manager.getSharedInstance().getNodeWithTag(nodeTag);
//        if (mNode != null)
//            setUpProgressDialog(mNode.getName());
    }//onCreate

    /**
     * if not already connected, show the dialog and stat the connection with the node
     */
    @Override
    public void onResume() {
        super.onResume();
        if (mNode != null && !mNode.isConnected()) {
//            if(mConnectionWait == null) {
//                setUpProgressDialog(mNode.getName());
//                //mConnectionWait.show();
//            }
//            else if (!mConnectionWait.isShowing())
//                mConnectionWait.show();

            mNode.connect(getActivity().getApplicationContext());
        }
        if (mNode != null) {
            mNode.addNodeStateListener(mNodeStateListener);
        }//if
    }//onResume


    /**
     * if we are still connection hide the progress dialog
     */
    @Override
    public void onPause() {
        if (mNode != null) {
            mNode.removeNodeStateListener(mNodeStateListener);
        }//if
//        //dismiss the dialog if we are showing it
//        if (mConnectionWait.isShowing()) {
//            mConnectionWait.dismiss();
//            mConnectionWait = null;
//        }//if

        super.onPause();
    }//onPause

    /**
     * if true avoid to disconnect the node when the fragment is destroyed
     * @param doIt true for skip the disconnect, false for disconnect, default = false
     */
    public void keepConnectionOpen(boolean doIt) {
        userAskToKeepConnection = doIt;
    }//keepConnectionOpen

    //@Override
    public void onDisconnection() {
        if (mNode != null) {
                mNode.disconnect();
        }//if
    }

    /**
     * if we are connected we disconnect the node
     */
    @Override
    public void onDestroy() {

        if (mNode != null && mNode.isConnected()) {
            if (!userAskToKeepConnection)
            {
                Log.d("notification", "onDestroy "+mNode.getState()); // for debug

//                mNode.removeNodeStateListener(mNodeStateListener);
                mNode.disconnect();
            }
        }//if

        super.onDestroy();
    }//onDestroy

}

