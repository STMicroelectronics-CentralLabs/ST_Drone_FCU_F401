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
import android.support.annotation.ColorRes;
import android.support.annotation.NonNull;
import android.support.v7.app.AppCompatActivity;
import android.text.Spannable;
import android.text.SpannableStringBuilder;
import android.text.style.ForegroundColorSpan;
import android.view.KeyEvent;
import android.view.MenuItem;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.EditText;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import com.st.BlueSTSDK.Debug;
import com.st.BlueSTSDK.Manager;
import com.st.BlueSTSDK.Node;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Activity that show a console with the stdout and stderr message from the board and permit to
 * send string to the stdin
 */
public class DebugConsoleActivity extends AppCompatActivity {

    private final static String NODE_FRAGMENT = DebugConsoleActivity.class.getCanonicalName() + "" +
            ".NODE_FRAGMENT";

    private final static String NODE_TAG = DebugConsoleActivity.class.getCanonicalName() + "" +
            ".NODE_TAG";
    /**
     * enable the console when the node connect
     */
    private Node.NodeStateListener mNodeStateChangeListener = new Node.NodeStateListener() {
        @Override
        public void onStateChange(Node node, Node.State newState, Node.State prevState) {
            if (newState == Node.State.Connected) {
                setUpConsoleService(node.getDebug());
            } else if (newState == Node.State.Dead) {
                DebugConsoleActivity.this.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(DebugConsoleActivity.this, R.string.DebugNotAvailable,
                                Toast.LENGTH_SHORT).show();
                    }
                });
            }//if-else
        }//onStateChange
    };//NodeStateListener

    /**
     * text view where we will dump the console out
     */
    private TextView mConsole;

    /**
     * scroll view attached to the text view, we use it for keep visualized the last line
     */
    private ScrollView mConsoleView;

    /**
     * text edit where the user will write its commands
     */
    private EditText mUserInput;

    /** node that will send the information */
    private Node mNode;

    /** fragment used for keep the connection open */
    private NodeContainerFragment mNodeContainer;

    /**
     * object that will send/receive commands from the node
     */
    private Debug mDebugService;

    /**
     * message that we are sending
     */
    private String mToSent=null;

    /**
     * index of the last byte send, negative if we didn't send anything
     */
    private int mNextPartToSentIndex = -1;

    /**
     * last time that the user send a message
     */
    private Date mLastMessageSending = new Date();
    /**
     * threshold to use for send an user message also if we are still sending a previous message
     */
    private static final long SENDING_TIME_OUT_MS = 1000; //ms

    private Debug.DebugOutputListener mConsoleListener;


    /**
     * create an intent for start the activity that will log the information from the node
     *
     * @param c    context used for create the intent
     * @param node note that will be used by the activity
     * @return intent for start this activity
     */
    public static Intent getStartIntent(Context c, @NonNull Node node) {
        Intent i = new Intent(c, DebugConsoleActivity.class);
        i.putExtra(NODE_TAG, node.getTag());
        i.putExtras(NodeContainerFragment.prepareArguments(node));
        return i;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        //load the gui
        setContentView(R.layout.activity_debug_console);
        mConsoleView = (ScrollView) findViewById(R.id.consoleView);
        mConsole = (TextView) findViewById(R.id.deviceConsole);
        mUserInput = (EditText) findViewById(R.id.inputText);

        /**
         * when the user click on the button "send" we send the message and delete the text the
         * textview
         */
        mUserInput.setOnEditorActionListener(new TextView.OnEditorActionListener() {
            @Override
            public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
                boolean handled = false;
                if (actionId == EditorInfo.IME_ACTION_SEND) {

                    String toSend = v.getText().toString();
                    v.setText(""); //reset the string
                    if (!toSend.isEmpty())
                        if (!sendMessage(toSend + '\n')) {
                            resetMessageToSend();
                            DebugConsoleActivity.this.runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    Toast.makeText(DebugConsoleActivity.this,
                                            R.string.AnotherMsgIsSending,
                                            Toast.LENGTH_SHORT).show();
                                }
                            });
                        }//if
                    handled = true;
                }//if action
                return handled;
            }//onEditorAction
        });//setOnEditorActionListener

        // recover the node
        String nodeTag = getIntent().getStringExtra(NODE_TAG);
        mNode = Manager.getSharedInstance().getNodeWithTag(nodeTag);

        //create/recover the NodeContainerFragment
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
     * write a message to the stdIn of the debug console prepare the string to sent and check
     * if there is a current message in queue to be sent
     *
     * @param message message to send
     * @return false if there is a message current sending else true
     */
    private boolean sendMessage(String message) {
        Date  now = new Date();
        //not message already sending or time out
        if ((mToSent == null) || (now.getTime() - mLastMessageSending.getTime() > SENDING_TIME_OUT_MS)) {
            resetMessageToSend();
            if (message != null && !message.isEmpty()) {
                mToSent = message;
                mNextPartToSentIndex=0;
                writeNextMessage();
                return true;
            }//if
        }//if
        return false;
    }//sendMessage

    /**
     * clear the current message that we were sending
     */
    private void resetMessageToSend(){
        mToSent = null;
        mNextPartToSentIndex = -1;
    }//resetMessageToSend

    /**
     * write the next part of the current message to the input characteristic
     * @return true if there is another message to send
     */
    //it is synchronized because it is called by a callback and we need to complete all the method
    //before run it again otherwise we can send the same message 2 times or
    private synchronized boolean writeNextMessage() {

        if (mToSent == null )
            return false;

        int byteSend = mDebugService.write(mToSent.substring(mNextPartToSentIndex));
        mNextPartToSentIndex += byteSend;

        if(byteSend<0)
            return false;

        if(mNextPartToSentIndex>=mToSent.length()) {
            resetMessageToSend(); // we sending complete we don't need it
            return false;
        }//if
        return  true;
    }//writeNextMessage

    /**
     * when the node connected check the presence of the debug service and enable the gui if it
     * present otherwise it will show an error message
     *
     * @param debugService debug service return from the node, null if not present
     */
    private void setUpConsoleService(Debug debugService){
        mDebugService=debugService;
        if(mDebugService!=null) {
            mConsoleListener = new UpdateConsole();
            mDebugService.addDebugOutputListener(mConsoleListener);
            DebugConsoleActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    mUserInput.setEnabled(true);
                }
            });
        } else {
            DebugConsoleActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(DebugConsoleActivity.this, R.string.DebugNotAvailable,
                            Toast.LENGTH_SHORT).show();
                }
            });
        }//if-else
    }//setUpConsoleService

    /**
     * if the node is connected enable the gui otherwise attach a listener for do it when the node
     * connects
     */
    @Override
    protected void onResume() {
        super.onResume();

        if (mNode.isConnected()) {
            setUpConsoleService(mNode.getDebug());
        } else
            mNode.addNodeStateListener(mNodeStateChangeListener);
    }//onResume


    /**
     * remove the listener that we add to the node and debug object
     */
    @Override
    protected void onPause() {
        mNode.removeNodeStateListener(mNodeStateChangeListener);

        if (mDebugService != null && mConsoleListener!=null)
            mDebugService.removeDebugOutputListener(mConsoleListener);

        super.onPause();
    }//onPause

    /**
     * if we have to leave this activity, we force to keep the connection open, since we go back
     * in the {@link FeatureListActivity}
     */
    @Override
    public void onBackPressed() {
        mNodeContainer.keepConnectionOpen(true);
        super.onBackPressed();
    }//onBackPressed

    /**
     * call when the user press the back button on the menu bar, we are leaving this activity so
     * we keep the connection open since we are going int the {@link FeatureListActivity}
     *
     * @param item menu item clicked
     * @return true if the item is handle by this function
     */
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {

        switch (item.getItemId()) {
            // Respond to the action bar's Up/Home button, we don't return for permit to
            // do the standard action of this button
            case android.R.id.home:
                mNodeContainer.keepConnectionOpen(true);
        }//switch

        return super.onOptionsItemSelected(item);
    }//onOptionsItemSelected


    /**
     * enum with the information about the type of console available
     */
    private enum ConsoleType{
        /** Node output console */
        OUTPUT,
        /** Node input console */
        INPUT,
        /** Node error console */
        ERROR;

        /**
         * get the color to use for the specific console
         * @return color to use for the text to write in the specific console
         */
        public @ColorRes int getColorID(){
            switch (this){
                case ERROR:
                    return  R.color.ErrorMsg;
                case OUTPUT:
                    return  R.color.OutMsg;
                case INPUT:
                    return  R.color.InMsg;
                default:
                    return R.color.InMsg;
            }//switch
        }//getClassID

        /**
         * get the char to use as prefix fo the specific console
         * @return prefix to use before write a string in the console
         */
        public char getPrefix(){
            if (this != INPUT)
                return  '<';
            return  '>';
        }//getPrefix

    }//ConsoleType

    /**
     * listener for debug message, it will update the textview with received message
     * <p>
     * The different message will be show with different color
     * </p>
     * <p>
     * We avoid to print the date if 2 message from the same stream arrive in a short time frame
     * </p>
     */
    private class UpdateConsole implements Debug.DebugOutputListener{
        //final DateFormat DATE_FORMAT = DateFormat.getDateTimeInstance();
        final SimpleDateFormat DATE_FORMAT = new SimpleDateFormat("yyMMdd HH:mm:ss.SSS",
                Locale.getDefault());

        /** console that we are using for print the message */
        private ConsoleType mTargetConsole = null;

        /**time of the last received message */
        private Date mLastMessageReceived = new Date();
        private  final long PAUSE_DETECTION_TIME_MS = 100; //ms

        /**
         * create the prefix with the current time, it is create only if the last message was
         * received more than PAUSE_DETECTION_TIME_MS ms ago
         * @param append if true force to create the prefix string
         * @return prefix string with the current time
         */
        private String appendDateTime(Boolean append) {
            String str = "";
            Date  now = new Date();
            if (append || (now.getTime() - mLastMessageReceived.getTime() > PAUSE_DETECTION_TIME_MS)) {
                str +="\n[" + DATE_FORMAT.format(now) + mTargetConsole.getPrefix() +"]";
            }
            mLastMessageReceived = now;

            return str;
        }//appendDateTime

        /**
         * append the message to the console
         * @param message message to add
         * @param std console to use for add the string
         */
        private void appendMessage(String message, ConsoleType std)
        {
            boolean forceAppendPrefix = (mTargetConsole != std);
            mTargetConsole = std;

            final SpannableStringBuilder displayText = new SpannableStringBuilder();

            displayText.append(appendDateTime(forceAppendPrefix));
            displayText.append(message);

            displayText.setSpan(
                    new ForegroundColorSpan(getResources().getColor(mTargetConsole.getColorID())),
                    0, displayText.length(),Spannable.SPAN_EXCLUSIVE_EXCLUSIVE);

            DebugConsoleActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    mConsole.append(displayText);
                    mConsoleView.fullScroll(View.FOCUS_DOWN);
                }//run
            });
        }//appendMessage

        @Override
        public void onStdOutReceived(Debug debug, final String message) {
            appendMessage(message, ConsoleType.OUTPUT);
        }//onStdOutReceived

        @Override
        public void onStdErrReceived(Debug debug, String message) {
            appendMessage(message, ConsoleType.ERROR);
        }//onStdErrReceived

        @Override
        public void onStdInSent(Debug debug, String message, boolean writeResult) {
            if (!writeResult )
                appendMessage(getResources().getString(R.string.ErrorSendMsg), ConsoleType.INPUT);
            appendMessage(message, ConsoleType.INPUT);

            //we finish to send a string -> send next message if present
            writeNextMessage();
        }//onStdInSent
    }//UpdateConsole

}

