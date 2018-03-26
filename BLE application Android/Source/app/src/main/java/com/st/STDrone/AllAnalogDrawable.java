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

import android.annotation.TargetApi;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.drawable.Drawable;
import android.graphics.drawable.ShapeDrawable;
import android.graphics.drawable.shapes.RectShape;
import android.graphics.drawable.shapes.OvalShape;
import android.support.annotation.Nullable;
import android.util.AttributeSet;
import android.view.View;

import static android.graphics.Color.argb;

/**
 * Created by ST on 7/10/2017.
 */

public class AllAnalogDrawable extends View {
    private ShapeDrawable mDrawableSquareL=new ShapeDrawable(new RectShape());
    private ShapeDrawable mDrawableSquareSmallL=new ShapeDrawable(new RectShape());
    private ShapeDrawable mDrawableCircleL=new ShapeDrawable(new OvalShape());
    private ShapeDrawable mDrawableCircleSmallL=new ShapeDrawable(new OvalShape());
    private ShapeDrawable mDrawableSquareR=new ShapeDrawable(new RectShape());
    private ShapeDrawable mDrawableSquareSmallR=new ShapeDrawable(new RectShape());
    private ShapeDrawable mDrawableCircleR=new ShapeDrawable(new OvalShape());
    private ShapeDrawable mDrawableCircleSmallR=new ShapeDrawable(new OvalShape());
    private ShapeDrawable mDrawableSquareBorderL=new ShapeDrawable(new RectShape());
    private ShapeDrawable mDrawableSquareSmallBorderL=new ShapeDrawable(new RectShape());
    private ShapeDrawable mDrawableCircleBorderL=new ShapeDrawable(new OvalShape());
    private ShapeDrawable mDrawableCircleSmallBorderL=new ShapeDrawable(new OvalShape());
    private ShapeDrawable mDrawableSquareBorderR=new ShapeDrawable(new RectShape());
    private ShapeDrawable mDrawableSquareSmallBorderR=new ShapeDrawable(new RectShape());
    private ShapeDrawable mDrawableCircleBorderR=new ShapeDrawable(new OvalShape());
    private ShapeDrawable mDrawableCircleSmallBorderR=new ShapeDrawable(new OvalShape());
    private static boolean mVisibleL = true;
    private static boolean mVisibleSmallL = true;
    private static boolean mVisibleR = true;
    private static boolean mVisibleSmallR = true;
    private static boolean mShapeCircle = true;
    private static float mDensityDpi;
    private final static boolean graphicsEnable = true;
    private final static float factor = 1.3f;
    private static Drawable mDrawableSmallL;
    private static Drawable mDrawableSmallR;
    private final static int PLANE_XY = 0;
    private final static int PLANE_XZ = 1;
    private final static int PLANE_ZY = 2;
    private static int mSwapCommandcode = PLANE_XZ; // default
    private static boolean mYawDisalble = false;
    private static Drawable mDrawableThrottleUP;
    private static Drawable mDrawableThrottleDown;
    private static Drawable mDrawableYawCW;
    private static Drawable mDrawableYawCCW;
    private static Drawable mDrawableRollCW;
    private static Drawable mDrawableRollCCW;
    private static Drawable mDrawablePitchCW;
    private static Drawable mDrawablePitchCCW;

    public AllAnalogDrawable(Context context) {
        super(context);
    }

    public AllAnalogDrawable(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
    }

    public AllAnalogDrawable(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
    }

    @TargetApi(21)
    public AllAnalogDrawable(Context context, @Nullable AttributeSet attrs, int defStyleAttr, int defStyleRes) {
        super(context, attrs, defStyleAttr, defStyleRes);
    }

    public void setYaw(boolean yawDisalble) {

        mYawDisalble = yawDisalble;
    }

    public void setPlanes(int swapCommandcode) {

        mSwapCommandcode = swapCommandcode;
    }

    public void setShape(boolean shapeCircle) {

        mShapeCircle = shapeCircle;
    }

    public void setDensity(float densityDpi) {

        mDensityDpi = densityDpi;
    }

    public void setBoundColorL(int x, int y, int height, int  width, int color) {

        int border = fromDp_to_px(5);

        if(mShapeCircle)
        {
             // If the color isn't set, the shape uses black as the default.
            mDrawableCircleBorderL.getPaint().setColor(argb(255,0x33,0xb5,0xe5));
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableCircleBorderL.setBounds(x-width, y-height, x + width, y + height);

            // If the color isn't set, the shape uses black as the default.
            mDrawableCircleL.getPaint().setColor(color);
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableCircleL.setBounds(x-width+border, y-height+border, x + width-border, y + height-border);
        }
        else
        {
            // If the color isn't set, the shape uses black as the default.
            mDrawableSquareBorderL.getPaint().setColor(argb(255,0x33,0xb5,0xe5));
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableSquareBorderL.setBounds(x-width, y-height, x + width, y + height);

            // If the color isn't set, the shape uses black as the default.
            mDrawableSquareL.getPaint().setColor(color);
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableSquareL.setBounds(x-width+border, y-height+border, x + width-border, y + height-border);
        }
        if(graphicsEnable)
        {
            int numSplits = 3;
            int halfLenghtSymbx = (width-border)/numSplits/2;
            int deltaCenterSymbx = (width-border)-halfLenghtSymbx;
            int halfLenghtSymby = (height-border)/numSplits/2;
            int deltaCenterSymby = (height-border)-halfLenghtSymby;
            if (!mYawDisalble) {
                mDrawableYawCW = getResources().getDrawable(R.drawable.software_drone_cursor_z1);
                mDrawableYawCCW = getResources().getDrawable(R.drawable.software_drone_cursor_z2);
                mDrawableYawCW.setBounds(x - deltaCenterSymbx - halfLenghtSymbx, y - halfLenghtSymby, x - deltaCenterSymbx + halfLenghtSymbx, y + halfLenghtSymby);
                mDrawableYawCCW.setBounds(x + deltaCenterSymbx - halfLenghtSymbx, y - halfLenghtSymby, x + deltaCenterSymbx + halfLenghtSymbx, y + halfLenghtSymby);
            }
            halfLenghtSymbx = halfLenghtSymby; // to be independent from x stretch of mYawDisalble
            switch (mSwapCommandcode) {
                case PLANE_XY:
                    mDrawablePitchCW = getResources().getDrawable(R.drawable.software_drone_cursor_x1);
                    mDrawablePitchCCW = getResources().getDrawable(R.drawable.software_drone_cursor_x2);
                    mDrawablePitchCW.setBounds(x-halfLenghtSymbx, y+deltaCenterSymby-halfLenghtSymby, x + halfLenghtSymbx, y+deltaCenterSymby + halfLenghtSymby);
                    mDrawablePitchCCW.setBounds(x-halfLenghtSymbx, y-deltaCenterSymby-halfLenghtSymby, x + halfLenghtSymbx, y-deltaCenterSymby + halfLenghtSymby);
                    break;
                case PLANE_XZ:
                    mDrawableThrottleUP = getResources().getDrawable(R.drawable.software_drone_cursor_up);
                    mDrawableThrottleDown = getResources().getDrawable(R.drawable.software_drone_cursor_down);
                    mDrawableThrottleDown.setBounds(x-halfLenghtSymbx, y+deltaCenterSymby-halfLenghtSymby, x + halfLenghtSymbx, y+deltaCenterSymby + halfLenghtSymby);
                    mDrawableThrottleUP.setBounds(x-halfLenghtSymbx, y-deltaCenterSymby-halfLenghtSymby, x + halfLenghtSymbx, y-deltaCenterSymby + halfLenghtSymby);
                    break;
                case PLANE_ZY:
                    mDrawableRollCW = getResources().getDrawable(R.drawable.software_drone_cursor_y1);
                    mDrawableRollCCW = getResources().getDrawable(R.drawable.software_drone_cursor_y2);
                    mDrawableRollCW.setBounds(x-halfLenghtSymbx, y+deltaCenterSymby-halfLenghtSymby, x + halfLenghtSymbx, y+deltaCenterSymby + halfLenghtSymby);
                    mDrawableRollCCW.setBounds(x-halfLenghtSymbx, y-deltaCenterSymby-halfLenghtSymby, x + halfLenghtSymbx, y-deltaCenterSymby + halfLenghtSymby);
                    break;
            }
        }
    }

    public void setBoundColorR(int x, int y, int height, int  width, int color) {

        int border = fromDp_to_px(5);

        if(mShapeCircle)
        {
            // If the color isn't set, the shape uses black as the default.
            mDrawableCircleBorderR.getPaint().setColor(argb(255,0x33,0xb5,0xe5));
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableCircleBorderR.setBounds(x-width, y-height, x + width, y + height);

            // If the color isn't set, the shape uses black as the default.
            mDrawableCircleR.getPaint().setColor(color);
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableCircleR.setBounds(x-width+border, y-height+border, x + width-border, y + height-border);
        }
        else
        {
            // If the color isn't set, the shape uses black as the default.
            mDrawableSquareBorderR.getPaint().setColor(argb(255,0x33,0xb5,0xe5));
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableSquareBorderR.setBounds(x-width, y-height, x + width, y + height);

            // If the color isn't set, the shape uses black as the default.
            mDrawableSquareR.getPaint().setColor(color);
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableSquareR.setBounds(x-width+border, y-height+border, x + width-border, y + height-border);
        }
        if(graphicsEnable)
        {
            int numSplits = 3;
            int halfLenghtSymbx = (width-border)/numSplits/2;
            int deltaCenterSymbx = (width-border)-halfLenghtSymbx;
            int halfLenghtSymby = (height-border)/numSplits/2;
            int deltaCenterSymby = (height-border)-halfLenghtSymby;
            switch (mSwapCommandcode) {
                case PLANE_XY:
                    mDrawableThrottleUP = getResources().getDrawable(R.drawable.software_drone_cursor_up);
                    mDrawableThrottleDown = getResources().getDrawable(R.drawable.software_drone_cursor_down);
                    mDrawableRollCW = getResources().getDrawable(R.drawable.software_drone_cursor_y1);
                    mDrawableRollCCW = getResources().getDrawable(R.drawable.software_drone_cursor_y2);
                    mDrawableRollCW.setBounds(x-deltaCenterSymbx-halfLenghtSymbx, y-halfLenghtSymby, x-deltaCenterSymbx + halfLenghtSymbx, y + halfLenghtSymby);
                    mDrawableRollCCW.setBounds(x+deltaCenterSymbx-halfLenghtSymbx, y-halfLenghtSymby, x+deltaCenterSymbx + halfLenghtSymbx, y + halfLenghtSymby);
                    mDrawableThrottleDown.setBounds(x-halfLenghtSymbx, y+deltaCenterSymby-halfLenghtSymby, x + halfLenghtSymbx, y+deltaCenterSymby + halfLenghtSymby);
                    mDrawableThrottleUP.setBounds(x-halfLenghtSymbx, y-deltaCenterSymby-halfLenghtSymby, x + halfLenghtSymbx, y-deltaCenterSymby + halfLenghtSymby);
                    break;
                case PLANE_XZ:
                    mDrawablePitchCW = getResources().getDrawable(R.drawable.software_drone_cursor_x1);
                    mDrawablePitchCCW = getResources().getDrawable(R.drawable.software_drone_cursor_x2);
                    mDrawableRollCW = getResources().getDrawable(R.drawable.software_drone_cursor_y1);
                    mDrawableRollCCW = getResources().getDrawable(R.drawable.software_drone_cursor_y2);
                    mDrawableRollCW.setBounds(x-deltaCenterSymbx-halfLenghtSymbx, y-halfLenghtSymby, x-deltaCenterSymbx + halfLenghtSymbx, y + halfLenghtSymby);
                    mDrawableRollCCW.setBounds(x+deltaCenterSymbx-halfLenghtSymbx, y-halfLenghtSymby, x+deltaCenterSymbx + halfLenghtSymbx, y + halfLenghtSymby);
                    mDrawablePitchCW.setBounds(x-halfLenghtSymbx, y+deltaCenterSymby-halfLenghtSymby, x + halfLenghtSymbx, y+deltaCenterSymby + halfLenghtSymby);
                    mDrawablePitchCCW.setBounds(x-halfLenghtSymbx, y-deltaCenterSymby-halfLenghtSymby, x + halfLenghtSymbx, y-deltaCenterSymby + halfLenghtSymby);
                    break;
                case PLANE_ZY:
                    mDrawableThrottleUP = getResources().getDrawable(R.drawable.software_drone_cursor_up);
                    mDrawableThrottleDown = getResources().getDrawable(R.drawable.software_drone_cursor_down);
                    mDrawablePitchCW = getResources().getDrawable(R.drawable.software_drone_cursor_x1);
                    mDrawablePitchCCW = getResources().getDrawable(R.drawable.software_drone_cursor_x2);
                    mDrawablePitchCW.setBounds(x-deltaCenterSymbx-halfLenghtSymbx, y-halfLenghtSymby, x-deltaCenterSymbx + halfLenghtSymbx, y + halfLenghtSymby);
                    mDrawablePitchCCW.setBounds(x+deltaCenterSymbx-halfLenghtSymbx, y-halfLenghtSymby, x+deltaCenterSymbx + halfLenghtSymbx, y + halfLenghtSymby);
                    mDrawableThrottleDown.setBounds(x-halfLenghtSymbx, y+deltaCenterSymby-halfLenghtSymby, x + halfLenghtSymbx, y+deltaCenterSymby + halfLenghtSymby);
                    mDrawableThrottleUP.setBounds(x-halfLenghtSymbx, y-deltaCenterSymby-halfLenghtSymby, x + halfLenghtSymbx, y-deltaCenterSymby + halfLenghtSymby);
                    break;
            }
        }
    }
    public void setBoundColorSmallL(int x, int y, int height, int  width, int color) {

        int border = fromDp_to_px(5);

        if(mShapeCircle)
        {
            // If the color isn't set, the shape uses black as the default.
            mDrawableCircleSmallBorderL.getPaint().setColor(argb(255,0x33,0xb5,0xe5));
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableCircleSmallBorderL.setBounds(x-width, y-height, x + width, y + height);

            if(graphicsEnable){
                mDrawableSmallL = getResources().getDrawable(R.drawable.software_drone_sphere);
                // If the bounds aren't set, the shape can't be drawn.
                int w = (int)((-width+border)*factor+0.5);
                int h = (int)((-height+border)*factor+0.5);
                mDrawableSmallL.setBounds(x+w, y+h, x-w, y-h);
            }else{
                // If the color isn't set, the shape uses black as the default.
                mDrawableCircleSmallL.getPaint().setColor(color);
                // If the bounds aren't set, the shape can't be drawn.
                mDrawableCircleSmallL.setBounds(x-width+border, y-height+border, x + width-border, y + height-border);
            }
        }
        else
        {
            // If the color isn't set, the shape uses black as the default.
            mDrawableSquareSmallBorderL.getPaint().setColor(argb(255,0x33,0xb5,0xe5));
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableSquareSmallBorderL.setBounds(x-width, y-height, x + width, y + height);

            // If the color isn't set, the shape uses black as the default.
            mDrawableSquareSmallL.getPaint().setColor(color);
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableSquareSmallL.setBounds(x-width+border, y-height+border, x + width-border, y + height-border);
        }
    }
    public void setBoundColorSmallR(int x, int y, int height, int  width, int color) {

        int border = fromDp_to_px(5);

        if(mShapeCircle)
        {
            // If the color isn't set, the shape uses black as the default.
            mDrawableCircleSmallBorderR.getPaint().setColor(argb(255,0x33,0xb5,0xe5));
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableCircleSmallBorderR.setBounds(x-width, y-height, x + width, y + height);

            if(graphicsEnable){
                mDrawableSmallR = getResources().getDrawable(R.drawable.software_drone_sphere);
                // If the bounds aren't set, the shape can't be drawn.
                int w = (int)((-width+border)*factor+0.5);
                int h = (int)((-height+border)*factor+0.5);
                mDrawableSmallR.setBounds(x+w, y+h, x-w, y-h);
            }else{
                // If the color isn't set, the shape uses black as the default.
                mDrawableCircleSmallR.getPaint().setColor(color);
                // If the bounds aren't set, the shape can't be drawn.
                mDrawableCircleSmallR.setBounds(x-width+border, y-height+border, x + width-border, y + height-border);
            }
        }
        else
        {
            // If the color isn't set, the shape uses black as the default.
            mDrawableSquareSmallBorderR.getPaint().setColor(argb(255,0x33,0xb5,0xe5));
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableSquareSmallBorderR.setBounds(x-width, y-height, x + width, y + height);

            // If the color isn't set, the shape uses black as the default.
            mDrawableSquareSmallR.getPaint().setColor(color);
            // If the bounds aren't set, the shape can't be drawn.
            mDrawableSquareSmallR.setBounds(x-width+border, y-height+border, x + width-border, y + height-border);
        }
    }

    public void setVisibleL(boolean visible) {

        mVisibleL = visible;
    }
    public void setVisibleSmallL(boolean visible) {

        mVisibleSmallL = visible;
    }
    public void setVisibleR(boolean visible) {

        mVisibleR = visible;
    }
    public void setVisibleSmallR(boolean visible) {

        mVisibleSmallR = visible;
    }

    public boolean getVisibleSmallL() {

        return mVisibleSmallL;
    }

    public boolean getVisibleSmallR() {

        return mVisibleSmallR;
    }


    @Override
    protected void onDraw(Canvas canvas) {
        if(mShapeCircle)
        {
            if(mVisibleL)
            {
                mDrawableCircleBorderL.draw(canvas);
                mDrawableCircleL.draw(canvas);
                if(graphicsEnable) {
                    if ((!mYawDisalble) && (mDrawableYawCW != null)) {
                        mDrawableYawCW.draw(canvas);
                        mDrawableYawCCW.draw(canvas);
                    }
                     switch (mSwapCommandcode) {
                        case PLANE_XY:
                            if(mDrawablePitchCW != null){
                                mDrawablePitchCW.draw(canvas);
                                mDrawablePitchCCW.draw(canvas);
                            }
                            break;
                        case PLANE_XZ:
                            if(mDrawableThrottleUP != null){
                                mDrawableThrottleUP.draw(canvas);
                                mDrawableThrottleDown.draw(canvas);
                            }
                            break;
                        case PLANE_ZY:
                            if(mDrawableRollCW != null){
                                mDrawableRollCW.draw(canvas);
                                mDrawableRollCCW.draw(canvas);
                            }
                            break;
                    }
                }
            }
            if(mVisibleSmallL)
            {
                mDrawableCircleSmallBorderL.draw(canvas);
                if(graphicsEnable && (mDrawableSmallL != null)) {
                    mDrawableSmallL.draw(canvas);
                }else{
                    mDrawableCircleSmallL.draw(canvas);
                }
            }
            if(mVisibleR)
            {
                mDrawableCircleBorderR.draw(canvas);
                mDrawableCircleR.draw(canvas);
                if(graphicsEnable) {
                    switch (mSwapCommandcode) {
                        case PLANE_XY:
                            if((mDrawableRollCW != null) && (mDrawableThrottleUP != null)) {
                                mDrawableRollCW.draw(canvas);
                                mDrawableRollCCW.draw(canvas);
                                mDrawableThrottleUP.draw(canvas);
                                mDrawableThrottleDown.draw(canvas);
                            }
                            break;
                        case PLANE_XZ:
                            if((mDrawableRollCW != null) && (mDrawablePitchCW != null)) {
                                mDrawableRollCW.draw(canvas);
                                mDrawableRollCCW.draw(canvas);
                                mDrawablePitchCW.draw(canvas);
                                mDrawablePitchCCW.draw(canvas);
                            }
                            break;
                        case PLANE_ZY:
                            if((mDrawablePitchCW != null) && (mDrawableThrottleUP != null)) {
                                mDrawablePitchCW.draw(canvas);
                                mDrawablePitchCCW.draw(canvas);
                                mDrawableThrottleUP.draw(canvas);
                                mDrawableThrottleDown.draw(canvas);
                            }
                            break;
                    }
                }
            }
            if(mVisibleSmallR)
            {
                mDrawableCircleSmallBorderR.draw(canvas);
                if(graphicsEnable && (mDrawableSmallR != null)) {
                    mDrawableSmallR.draw(canvas);
                }else{
                    mDrawableCircleSmallR.draw(canvas);
                }
            }
        }
        else
        {
            if(mVisibleL)
            {
                mDrawableSquareBorderL.draw(canvas);
                mDrawableSquareL.draw(canvas);
                if(graphicsEnable) {
                    if ((!mYawDisalble) && (mDrawableYawCW != null)) {
                        mDrawableYawCW.draw(canvas);
                        mDrawableYawCCW.draw(canvas);
                    }
                    switch (mSwapCommandcode) {
                        case PLANE_XY:
                            if(mDrawablePitchCW != null){
                                mDrawablePitchCW.draw(canvas);
                                mDrawablePitchCCW.draw(canvas);
                            }
                            break;
                        case PLANE_XZ:
                            if(mDrawableThrottleUP != null){
                                mDrawableThrottleUP.draw(canvas);
                                mDrawableThrottleDown.draw(canvas);
                            }
                            break;
                        case PLANE_ZY:
                            if(mDrawableRollCW != null){
                                mDrawableRollCW.draw(canvas);
                                mDrawableRollCCW.draw(canvas);
                            }
                            break;
                    }
                }
            }
            if(mVisibleSmallL)
            {
                mDrawableSquareSmallBorderL.draw(canvas);
                mDrawableSquareSmallL.draw(canvas);
            }
            if(mVisibleR)
            {
                mDrawableSquareBorderR.draw(canvas);
                mDrawableSquareR.draw(canvas);
                if(graphicsEnable) {
                    switch (mSwapCommandcode) {
                        case PLANE_XY:
                            if((mDrawableRollCW != null) && (mDrawableThrottleUP != null)) {
                                mDrawableRollCW.draw(canvas);
                                mDrawableRollCCW.draw(canvas);
                                mDrawableThrottleUP.draw(canvas);
                                mDrawableThrottleDown.draw(canvas);
                            }
                            break;
                        case PLANE_XZ:
                            if((mDrawableRollCW != null) && (mDrawablePitchCW != null)) {
                                mDrawableRollCW.draw(canvas);
                                mDrawableRollCCW.draw(canvas);
                                mDrawablePitchCW.draw(canvas);
                                mDrawablePitchCCW.draw(canvas);
                            }
                            break;
                        case PLANE_ZY:
                            if((mDrawablePitchCW != null) && (mDrawableThrottleUP != null)) {
                                mDrawablePitchCW.draw(canvas);
                                mDrawablePitchCCW.draw(canvas);
                                mDrawableThrottleUP.draw(canvas);
                                mDrawableThrottleDown.draw(canvas);
                            }
                            break;
                    }
                }
            }
            if(mVisibleSmallR)
            {
                mDrawableSquareSmallBorderR.draw(canvas);
                mDrawableSquareSmallR.draw(canvas);
            }
        }
    }

    private int fromDp_to_px(int dp) {
        return (int)(dp * (mDensityDpi / 160));
    }
}

