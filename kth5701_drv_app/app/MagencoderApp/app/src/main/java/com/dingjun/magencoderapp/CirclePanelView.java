package com.dingjun.magencoderapp;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

public class CirclePanelView extends View {

    private Paint circlePaint; // 表盘画笔
    private Paint pointerPaint; // 指针画笔
    private int centerX, centerY; // 圆心坐标
    private float radius; // 表盘半径
    private double pointerAngle = 0; // 指针角度（单位：度）

    private float radiusOuter;
    private Paint circleOuterPaint; // 表盘画笔

    private float radiusBase;
    private Paint circleBase;

    public CirclePanelView(Context context) {
        super(context);
        init();
    }

    public CirclePanelView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public CirclePanelView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        circleBase = new Paint();
        circleBase.setColor(Color.RED);
        circleBase.setStyle(Paint.Style.FILL);
        circleBase.setStrokeWidth(1);
        circleBase.setAntiAlias(true);

        circleOuterPaint = new Paint();
        circleOuterPaint.setColor(Color.GRAY);
        circleOuterPaint.setStyle(Paint.Style.STROKE);
        circleOuterPaint.setStrokeWidth(2);
        circleOuterPaint.setAntiAlias(true);

        circlePaint = new Paint();
        circlePaint.setColor(Color.BLUE);
        circlePaint.setStyle(Paint.Style.STROKE);
        circlePaint.setStrokeWidth(3);
        circlePaint.setAntiAlias(true);

        pointerPaint = new Paint();
        pointerPaint.setColor(Color.RED);
        pointerPaint.setStyle(Paint.Style.STROKE);
        pointerPaint.setStrokeWidth(1);
        pointerPaint.setAntiAlias(true);
    }

    // 设置指针角度
    public void setPointerAngle(double angle) {
        this.pointerAngle = angle;
        invalidate(); // 重新绘制 View
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        centerX = w / 2;
        centerY = h / 2;
        radius = Math.min(centerX, centerY) - 30; // 确保表盘适配屏幕
        radiusOuter = radius + 5; // 确保表盘适配屏幕
        radiusBase = 8;

    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        //
        // 绘制表盘
        canvas.drawCircle(centerX, centerY, radius, circlePaint);
        // 绘制外圈
        canvas.drawCircle(centerX, centerY, radiusOuter, circleOuterPaint);
        //绘制指针基点
        canvas.drawCircle(centerX, centerY, radiusBase, circleBase);

        // 绘制指针
        float pointerLength = radius - 30; // 指针长度
        float angleInRadians = (float) Math.toRadians(pointerAngle - 90); // 转换为弧度
        float endX = (float) (centerX + pointerLength * Math.cos(angleInRadians));
        float endY = (float) (centerY + pointerLength * Math.sin(angleInRadians));
        //canvas.drawLine(centerX, centerY, endX, endY, pointerPaint);
    }
}







