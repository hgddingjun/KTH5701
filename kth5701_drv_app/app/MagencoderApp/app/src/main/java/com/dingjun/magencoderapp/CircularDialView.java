package com.dingjun.magencoderapp;

import android.app.Notification;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.os.Build;
import android.util.AttributeSet;
import android.view.View;

public class CircularDialView extends View {
    private Paint paint;         // 用于刻度和数字的画笔
    private Paint pointerPaint; // 用于指针的画笔
    private int radius;         // 表盘半径
    private int centerX;        // 表盘中心X坐标
    private int centerY;        // 表盘中心Y坐标
    private float pointerAngle = 0; // 指针的角度

    private Path pointerPath;     // 画红色三角形指针
    private Paint degreePaint;    // 表盘中下方画实时度数
    private Paint titlePaint;     // 标题


    public CircularDialView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public CircularDialView(Context context) {
        super(context);
        init();
    }


    private void init() {
        paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        paint.setColor(Color.BLUE);
        
        paint.setStrokeWidth(1);
        paint.setTextSize(15);
        paint.setTextAlign(Paint.Align.CENTER);

        pointerPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        pointerPaint.setColor(Color.RED);
        pointerPaint.setStyle(Paint.Style.FILL);

        pointerPath = new Path();

        // 初始化文本画笔
        degreePaint = new Paint();
        degreePaint.setColor(Color.RED);
        degreePaint.setTextSize(20);
        degreePaint.setAntiAlias(true);
        degreePaint.setTextAlign(Paint.Align.CENTER);


        titlePaint = new Paint();
        titlePaint.setColor(Color.BLACK);
        titlePaint.setTextSize(15);
        titlePaint.setAntiAlias(true);
        titlePaint.setTextAlign(Paint.Align.CENTER);
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        radius = Math.min(w, h) / 2 - 35; // 确保表盘在视图中有边距
        centerX = w / 2;
        centerY = h / 2;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        drawDial(canvas);
        drawPointer(canvas);
    }


    private void drawDial(Canvas canvas) {
        for (int i = 0; i < 360; i++) {
            double angle = Math.toRadians(i);

            // 计算刻度线起点和终点
            float startX = (float) (centerX + (radius - getLineLength(i)) * Math.cos(angle));
            float startY = (float) (centerY + (radius - getLineLength(i)) * Math.sin(angle));
            float endX = (float) (centerX + radius * Math.cos(angle));
            float endY = (float) (centerY + radius * Math.sin(angle));

            // 设置刻度线的粗细
            paint.setStrokeWidth(getLineWidth(i));
            canvas.drawLine(startX, startY, endX, endY, paint);

            // 每隔30°画数字
            if (i % 30 == 0) {
                float textX = (float) (centerX + (radius - 30) * Math.cos(angle));
                float textY = (float) (centerY + (radius - 30) * Math.sin(angle)) + 5;
                canvas.drawText(String.valueOf(i), textX, textY, paint);
            }
        }
    }

    // 根据刻度角度返回刻度线的长度
    private int getLineLength(int degree) {
        if (degree % 30 == 0) {
            return 20; // 每30°刻度线长度
        } else if (degree % 10 == 0) {
            return 15; // 每10°刻度线长度
        } else {
            return 10; // 每1°刻度线长度
        }
    }

    // 根据刻度角度返回刻度线的宽度
    private int getLineWidth(int degree) {
        if (degree % 30 == 0) {
            return 1; // 每30°刻度线宽度
        } else if (degree % 10 == 0) {
            return 1; // 每10°刻度线宽度
        } else {
            return 1; // 每1°刻度线宽度
        }
    }


//    private void drawDial(Canvas canvas) {
//        for (int i = 0; i < 360; i += 10) {
//            double angle = Math.toRadians(i);
//            float startX = (float) (centerX + (radius - (i % 30 == 0 ? 40 : 20)) * Math.cos(angle));
//            float startY = (float) (centerY + (radius - (i % 30 == 0 ? 40 : 20)) * Math.sin(angle));
//            float endX = (float) (centerX + radius * Math.cos(angle));
//            float endY = (float) (centerY + radius * Math.sin(angle));
//            paint.setStrokeWidth(i % 30 == 0 ? 6 : 3);
//            canvas.drawLine(startX, startY, endX, endY, paint);
//
//            // 每隔30°画数字
//            if (i % 30 == 0) {
//                float textX = (float) (centerX + (radius - 60) * Math.cos(angle));
//                float textY = (float) (centerY + (radius - 60) * Math.sin(angle)) + 15;
//                canvas.drawText(String.valueOf(i), textX, textY, paint);
//            }
//        }
//    }


    private void drawPointer(Canvas canvas) {
        String degreeText;
        String titleText;
        double angle = Math.toRadians(pointerAngle);
        float pointerLength = radius - 20;
        float pointerWidth = 4;

        float endX = (float) (centerX + pointerLength * Math.cos(angle));
        float endY = (float) (centerY + pointerLength * Math.sin(angle));

        float baseX1 = (float) (centerX - pointerWidth * Math.sin(angle));
        float baseY1 = (float) (centerY + pointerWidth * Math.cos(angle));

        float baseX2 = (float) (centerX + pointerWidth * Math.sin(angle));
        float baseY2 = (float) (centerY - pointerWidth * Math.cos(angle));

        // 绘制指针
        pointerPaint.setColor(Color.RED);
        pointerPaint.setStyle(Paint.Style.FILL);
//        canvas.drawLine(baseX1, baseY1, endX, endY, pointerPaint);
//        canvas.drawLine(baseX2, baseY2, endX, endY, pointerPaint);
//        canvas.drawLine(baseX2, baseY2, baseX1, baseY1, pointerPaint);

        // 画红色三角形指针
        pointerPath.reset();
        pointerPath.moveTo(endX, endY);
        pointerPath.lineTo(baseX1, baseY1);
        pointerPath.lineTo(baseX2, baseY2);
        pointerPath.close();
        canvas.drawPath(pointerPath, pointerPaint);

        // 画度数
        // 在表盘的中下方绘制当前度数
        if(pointerAngle < 0.0f) {
            degreeText = String.format("%.1f°", 360.f + pointerAngle);
        } else {
            degreeText = String.format("%.1f°", pointerAngle);
        }
        canvas.drawText(degreeText, centerX, centerY + radius / 2, degreePaint);

        /* Magnetic Encoder */
        titleText = String.format("%s", "Magnetic Encoder");
        canvas.drawText(titleText, centerX, centerY - radius / 2 + 20, titlePaint);

    }

    public void setPointerAngle(float angle) {
        this.pointerAngle = angle; //% 360;
        invalidate(); // 触发重绘
    }
}
