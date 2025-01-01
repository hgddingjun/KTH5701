package com.dingjun.magencoderapp;

import android.content.pm.PackageManager;
import android.os.Bundle;
import android.util.Log;

import androidx.activity.EdgeToEdge;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import android.Manifest;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {
    private static final int REQUEST_READ_STORAGE = 1;
    private final String TAG = "MagneticEncoderApp";
    //private  TextView resultView;
    //private String hexString;
    //private String result;

    //private String dataPart;

    private String [] hexValues;
    private byte [] bytes;
    private final String readPath = "/sys/devices/platform/11008000.i2c1/i2c-1/1-0068/kthReg";

    private double angle = 0.0F;
    private double angleOutput = 0.0F;

    private MAgRepeatingThread magThread;
    private boolean running = true; // 控制线程的运行状态

    private CirclePanelView circlePanelView;

    private CircularDialView circularDialView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_main);
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main), (v, insets) -> {
            Insets systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars());
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom);
            return insets;
        });


        Log.i(TAG,"onCreate");
        circlePanelView = findViewById(R.id.circlePanelView);

        circularDialView = findViewById(R.id.circularDialView);


        requestStoragePermission();

        // 在 Activity 中使用
        magThread = new MAgRepeatingThread();
        magThread.start();

    }

    private void requestStoragePermission() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.READ_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.READ_EXTERNAL_STORAGE}, REQUEST_READ_STORAGE);
        }
    }

    public class MAgRepeatingThread extends Thread {

        @Override
        public void run() {
            while (running) {
                try {
                    //执行任务
                    String hexString = readSystemFsNode(readPath);
                    String result = calculateRotationAngle(hexString);
                    //TextView resultView = findViewById(R.id.resultTextView);
                    //resultView.setText(result);
                    Log.i(TAG,"angleOutput:[ " + angleOutput + " ] Magnetic Thread is running...");

                    circlePanelView.post(new Runnable() {
                        @Override
                        public void run() {
                            //circlePanelView.setPointerAngle((int)angleOutput);

                            // 模拟旋转指针，设置角度
                            circularDialView.setPointerAngle((float)angleOutput);
                        }
                    });

                    Thread.sleep(100); // 任务间隔


                } catch (InterruptedException e) {
                    e.printStackTrace();
                    running = false; // 出现中断时退出
                }
            }
            Log.i(TAG,"Magnetic Thread has stopped.");
        }

        // 停止线程
        public void stopThread() {
            running = false;
        }

        public void resumeThread() {
            running = true;
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        Log.i(TAG,"onPause");
        //magThread.stopThread();
    }

    @Override
    protected void onResume() {
        super.onResume();
        Log.i(TAG, "onResume");
        //magThread.resumeThread();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Log.i(TAG,"onDestroy");
        magThread.stopThread();
    }

    private String calculateRotationAngle(String hexString) {

        //Log.d(TAG,"hexString = " + hexString);
        if (hexString == null || hexString.isEmpty()) {
            Log.e(TAG, "ERROR: Input hexString is null or empty!");
            return "Input hexString is null or empty";
        }

        String dataPart = hexString.substring(hexString.indexOf(":")+2);

        hexValues = dataPart.split(", ");
        bytes = new byte[hexValues.length];

        StringBuilder resultBuilder = new StringBuilder();
        resultBuilder.append("Parsed byte array: ");
        for(int i=0; i<hexValues.length; i++) {
            bytes[i] = (byte) Integer.parseInt(hexValues[i].substring(2), 16);
            resultBuilder.append(String.format("0x%02X ", bytes[i]));
        }

        angle = (float)(( bytes[3] << 8) + bytes[4]); //角度
        angleOutput =  angle * 360.0 /65536;

        return resultBuilder.toString();
    }

    private String readSystemFsNode(String path) {
        //Log.i(TAG,"readSystemFsNode read path: " + path);

        StringBuilder data = new StringBuilder();
        BufferedReader reader = null;

        try {
            reader = new BufferedReader(new FileReader(path));
            String line;
            while(null != (line = reader.readLine())) {
                Log.e(TAG,"line=" + line);
                data.append(line).append(""); //"\n"
            }
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        } finally {
            if(null == reader) {
                try {
                    reader.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }

        return data.toString();

    }
}