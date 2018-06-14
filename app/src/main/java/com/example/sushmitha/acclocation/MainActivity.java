package com.example.sushmitha.acclocation;

import android.annotation.SuppressLint;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

import android.Manifest;
import android.app.AlertDialog;
import android.app.Dialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Handler;
import android.provider.Settings;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.telephony.PhoneStateListener;
import android.telephony.SignalStrength;
import android.telephony.TelephonyManager;
import android.text.method.TextKeyListener;
import android.util.Log;
import android.widget.EditText;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import static com.example.sushmitha.acclocation.R.id.editText5;

public class MainActivity extends AppCompatActivity implements SensorEventListener, LocationListener {

    private final Object sync = new Object();
    Dialog alertDialog;
    SensorManager sensorManager;
    Sensor accelerometer, rotation, lightsensor;
    float[] gravitaionalValues = new float[3];
    float[] velocityValues = new float[3];
    private float time;
    float[] magnetOmeterValues = null;
    EditText latitude,longitude,latitude_acc,longitude_acc,dist;
    double gx = 0.0, gy = 0.0, gz = 0.0;
    Handler sensorHandler;
    static Double earthRadius = 6378D;
    static Double oldLat, oldLong;
    static Boolean IsFirst = true,night=true;
    private static String TAG="Checking";
    static Double sensorLatitude, sensorLongitude;
    int level,gps_count=0,network_count=0;
    static long GPSTime;
    public static Float currentAcceleration = 0.0F;
    public static Float currentDirection = 0.0F;
    public static Float CurrentSpeed = 0.0F;
    public static Float distance_Travelled = 0.0F;
    float[] prevValues;
    List<Float> accelorometerValuesx=new ArrayList<Float>();
    List<Float> accelorometerValuesy=new ArrayList<Float>();
    List<Float> accelorometerValuesz=new ArrayList<Float>();

    float prevTime, currentTime, changeTime, distanceX, distanceY, distanceZ;
    WifiManager wifi;
    Float lightLevel=0f;
    double distanceInMeters;

    LocationManager locationManager;
    public static Float prevAcceleration = 0.0F;
    public static Float prevSpeed = 0.0F;
    public static Float prevDistance = 0.0F;

    Handler locationHandler=new Handler();
    Boolean First, initilizeSensor = true;
    TelephonyManager meTelephonyManager;
    MyPhoneStateListener mePhoneStatelistener;
    int meSignalStrength = 0;
    Double latitudeOfMethod1, longitudeOfMethod1;
    boolean strengthFlag=false, lightFlag=false,isGPSEnabled=false,isNetworkEnabled=false;

    public MainActivity() {
        sensorHandler = new Handler();
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        First = initilizeSensor = true;
        prevValues = new float[3];
        latitude=(EditText)findViewById(R.id.editText);
        longitude=(EditText)findViewById(R.id.editText2);
        latitude_acc=(EditText)findViewById(R.id.editText3);
        longitude_acc=(EditText)findViewById(R.id.editText4);
        dist=(EditText)findViewById(R.id.editText5);
        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        assert sensorManager != null;
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        locationManager = (LocationManager) getSystemService(LOCATION_SERVICE);
        rotation = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        mePhoneStatelistener = new MyPhoneStateListener();
        meTelephonyManager = (TelephonyManager) getSystemService(TELEPHONY_SERVICE);
        wifi = (WifiManager)getSystemService(WIFI_SERVICE);
        lightsensor=sensorManager.getDefaultSensor(Sensor.TYPE_LIGHT);
        sensorManager.registerListener(this,lightsensor,SensorManager.SENSOR_DELAY_FASTEST);
        meTelephonyManager.listen(mePhoneStatelistener, PhoneStateListener.LISTEN_SIGNAL_STRENGTHS);
        isGPSEnabled =locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER);
        // getting network status
        isNetworkEnabled = locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER);

    }

    @Override
    protected void onResume() {
        super.onResume();

        int numberOfLevels = 5;
        WifiInfo wifiInfo = wifi.getConnectionInfo();
        level = WifiManager.calculateSignalLevel(wifiInfo.getRssi(), numberOfLevels);
        Log.i(TAG,"LEVEL"+level);
        if(!wifi.isWifiEnabled()){
            Log.i(TAG,"onCreate : WIFI DISABLED");
        }
        else{

            if (level<-90){
                //outdoors
                synchronized (sync) {
                    gps_count += 1;
                    if (network_count > 0) {
                        network_count--;
                    }
                }

            }
            else{
                synchronized (sync) {
                    network_count++;
                    if (gps_count > 0) {
                        gps_count--;
                    }
                }
            }
        }
    }
    class MyPhoneStateListener extends PhoneStateListener {

        @Override
        public void onSignalStrengthsChanged(SignalStrength signalStrength) {
            strengthFlag=true;
            super.onSignalStrengthsChanged(signalStrength);
            meSignalStrength = signalStrength.getGsmSignalStrength();
            meSignalStrength = (2 * meSignalStrength) - 113; //it will be in  -> dBm
//outdoor
            if (meSignalStrength>-90){
                synchronized (sync) {
                    gps_count += 1;
                    if (network_count > 0) {
                        network_count--;
                    }
                }

            }
            else{
                synchronized (sync) {
                    if (network_count <= 5) {
                        network_count += 1;
                    } else if (gps_count > 0) {
                        gps_count--;
                    }
                }

            }
        }
    }

    @SuppressLint("SetTextI18n")
    public void calculateDistance_m1_m2(double latitude1, double long1, double lat2, double lng2) {
        distanceInMeters=0;
        double dLat = Math.toRadians(lat2 - latitude1);
        double dLon = Math.toRadians(lng2 - long1);
        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2)
                + Math.cos(Math.toRadians(latitude1))
                * Math.cos(Math.toRadians(lat2)) * Math.sin(dLon / 2)
                * Math.sin(dLon / 2);
        double c = 2 * Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
        distanceInMeters = Math.round(earthRadius * c*1000);
        dist.setText(Double.toString(distanceInMeters) + " m");
    }

    public class Method1 implements Runnable{
        @SuppressLint("SetTextI18n")
        @Override
        public void run() {
            if (latitude.length() > 0) {
                TextKeyListener.clear(latitude.getText());
            }
            latitude.setText(latitudeOfMethod1.toString());
            if (longitude.length() > 0) {
                TextKeyListener.clear(longitude.getText());
            }
            longitude.setText(longitudeOfMethod1.toString());
        }
    }

    @Override
    public void onLocationChanged(Location location) {
        //TAKING VALUES FROM GPS/NETWORK FOR THE FIRST TIME
        if(First) {
            oldLat = location.getLatitude();
            oldLong = location.getLongitude();
            GPSTime = location.getTime();
            sensorManager.registerListener(this,rotation,SensorManager.SENSOR_DELAY_FASTEST);
            sensorManager.registerListener(this,accelerometer,SensorManager.SENSOR_DELAY_FASTEST);
        }
        latitudeOfMethod1=location.getLatitude();
        longitudeOfMethod1=location.getLongitude();
        locationHandler.post(new Method1());
    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {

    }

    @Override
    public void onProviderEnabled(String provider) {
        alertDialog.dismiss();
    }

    @Override
    public void onProviderDisabled(String provider) {
        Toast.makeText(getApplicationContext(), "Location services off. Turn on Location", Toast.LENGTH_LONG).show();
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("Location Services Not Active");
        builder.setMessage("Please enable Location Services and GPS");
        builder.setPositiveButton("OK", new DialogInterface.OnClickListener() {
            public void onClick(DialogInterface dialogInterface, int i) {
                // Show location settings when the user acknowledges the alert dialog
                Intent intent = new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS);
                startActivity(intent);
            }
        });
        alertDialog = builder.create();
        alertDialog.setCanceledOnTouchOutside(false);
        alertDialog.show();

    }

    public class Method2 implements Runnable{

        @Override
        public void run() {
            gx=(float)0.98*gx+(float)0.02*gravitaionalValues[0];
            gy=(float)0.98*gy+(float)0.02*gravitaionalValues[1];
            gz=(float)0.98*gz+(float)0.02*gravitaionalValues[2];
            gravitaionalValues[0]-=gx;
            gravitaionalValues[1]-=gy;
            gravitaionalValues[2]-=gz;

            //this is the time when the data is taken from the gps/network provider
            if(First){
                prevValues = gravitaionalValues;
                accelorometerValuesx.add(gravitaionalValues[0]);
                accelorometerValuesy.add(gravitaionalValues[1]);
                accelorometerValuesz.add(gravitaionalValues[2]);
                prevTime = time / 1000000000;
                First = false;
                distanceX = distanceY= distanceZ = 0;
            }
            else{

                currentTime = time / 1000000000.0f;
                changeTime = currentTime - prevTime;
                prevTime = currentTime;
                accelorometerValuesx.add(gravitaionalValues[0]);
                accelorometerValuesy.add(gravitaionalValues[1]);
                accelorometerValuesz.add(gravitaionalValues[2]);

                if(accelorometerValuesx.size()>49){
                    float avgx=0,avgy=0,avgz=0;
                    for (int i=0;i<50;i++){
                        avgx+=accelorometerValuesx.get(i);
                        avgy+=accelorometerValuesy.get(i);
                        avgz+=accelorometerValuesz.get(i);
                    }
                    accelorometerValuesx.clear();
                    accelorometerValuesy.clear();
                    accelorometerValuesz.clear();
                    gravitaionalValues[0]=avgx/50;
                    gravitaionalValues[1]=avgy/50;
                    gravitaionalValues[2]=avgz/50;

                    calculateDistance(gravitaionalValues, changeTime);
                    currentAcceleration =  (float) Math.sqrt(gravitaionalValues[0] * gravitaionalValues[0] + gravitaionalValues[1] * gravitaionalValues[1] + gravitaionalValues[2] * gravitaionalValues[2]);
                    CurrentSpeed = (float) Math.sqrt(velocityValues[0] * velocityValues[0] + velocityValues[1] * velocityValues[1] + velocityValues[2] * velocityValues[2]);
                    distance_Travelled = (float) Math.sqrt(distanceX *  distanceX + distanceY * distanceY +  distanceZ * distanceZ);
                    distance_Travelled = distance_Travelled / 1000;
                }


                if(initilizeSensor){
                    prevAcceleration = currentAcceleration;
                    prevDistance = distance_Travelled;
                    prevSpeed = CurrentSpeed;
                    initilizeSensor = false;
                }
                prevValues = gravitaionalValues;

            }
            if(currentAcceleration != prevAcceleration || CurrentSpeed != prevSpeed || prevDistance != distance_Travelled){

                if (gravitaionalValues != null && magnetOmeterValues != null && currentAcceleration != null) {
                    float RT[] = new float[9];
                    float I[] = new float[9];
                    boolean success = SensorManager.getRotationMatrix(RT, I, gravitaionalValues,
                            magnetOmeterValues);
                    if (success) {
                        float orientation[] = new float[3];
                        SensorManager.getOrientation(RT, orientation);
                        float azimut = (float) Math.round(Math.toDegrees(orientation[0]));
                        currentDirection =(azimut+ 360) % 360;
                        calculateGPS_Coordinates(distance_Travelled,currentDirection);
                    }
                    prevAcceleration = currentAcceleration;
                    prevSpeed = CurrentSpeed;
                    prevDistance = distance_Travelled;
                }
            }
        }
        public void calculateDistance(float[] values,float dT){
            float[] distance = new float[values.length];
            for (int i = 0; i < values.length; i++) {
                velocityValues[i] = values[i] * dT;
                distance[i] = velocityValues[i] * dT + values[i] * dT * dT / 2;
            }
            distanceX = distance[0];
            distanceY = distance[1];
            distanceZ = distance[2];
        }

        @SuppressLint("SetTextI18n")
        public void calculateGPS_Coordinates(Float distance_Travelled, Float currentDirection){
            Log.i(TAG,"calculateGPS_Coordinates");

            //when gps/network provider is being used
            if(IsFirst){
                sensorLatitude = oldLat;
                sensorLongitude = oldLong;
                IsFirst  = false;
                return;
            }

            Date CurrentTime = new Date();

            if(CurrentTime.getTime() - GPSTime > 0) {
                //Convert Variables to Radian for the Formula
                oldLat = Math.PI * oldLat / 180;
                oldLong = Math.PI * oldLong / 180;
                currentDirection = (float) (Math.PI * currentDirection / 180.0);

                //Formulae to Calculate the NewLAtitude and NewLongtiude
                Double newLatitude = Math.asin(Math.sin(oldLat) * Math.cos(distance_Travelled / earthRadius) +
                        Math.cos(oldLat) * Math.sin(distance_Travelled / earthRadius) * Math.cos(currentDirection));
                Double newLongitude = oldLong + Math.atan2(Math.sin(currentDirection) * Math.sin(distance_Travelled / earthRadius)
                        * Math.cos(oldLat), Math.cos(distance_Travelled / earthRadius)
                        - Math.sin(oldLat) * Math.sin(newLatitude));
                //Convert Back from radians
                newLatitude = 180 * newLatitude / Math.PI;
                newLongitude = 180 * newLongitude / Math.PI;
                currentDirection = (float) (180 * currentDirection / Math.PI);
                latitude_acc.setText(newLatitude.toString());
                longitude_acc.setText(newLongitude.toString());

                //Update old Latitude and Longitude
                oldLat = newLatitude;
                oldLong = newLongitude;

                sensorLatitude = oldLat;
                sensorLongitude = oldLong;
                calculateDistance_m1_m2(latitudeOfMethod1, longitudeOfMethod1, sensorLatitude, sensorLongitude);

            }
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {

            gravitaionalValues=event.values;
            time=event.timestamp;
            sensorHandler.post(new Method2());
        }
        if(event.sensor.getType()==Sensor.TYPE_LIGHT){
            lightFlag=true;
            lightLevel=event.values[0];
            //activate light sensor
            if(lightLevel>3000){
                //this means it is daylight and we are outdoor so gps should be enabled
                gps_count+=1;
                if (network_count > 0) {
                    network_count--;
                }
            }
            else if (lightLevel<5 && night){
                //outdoor
                synchronized (sync) {
                    gps_count += 1;
                    if (network_count > 0) {
                        network_count--;
                    }
                }

            }
            else{
                synchronized (sync) {
                    network_count += 1;
                    if (gps_count > 0) {
                        gps_count--;
                    }
                }

            }
            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
                return;
            }

            if (gps_count > network_count) {
                locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, this);//error here
                //Toast.makeText(getApplicationContext(), "GPS Selected", Toast.LENGTH_LONG).show();
                Log.i(TAG, "gps selected");
                gps_count = 0;
                network_count = 0;
            } else {
                locationManager.requestLocationUpdates(LocationManager.NETWORK_PROVIDER, 0, 0, this);
                // Toast.makeText(getApplicationContext(), "Network Selected", Toast.LENGTH_LONG).show();
                Log.i(TAG, "nw selected");
                gps_count = 0;
                network_count = 0;
            }
        }

        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            magnetOmeterValues = event.values;
        }

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
    @Override
    protected void onPause() {
        super.onPause();
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            return;
        }
        locationManager.removeUpdates(this);
        sensorManager.unregisterListener(this);
    }
    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            return;
        }
        locationManager.removeUpdates(this);
        sensorManager.unregisterListener(this);
    }

}
