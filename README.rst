.. code-block:: xml
    
    <node pkg="ntrip_ros" type="ntripclient.py" name="ntrip_client" output="screen">
        <param name="ntrip_server" value="rtk2go.com:2101" />
        <param name="ntrip_user" value="username" />
        <param name="ntrip_pass" value="password" />
        <param name="ntrip_stream" value="BB-UM-SB" />
        <param name="nmea_gga" value="$GPGGA,%02d%02d%04.2f,3213.06516704,N,599.62092864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F" />
        <param name="rtcm_topic" value="/mavros/gps_rtk/send_rtcm" />
    </node>
