# eCompassCalibration

dataformat:

  sprintf((char*)data, "RAW: %d,%d,%d,%d,%d,%d,%d,%d,%d\n\r", 0, 0, 0, 0, 0, 0, mag.XAxis, mag.YAxis, mag.ZAxis, 0 );
  Serial.print((char*)data);

# LabView Software

!()[./pict/main_1.png]
