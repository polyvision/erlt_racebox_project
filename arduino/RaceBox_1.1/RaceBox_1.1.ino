#include <Arduino.h>

/*
 * RST_TIMING
 * SLAVES_RSSI_STRENGTH
 * SLAVES_CRSSI_STRENGTH
 * SLAVE_CHANNELS
 * S_VTX_STR X Y
 * S_VTX_CH X Y
 * SLAVES_SET_MLT X
 * SLAVES_MLT
 * SLAVES_SS_GS
 * SLAVES_SS_GCO
 * SLAVES_SS_SCO X Y
 * LIPO_VOLTAGE
 * VERSION
 */
#include <Wire.h>
#include "util.h"
#include <EEPROM.h>


#define RB_VERSION "101"


int slave_channels[8];

// function declarations
void read_timing_data(int slave);
void process_cmd(String incomming);
void set_vtx_channel_for_slave(int slave,int channel);
void do_soft_reset(int slave);
void get_vtx_channel_for_slave(int slave);
void read_rssi_strength(int slave);
void read_minimum_lap_time(int slave);
void read_current_rssi_strength(int slave);
void read_smart_sense_strength(int slave);
void read_firmware_version(int slave);
void read_smart_sense_cut_off(int slave);
void set_minimum_lap_time(int slave,long mlp);
void set_smart_sense_cut_off(int slave,int x);
void set_vtx_trigger_strength(int slave,int rssi);
void invalidate_last_tracking(int slave);

void setup() {
  analogReference(INTERNAL);
  pinMode(A7, INPUT);
  delay(1000); // one second power up delay

  Serial.begin(115200);

  Wire.begin();  // initialize I2C-Bus
  //Wire.onReceive(i2c_receive);


  // setting the channel
  for(int i = 0; i < 8; i++){
    slave_channels[i] = EEPROMReadInt(2*i);
    set_vtx_channel_for_slave(i+1,slave_channels[i]);
  }

  Serial.println("RACEBOX READY");
}

void loop() {
  // put your main code here, to run repeatedly:

  for(int i = 1; i <= 8; i++){
    read_timing_data(i);
  }

  if(Serial.available()){
    String incomming = Serial.readString();
    process_cmd(incomming);
  }
}

// cmd: LIPO_VOLTAGE
void read_lipo_voltage(){
  float vin = 0.0;
  float vout = 0.0;
  float R1 = 100000.0; // resistance of R1 (100K) -see text!
  float R2 = 10000.0; // resistance of R2 (10K) - see text!
  int sensorValue = analogRead(A7);
  vout = (sensorValue * 1.1) / 1024.0; // see text
  vin = vout / (R2/(R1+R2));

  Serial.print("Analog read: ");
  Serial.println(sensorValue);
  // print out the converted (voltage) value
  Serial.print("VIN: ");
  Serial.println(vin);
  Serial.print("VOUT: ");
  Serial.println(vout);
}

void process_cmd(String incomming){
  if(incomming.indexOf("LIPO_VOLTAGE") == 0){
    read_lipo_voltage();
  }

  if(incomming.indexOf("RST_TIMING") == 0){
    for(int i = 1; i <= 8; i++){
      do_soft_reset(i);
    }
    Serial.println("SOFT_RESET_DONE");
  }

  if(incomming.indexOf("SLAVE_CHANNELS") == 0){
    for(int i = 1; i <= 8; i++){
      get_vtx_channel_for_slave(i);
    }
  }

  if(incomming.indexOf("SLAVES_RSSI_STRENGTH") == 0){
    for(int i = 1; i <= 8; i++){
      read_rssi_strength(i);
    }
  }

  if(incomming.indexOf("SLAVES_MLT") == 0){
    for(int i = 1; i <= 8; i++){
      read_minimum_lap_time(i);
    }
  }

  if(incomming.indexOf("SLAVES_CRSSI_STRENGTH") == 0){
    for(int i = 1; i <= 8; i++){
      read_current_rssi_strength(i);
    }
  }



  if(incomming.indexOf("SLAVES_SS_GS") == 0){
    for(int i = 1; i <= 8; i++){
      read_smart_sense_strength(i);
    }
  }

  if(incomming.indexOf("VERSION") == 0){
    Serial.print("MASTER_FV_VERSION ");
    Serial.println(RB_VERSION);

    for(int i = 1; i <= 8; i++){
      read_firmware_version(i);
    }
  }

  if(incomming.indexOf("SLAVES_SS_GCO") == 0){
    for(int i = 1; i <= 8; i++){
      read_smart_sense_cut_off(i);
    }
  }


  if(incomming.indexOf("SLAVES_SET_MLT") == 0){
    String mlp = getValue(incomming, ' ', 1);
    for(int i = 1; i <= 8; i++){
      set_minimum_lap_time(i,mlp.toInt());
    }
  }

  if(incomming.indexOf("SLAVES_SS_SCO") == 0){
    String slave = getValue(incomming, ' ', 1);
    String str = getValue(incomming, ' ', 2);
    set_smart_sense_cut_off(slave.toInt(),str.toInt());
  }

  if(incomming.indexOf("S_VTX_STR") == 0){
    String slave = getValue(incomming, ' ', 1);
    String str = getValue(incomming, ' ', 2);
    set_vtx_trigger_strength(slave.toInt(),str.toInt());
  }

  if(incomming.indexOf("S_VTX_CH") == 0){
    String slave = getValue(incomming, ' ', 1);
    String str = getValue(incomming, ' ', 2);
    set_vtx_channel_for_slave(slave.toInt(),str.toInt());
    get_vtx_channel_for_slave(slave.toInt());
  }

  if(incomming.indexOf("ITT") == 0){
    String slave = getValue(incomming, ' ', 1);
    invalidate_last_tracking(slave.toInt());
  }
}

void read_firmware_version(int slave){
  Wire.beginTransmission(slave);
  Wire.write("FWV#");
  Wire.endTransmission();; // ending I2C transmission

  delay(100);

  uint16_t fw_version = 0;
  byte byte_data[2];

  if(Wire.requestFrom(slave, 2) == 2){
    //I2C_readAnything(slave_rssi_strength);

    for(int i = 0; i < 2; i++){
        byte_data[i] = Wire.read();
        //byte_counter++;

        //Serial.print("BYTE: ");
        //Serial.println(byte_data[i]);
      }


    fw_version = byte_data[0];
    fw_version = (fw_version << 8) | byte_data[1];

    Serial.print("SLAVE_FV_VERSION ");
    Serial.print(slave);
    Serial.print(" ");
    Serial.println(fw_version);
  }
}

void read_rssi_strength(int slave){
  Wire.beginTransmission(slave);
  Wire.write("GRSSIS#");
  Wire.endTransmission();; // ending I2C transmission

  delay(100);

  uint16_t slave_rssi_strength = 0;
  byte byte_data[2];

  if(Wire.requestFrom(slave, 2) == 2){
    //I2C_readAnything(slave_rssi_strength);

    for(int i = 0; i < 2; i++){
        byte_data[i] = Wire.read();
        //byte_counter++;

        //Serial.print("BYTE: ");
        //Serial.println(byte_data[i]);
      }


    slave_rssi_strength = byte_data[0];
    slave_rssi_strength = (slave_rssi_strength << 8) | byte_data[1];

    Serial.print("GRSSIS ");
    Serial.print(slave);
    Serial.print(" ");
    Serial.println(slave_rssi_strength);
  }
}

void read_current_rssi_strength(int slave){
  Wire.beginTransmission(slave);
  Wire.write("CRSSIS#");
  Wire.endTransmission();; // ending I2C transmission

  delay(100);

  uint16_t slave_rssi_strength = 0;
  byte byte_data[2];

  if(Wire.requestFrom(slave, 2) == 2){
    //I2C_readAnything(slave_rssi_strength);

    for(int i = 0; i < 2; i++){
        byte_data[i] = Wire.read();
        //byte_counter++;

        //Serial.print("BYTE: ");
        //Serial.println(byte_data[i]);
      }


    slave_rssi_strength = byte_data[0];
    slave_rssi_strength = (slave_rssi_strength << 8) | byte_data[1];

    Serial.print("CRSSIS ");
    Serial.print(slave);
    Serial.print(" ");
    Serial.println(slave_rssi_strength);
  }
}


void read_minimum_lap_time(int slave){
  Wire.beginTransmission(slave);
  Wire.write("GMLT#");
  Wire.endTransmission();; // ending I2C transmission

  delay(100);

  uint16_t t_time = 0;
  byte byte_data[4];

  if(Wire.requestFrom(slave, 4) == 4){
    //I2C_readAnything(slave_rssi_strength);

    for(int i = 0; i < 4; i++){
        byte_data[i] = Wire.read();
        //byte_counter++;

        //Serial.print("BYTE: ");
        //Serial.println(byte_data[i]);
      }


    t_time = byte_data[0];
    t_time = (t_time << 8) | byte_data[1];
    t_time = (t_time << 8) | byte_data[2];
    t_time = (t_time << 8) | byte_data[3];

    Serial.print("GMLT ");
    Serial.print(slave);
    Serial.print(" ");
    Serial.println(t_time);
  }
}

void read_smart_sense_strength(int slave){
  Wire.beginTransmission(slave);
  Wire.write("GSSS#");
  Wire.endTransmission();; // ending I2C transmission

  delay(100);

  uint16_t t_time = 0;
  byte byte_data[4];

  if(Wire.requestFrom(slave, 2) == 2){
    //I2C_readAnything(slave_rssi_strength);

    for(int i = 0; i < 2; i++){
        byte_data[i] = Wire.read();
        //byte_counter++;

        //Serial.print("BYTE: ");
        //Serial.println(byte_data[i]);
      }


    t_time = byte_data[0];
    t_time = (t_time << 8) | byte_data[1];

    Serial.print("GSSS ");
    Serial.print(slave);
    Serial.print(" ");
    Serial.println(t_time);
  }
}

void read_smart_sense_cut_off(int slave){
  Wire.beginTransmission(slave);
  Wire.write("GSSCO#");
  Wire.endTransmission();; // ending I2C transmission

  delay(100);

  uint16_t t_time = 0;
  byte byte_data[4];

  if(Wire.requestFrom(slave, 2) == 2){
    //I2C_readAnything(slave_rssi_strength);

    for(int i = 0; i < 2; i++){
        byte_data[i] = Wire.read();
        //byte_counter++;

        //Serial.print("BYTE: ");
        //Serial.println(byte_data[i]);
      }


    t_time = byte_data[0];
    t_time = (t_time << 8) | byte_data[1];

    Serial.print("GSSCO ");
    Serial.print(slave);
    Serial.print(" ");
    Serial.println(t_time);
  }
}

void read_timing_data(int slave){
  //Serial.print("read_timing_data:");
  //Serial.println(slave);


  // TIMING AVAILABLE?
  Wire.beginTransmission(slave);
  Wire.write("TAV#");
  Wire.endTransmission();; // ending I2C transmission
  bool tav = false;
  delay(50);
  if(Wire.requestFrom(slave, 1) == 1){
    tav=(bool) Wire.read();

    //Serial.print("TAV:"); // just for debugging
    //Serial.println(tav);
  }else{
    return; // no valid response
  }


  // GET THE TIMING
  if(tav == true){
    Wire.beginTransmission(slave);
    Wire.write("LTT#");
    Wire.endTransmission();; // ending I2C transmission
    byte byte_data[4];

    delay(50);

    unsigned long t_time = 0;
    if(Wire.requestFrom(slave, 4) == 4){
      for(int i = 0; i < 4; i++){
        byte_data[i] = Wire.read();
        //byte_counter++;

        //Serial.print("BYTE: ");
        //Serial.println(byte_data[i]);
      }


      t_time = byte_data[0];
      t_time = (t_time << 8) | byte_data[1];
      t_time = (t_time << 8) | byte_data[2];
      t_time = (t_time << 8) | byte_data[3];
      Serial.print("TT_CH ");
      Serial.print(slave);
      Serial.print(" ");
      Serial.print(t_time);
      Serial.println("#");
    }
  }
}

void do_soft_reset(int slave){
  Wire.beginTransmission(slave);
  Wire.write("SRST#");
  Wire.endTransmission();    // stop transmitting

  delay(100);
}

void set_minimum_lap_time(int slave,long mlp){
  Wire.beginTransmission(slave);
  Wire.write("SMLT ");
  Wire.write(String(mlp).c_str());
  Wire.write("#");
  Wire.endTransmission();    // stop transmitting
  delay(100);
}

void set_vtx_channel_for_slave(int slave,int channel){
  slave_channels[slave-1] = channel;
  EEPROMWriteInt((slave-1)*2,channel); // saving the channel in the EEPROM
  Wire.beginTransmission(slave);
  Wire.write("SVTX ");
  Wire.write(String(channel).c_str());
  Wire.write("#");
  Wire.endTransmission();    // stop transmitting

  delay(100);
}

void get_vtx_channel_for_slave(int slave){
  Serial.print("CHANNEL ");
  Serial.print(slave);
  Serial.print(" ");
  Serial.println(slave_channels[slave-1]);
}

void set_smart_sense_cut_off(int slave,int x){
  Wire.beginTransmission(slave);
  Wire.write("SSSCO ");
  Wire.write(String(x).c_str());
  Wire.write("#");
  Wire.endTransmission();    // stop transmitting

  delay(100);
  Serial.print("SLAVES_SS_SCO ");
  Serial.print(slave);
  Serial.print(" ");
  Serial.print(x);
  Serial.print(" ");
  Serial.println("finished");
}

void set_vtx_trigger_strength(int slave,int rssi){
  Wire.beginTransmission(slave);
  Wire.write("SRSSIMS ");
  Wire.write(String(rssi).c_str());
  Wire.write("#");
  Wire.endTransmission();    // stop transmitting

  delay(100);
  Serial.print("S_VTX_STR ");
  Serial.print(slave);
  Serial.print(" ");
  Serial.print(rssi);
  Serial.print(" ");
  Serial.println("finished");
}

void invalidate_last_tracking(int slave){
  Wire.beginTransmission(slave);
  Wire.write("ILT#");
  Wire.endTransmission();    // stop transmitting

  delay(10);
  Serial.print("ILT ");
  Serial.print(slave);
  Serial.print(" ");
  Serial.println("finished");
}

void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}
