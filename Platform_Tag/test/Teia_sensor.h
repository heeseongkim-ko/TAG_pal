#ifndef TEIA_SENSOR_H_INCLUDED
#define TEIA_SENSOR_H_INCLUDED

void sens_check_and_standby(void);
void sensors_config(void);
void configure_sensors(void);
void read_sensors_data(void);
bool check_NFC_EEPROM(void);

#endif  /* TEIA_SENSOR_H_INCLUDED */