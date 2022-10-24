#include <stdio.h>

#include <pzem-driver.h>

#define TXPIN 17
#define RXPIN 16

#define ADDRESS 0x10


void app_main(void)
{
    initialize_pzem(TXPIN,RXPIN);

    updateValues(ADDRESS);
    printf("Voltage: %f\n", getVoltage());
    printf("Current: %f\n", getCurrent());
    printf("Power: %f\n", getPower());
    printf("Energy: %f\n", getEnergy());
    printf("Frequency: %f\n", getFrequency());
    printf("PF: %f\n", getPF());
    printf("Alarm: %d\n", getAlarm());


    }
