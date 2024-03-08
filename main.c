#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "cmps12_i2c.h"

int main(void){
	const char *device = "/dev/i2c-1";
	int cmps12_file = cmps12_init(device);

    if(cmps12_file < 0){
        fprintf(stderr, "Failed to initialize cmps12\n");

        return 1;
    }

    uint8_t calibration_status = cmps12_read_calibration_state(cmps12_file);
    if (calibration_status == 0x03) {
        printf("CMPS12 sensor fully calibrated\n");
    } else {
        printf("CMPS12 sensor calibration in progress\n");
    }
    while (1){
        AllSensorData all_data = cmps12_read_all_data(cmps12_file);

        // Print the data
        printf("Magnetometer: X=%u, Y=%u, Z=%u\n", all_data.magnetometer.x, all_data.magnetometer.y, all_data.magnetometer.z);
        printf("Accelerometer: X=%u, Y=%u, Z=%u\n", all_data.accelerometer.x, all_data.accelerometer.y, all_data.accelerometer.z);
        printf("Gyroscope: X=%u, Y=%u, Z=%u\n", all_data.gyro.x, all_data.gyro.y, all_data.gyro.z);
        printf("Orientation: Bearing=%u, Pitch=%u, Roll=%u\n", all_data.orientation.bearing, all_data.orientation.pitch, all_data.orientation.roll);

        usleep(10000); //delay 1 second

    }
    

    cmps12_release_i2c_device(cmps12_file);

    return 0;
}