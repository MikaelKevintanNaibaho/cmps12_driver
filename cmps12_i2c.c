#include "cmps12_i2c.h"



int cmps12_open_i2c_device(const char *device)
{
    int file;

    //open i2c device
    if((file = open(device, O_RDWR)) < 0)
    {
        perror("failed to open the i2c device");
        return -1;
    }

    //set I2C_SLAVE_FORCE mode
    if(ioctl(file, I2C_SLAVE_FORCE, CMPS12_ADDRESS) < 0)
    {
        perror("failed to set I2C_SLAVE_FORCE mode");
        close(file);
        return -1;
    }

    return file;
}

int cmps12_release_i2c_device(int file)
{
    close(file);
}


int cmps12_read_register(int file, uint8_t register_addr, uint8_t* data, size_t data_size)
{
    if (file < 0 || data == NULL || data_size == 0)
    {
        return -1;
    }

    //send register address
    if (write(file, &register_addr, 1) != 1 )
    {
        return -2;
    }

    //start reading data
    if(ioctl(file, I2C_SLAVE_FORCE, CMPS12_ADDRESS) < 0 )
    {
        return -3;
    }

    //read data dari register
    if (read(file, data, data_size) != data_size)
    {
        return -4;
    }

    return 0;
}

int cmps12_write_register(int file, uint8_t register_addr, uint8_t data, size_t data_size)
{
    //check file valid atau tidak
    if (file < 0 || data == NULL || data_size == 0)
    {
        return -1;
    }
    
    //set slave address
    if(ioctl(file, I2C_SLAVE_FORCE, CMPS12_ADDRESS) < 0)
    {
        perror("ioctl I2C_SLAVE FORCE failed");
        return -2;
    }
    
    //buat buffer untuk address dan data
    uint8_t buffer[data_size + 1];
    buffer[0] = register_addr;
    buffer[1] = data;

    //write buffer to device
    if(write(file, buffer, sizeof(buffer)) != sizeof(buffer))
    {
        perror("write to register failed");
        return -3;
    }

    return 0;
}

int cmps12_init(const char* device)
{
    int file;
    file = cmps12_open_i2c_device(device);
    if(file < 0){
        return file;
    }

    return file;
}

uint8_t cmps12_read_bearing_8_bit(int cmps12_file)
{
    uint8_t bearing_8bit;

    if(cmps12_read_register(cmps12_file, COMPASS_BEARING_8_BIT_REG, &bearing_8bit, sizeof(bearing_8bit)) < 0){
        fprintf(stderr, "failed to read compas bearing 8 bit\n");
        return 0;
    }

    return bearing_8bit;
}

uint8_t cmps12_read_calibration_state(int cmps12_file)
{
    uint8_t calibration_state;

    if(cmps12_read_register(cmps12_file, CALIBRATION_REG, &calibration_state, sizeof(calibration_state)) < 0){
        fprintf(stderr, "failed to read calibration state 8 bit\n");
        return 0;
    }

    return calibration_state & 0x03;
}

uint16_t cmps12_read_bearing_16_bit_quaternion(int cmps12_file)
{
    uint8_t high_byte, low_byte;
    uint16_t bearing_16_bit;

    //read high byte
    if(cmps12_read_register(cmps12_file, COMPASS_BEARING_16_BIT_REG_HIGH_QUATERNION, &high_byte, sizeof(high_byte)) < 0){
        fprintf(stderr, "failed to read high byte of bearing(quaternion) \n");
        return 0;
    }

    //read low byte
    if(cmps12_read_register(cmps12_file, COMPASSS_BEARING_16_BIT_REG_LOW_QUATERNION, &low_byte, sizeof(low_byte)) < 0){
        fprintf(stderr, "failed to read low byte of bearing(quaternion) \n");
        return 0;
    }

    //combine high and low
    bearing_16_bit = (high_byte << 8) | low_byte;

    return bearing_16_bit;
}

uint8_t cmps12_read_pitch_90_degress(int cmps12_file)
{
    uint8_t pitch_90_degress;

    if(cmps12_read_register(cmps12_file, PITCH_ANGLE_REG_90_DEGRESS, &pitch_90_degress, sizeof(pitch_90_degress)) < 0){
        fprintf(stderr, "failed to read pitch 90 degress\n");
        return 0;
    }

    return pitch_90_degress;
}

uint8_t cmps12_read_roll_90_degress(int cmps12_file)
{
    uint8_t roll_90_degress;

    if(cmps12_read_register(cmps12_file, ROLL_ANGLE_REG_90_DEGRESS, &roll_90_degress, sizeof(roll_90_degress)) < 0){
        fprintf(stderr, "failde to read roll 90 degress\n");
        return 0;
    }

    return roll_90_degress;
}

//read the magnetometer X axis
uint16_t cmps12_read_mag_x_16_bit(int cmps12_file)
{
    uint8_t high_byte, low_byte;
    uint16_t mag_x_16_bit;

    //read the high byte
    if(cmps12_read_register(cmps12_file, MAG_X_RAW_16_BIT_REG_HIGH, &high_byte, sizeof(high_byte)) < 0){
        fprintf(stderr, "failed to read high byte of mag_x\n");
        return 0;
    }

    //read the low_byte
     if(cmps12_read_register(cmps12_file, MAG_X_RAW_16_BIT_REG_LOW, &low_byte, sizeof(low_byte)) < 0){
        fprintf(stderr, "failed to read low byte of mag_x \n");
        return 0;
    }

    mag_x_16_bit = (high_byte << 8) | low_byte;

    return mag_x_16_bit;
}

uint16_t cmps12_read_mag_y_16_bit(int cmps12_file)
{
    uint8_t high_byte, low_byte;
    uint16_t mag_y_16_bit;

    //read the high byte
    if(cmps12_read_register(cmps12_file, MAG_Y_RAW_16_BIT_REG_HIGH, &high_byte, sizeof(high_byte)) < 0){
        fprintf(stderr, "failed to read high byte of mag_y\n");
        return 0;
    }

    //read the low_byte
     if(cmps12_read_register(cmps12_file, MAG_Y_RAW_16_BIT_REG_LOW, &low_byte, sizeof(low_byte)) < 0){
        fprintf(stderr, "failed to read low byte of mag_y \n");
        return 0;
    }

    mag_y_16_bit = (high_byte << 8) | low_byte;

    return mag_y_16_bit;
}

uint16_t cmps12_read_mag_z_16_bit(int cmps12_file)
{
    uint8_t high_byte, low_byte;
    uint16_t mag_z_16_bit;

    //read the high byte
    if(cmps12_read_register(cmps12_file, MAG_Z_RAW_16_BIT_REG_HIGH, &high_byte, sizeof(high_byte)) < 0){
        fprintf(stderr, "failed to read high byte of mag_z\n");
        return 0;
    }

    //read the low_byte
     if(cmps12_read_register(cmps12_file, MAG_Z_RAW_16_BIT_REG_LOW, &low_byte, sizeof(low_byte)) < 0){
        fprintf(stderr, "failed to read low byte of mag_z \n");
        return 0;
    }

    mag_z_16_bit = (high_byte << 8) | low_byte;

    return mag_z_16_bit;
}

//read the accelerometer
uint16_t cmps12_read_accelerometer_x_16_bit(int cmps12_file)
{
    uint8_t high_byte, low_byte;
    uint16_t acc_x_16_bit;

    //read the high byte
    if(cmps12_read_register(cmps12_file, ACCELEROMETER_X_RAW_16_BIT_HIGH, &high_byte, sizeof(high_byte)) < 0){
        fprintf(stderr, "failed to read high byte of accelerometer x\n");
        return 0;
    }

    //read the low_byte
     if(cmps12_read_register(cmps12_file, ACCELEROMETER_X_RAW_16_BIT_LOW, &low_byte, sizeof(low_byte)) < 0){
        fprintf(stderr, "failed to read low byte of accelerometer x\n");
        return 0;
    }

    acc_x_16_bit = (high_byte << 8) | low_byte;

    return acc_x_16_bit;
}

uint16_t cmps12_read_accelerometer_x_16_bit(int cmps12_file)
{
    uint8_t high_byte, low_byte;
    uint16_t acc_y_16_bit;

    //read the high byte
    if(cmps12_read_register(cmps12_file, ACCELEROMETER_Y_RAW_16_BIT_HIGH, &high_byte, sizeof(high_byte)) < 0){
        fprintf(stderr, "failed to read high byte of accelerometer y\n");
        return 0;
    }

    //read the low_byte
     if(cmps12_read_register(cmps12_file, ACCELEROMETER_Y_RAW_16_BIT_LOW, &low_byte, sizeof(low_byte)) < 0){
        fprintf(stderr, "failed to read low byte of accelerometer y\n");
        return 0;
    }

    acc_y_16_bit = (high_byte << 8) | low_byte;

    return acc_y_16_bit;
}

uint16_t cmps12_read_accelerometer_z_16_bit(int cmps12_file)
{
    uint8_t high_byte, low_byte;
    uint16_t acc_z_16_bit;

    //read the high byte
    if(cmps12_read_register(cmps12_file, ACCELEROMETER_Z_RAW_16_BIT_HIGH, &high_byte, sizeof(high_byte)) < 0){
        fprintf(stderr, "failed to read high byte of accelerometer z\n");
        return 0;
    }

    //read the low_byte
     if(cmps12_read_register(cmps12_file, ACCELEROMETER_Z_RAW_16_BIT_LOW, &low_byte, sizeof(low_byte)) < 0){
        fprintf(stderr, "failed to read low byte of accelerometer z\n");
        return 0;
    }

    acc_z_16_bit = (high_byte << 8) | low_byte;

    return acc_z_16_bit;
}

uint16_t cmps12_read_gyro_x_16_bit(int cmps12_file)
{
    uint8_t high_byte, low_byte;
    uint16_t gyro_x_16_bit;

    //read the high byte
    if(cmps12_read_register(cmps12_file, GYRO_X_RAW_16_BIT_HIGH, &high_byte, sizeof(high_byte)) < 0){
        fprintf(stderr, "failed to read high byte of gyro x\n");
        return 0;
    }

    //read the low_byte
     if(cmps12_read_register(cmps12_file, GYRO_X_RAW_16_BIT_LOW, &low_byte, sizeof(low_byte)) < 0){
        fprintf(stderr, "failed to read low byte of gyro x\n");
        return 0;
    }

    gyro_x_16_bit = (high_byte << 8) | low_byte;

    return gyro_x_16_bit;
}

uint16_t cmps12_read_gyro_y_16_bit(int cmps12_file)
{
    uint8_t high_byte, low_byte;
    uint16_t gyro_y_16_bit;

    //read the high byte
    if(cmps12_read_register(cmps12_file, GYRO_Y_RAW_16_BIT_HIGH, &high_byte, sizeof(high_byte)) < 0){
        fprintf(stderr, "failed to read high byte of gyro y\n");
        return 0;
    }

    //read the low_byte
     if(cmps12_read_register(cmps12_file, GYRO_Y_RAW_16_BIT_LOW, &low_byte, sizeof(low_byte)) < 0){
        fprintf(stderr, "failed to read low byte of gyro y\n");
        return 0;
    }

    gyro_y_16_bit = (high_byte << 8) | low_byte;

    return gyro_y_16_bit;
}

uint16_t cmps12_read_gyro_z_16_bit(int cmps12_file)
{
    uint8_t high_byte, low_byte;
    uint16_t gyro_z_16_bit;

    //read the high byte
    if(cmps12_read_register(cmps12_file, GYRO_Z_RAW_16_BIT_HIGH, &high_byte, sizeof(high_byte)) < 0){
        fprintf(stderr, "failed to read high byte of gyro z\n");
        return 0;
    }

    //read the low_byte
     if(cmps12_read_register(cmps12_file, GYRO_Z_RAW_16_BIT_LOW, &low_byte, sizeof(low_byte)) < 0){
        fprintf(stderr, "failed to read low byte of gyro Z\n");
        return 0;
    }

    gyro_z_16_bit = (high_byte << 8) | low_byte;

    return gyro_z_16_bit;
}

uint16_t cmps12_temperature_16_bit(int cmps12_file)
{
    uint8_t high_byte, low_byte;
    uint16_t temp_16_bit;

    //read the high byte
    if(cmps12_read_register(cmps12_file, TEMP_REG_16_BIT_HIGH, &high_byte, sizeof(high_byte)) < 0){
        fprintf(stderr, "failed to read high byte of temperature\n");
        return 0;
    }

    //read the low_byte
     if(cmps12_read_register(cmps12_file, TEMP_REG_16_BIT_LOW, &low_byte, sizeof(low_byte)) < 0){
        fprintf(stderr, "failed to read low byte of temperature\n");
        return 0;
    }

    temp_16_bit = (high_byte << 8) | low_byte;

    return temp_16_bit;
}

//read bearing dari BNO055
uint16_t cmps12_read_bearing_16_bit_BNO055(int cmps12_file)
{
    uint8_t high_byte, low_byte;
    uint16_t bearing_16_bit;

    // Read the high byte of the 16-bit compass bearing
    if (cmps12_read_register(cmps12_file, COMPASS_BEARING_16_BIT_REG_HIGH_BNO055, &high_byte, sizeof(high_byte)) < 0) {
        fprintf(stderr, "Failed to read high byte of compass bearing\n");
        return 0; // Return 0 on error
    }

    // Read the low byte of the 16-bit compass bearing
    if (cmps12_read_register(cmps12_file, COMPASSS_BEARING_16_BIT_REG_LOW_BNO055, &low_byte, sizeof(low_byte)) < 0) {
        fprintf(stderr, "Failed to read low byte of compass bearing\n");
        return 0; // Return 0 on error
    }

    // Combine the high and low bytes to form the 16-bit compass bearing
    bearing_16_bit = (high_byte << 8) | low_byte;

    // Divide the 16-bit bearing by 16 to get the bearing in degrees
    return bearing_16_bit / 16;
}

//read pitch 180 degress
uint16_t cmps12_read_pitch_180_degress(int cmps12_file)
{
    uint8_t high_byte, low_byte;
    uint16_t pitch_180_degress;

    //read the high byte
    if(cmps12_read_register(cmps12_file, PITCH_ANGLE_16_BIT_REG_HIGH_180_DEGRESS, &high_byte, sizeof(high_byte)) < 0){
        fprintf(stderr, "failed to read high byte of pitch 180\n");
        return 0;
    }

    //read the low_byte
     if(cmps12_read_register(cmps12_file, PITCH_ANGLE_16_BIT_REG_LOW_180_DEGRESS, &low_byte, sizeof(low_byte)) < 0){
        fprintf(stderr, "failed to read low byte of pitch 180\n");
        return 0;
    }

    pitch_180_degress = (high_byte << 8) | low_byte;

    return pitch_180_degress; 
}

SensorData cmps12_read_magnetometer_data(int cmps12_file)
{
    SensorData mag_data;

    mag_data.x = cmps12_read_mag_x_16_bit(cmps12_file);
    mag_data.y = cmps12_read_mag_y_16_bit(cmps12_file);
    mag_data.z = cmps12_read_mag_z_16_bit(cmps12_file);

    return mag_data;
}

SensorData cmps12_read_accelerometer_data(int cmps12_file)
{
    SensorData acc_data;

    acc_data.x = cmps12_read_accelerometer_x_16_bit(cmps12_file);
    acc_data.y = cmps12_read_accelerometer_y_16_bit(cmps12_file);
    acc_data.z = cmps12_read_accelerometer_z_16_bit(cmps12_file);

    return acc_data;
}

SensorData cmps12_read_gyro_data(int cmps12_file)
{
    SensorData gyro_data;

    gyro_data.x = cmps12_read_gyro_x_16_bit(cmps12_file);
    gyro_data.y = cmps12_read_gyro_y_16_bit(cmps12_file);
    gyro_data.z = cmps12_read_gyro_z_16_bit(cmps12_file);

    return gyro_data;
}

Orientation cmps12_read_orientation_quaternion(int cmps12_file)
{
    Orientation orientation_data;

    orientation_data.bearing = cmps12_read_bearing_16_bit_quaternion(cmps12_file);
    orientation_data.pitch = cmps12_read_pitch_90_degress(cmps12_file);
    orientation_data.roll = cmps12_read_roll_90_degress(cmps12_file);

    return orientation_data;
}

Orientation cmps12_read_orientation_BNO055(int cmps12_file)
{
    Orientation orientation_data;

    orientation_data.bearing = cmps12_read_bearing_16_bit_BNO055(cmps12_file);
    orientation_data.pitch = cmps12_read_pitch_90_degress(cmps12_file);
    orientation_data.roll = cmps12_read_roll_90_degress(cmps12_file);

    return orientation_data;
}

//read all the sensor data
AllSensorData cmps12_read_all_data(int cmps12_file){
    AllSensorData all_data;

    all_data.magnetometer = cmps12_read_magnetometer_data(cmps12_file);
    all_data.accelerometer = cmps12_read_accelerometer_data(cmps12_file);
    all_data.gyro = cmps12_read_gyro_data(cmps12_file);
    all_data.orientation = cmps12_read_orientation_BNO055(cmps12_file);

    return all_data;
}