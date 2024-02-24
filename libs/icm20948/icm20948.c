#include "icm20948.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>



static void twi_write_register_block(uint8_t addr, uint8_t reg, uint8_t *block, uint8_t count)
{
    uint8_t data[17];
    data[0] = reg;
    memcpy(&data[1], block, MIN(count, 16));
    //nrf_drv_twi_tx(&m_twi, addr, &data[0], count+1, false);
    
    int ret = i2c_write_dt(&dev_i2c,data,sizeof(data));
	if(ret != 0){
			printk("Failed to write to I2C err no.3 0x%x at Reg. 0x%x\n",dev_i2c.addr,reg);
		}

}

static void twi_write_register(uint8_t addr, uint8_t reg, uint8_t value)
{
   // twi_write_register_block(addr, reg, &value, 1);
	i2c_reg_write_byte_dt(&dev_i2c, reg, value);
}

int inv_icm20948_i2c_write_reg(uint8_t reg, uint8_t value)
{
    if (verbose)
    {
        printf("Executing write reg %s(0x%02x, 0x%02x)\r\n", __func__, reg, value);
    }

    twi_write_register(IMU_ADDR, reg, value);

    return 0;
}

int inv_icm20948_i2c_read_reg(uint8_t reg, uint8_t *value)
{
    if (verbose)
        printf("Executing read reg %s(0x%02x)\r\n", __func__, reg);

    *value = twi_read_register(IMU_ADDR, reg);

    if (verbose)
    {
        printf("0x%02x\r\n", *value);
    }

    return 0;
}

int inv_icm20948_i2c_read_reg_block(uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
    int i;

    if (verbose)
        printf("Executing read block %s(0x%02x, %d)\r\n", __func__, reg, rlen);

    twi_read_register_block(IMU_ADDR, reg, rbuffer, rlen);

    if (verbose)
    {
        for (i = 0; i < rlen; i++)
            printf(" val[%d]:0x%02x%s", i, rbuffer[i], (i%4==3?"\r\n":", "));
        printf("\r\n");
   }

    return 0;
}

int inv_icm20948_i2c_write_reg_block(uint8_t reg, uint8_t *wbuffer, uint32_t wlen)
{
    int i;

    if (verbose)
    {
        printf("Executing write block %s(0x%02x, 0x%02x, %u)", __func__, reg, wbuffer[0], wlen);
        if (wlen > 1)
        {
            printf("\r\n");
            for (i = 0; i < wlen; i++)
                printf(" val[%d]:0x%02x%s", i, wbuffer[i], (i%4==3?"\r\n":((i==(wlen-1)?"\r\n":","))));
        }
    }

    twi_write_register_block(IMU_ADDR, reg, wbuffer, wlen);

    return 0;
}

void inv_icm20948_write_register(uint16_t reg, uint8_t value)
{
    inv_icm20948_set_bank(reg);
    inv_icm20948_i2c_write_reg((uint8_t)(reg&0xff), value);
}

void inv_icm20948_write_register_block(uint16_t reg, uint8_t *block, uint8_t count)
{
    inv_icm20948_set_bank(reg);
    inv_icm20948_i2c_write_reg_block((uint8_t)(reg&0xff), block, count);
}

uint8_t inv_icm20948_read_register(uint16_t reg)
{
    uint8_t value;
    inv_icm20948_set_bank(reg);
	i2c_reg_read_byte_dt(&dev_i2c, (uint8_t)(reg&0xff), &value);
    //inv_icm20948_i2c_read_reg((uint8_t)(reg&0xff), &value);
    return value;
}

void inv_icm20948_read_register_block(uint16_t reg, uint8_t *block, uint8_t count)
{
    inv_icm20948_set_bank(reg);
    inv_icm20948_i2c_read_reg_block((uint8_t)(reg&0xff), block, count);
}

/***********************************************************************/
/* config icm                                                    */
/***********************************************************************/
void inv_icm20948_set_bank(uint16_t reg)
{
    uint8_t bank = (reg & 0xff00) >> 4;
    if (bank != current_bank) {
        i2c_reg_write_byte_dt(&dev_i2c,IMU_REG_BANK_SEL, bank);
        current_bank = bank;
    }
}
int16_t inv_icm20948_set_sleep_mode(bool sleep_mode)
{
    uint8_t temp, temp2;
	int ret;
    // get the sleep enable bit
    //temp = inv_icm20948_read_register(IMU_PWR_MGMT_1);
	inv_icm20948_set_bank(IMU_PWR_MGMT_1);
	i2c_reg_read_byte_dt(&dev_i2c,(uint8_t)(IMU_PWR_MGMT_1&0xff) , &temp);
    if (sleep_mode == false)
        temp &= ~(IMU_BIT_SLEEP);    // clear the sleep bit
    else
        temp |= IMU_BIT_SLEEP;       // set the sleep bit
   // inv_icm20948_write_register(IMU_PWR_MGMT_1, temp);
	inv_icm20948_set_bank(IMU_PWR_MGMT_1);
	ret = i2c_reg_write_byte_dt(&dev_i2c,(uint8_t)(IMU_PWR_MGMT_1&0xff),temp);

    //temp2 = inv_icm20948_read_register(IMU_PWR_MGMT_1);
	inv_icm20948_set_bank(IMU_PWR_MGMT_1);
	i2c_reg_read_byte_dt(&dev_i2c, (uint8_t)(IMU_PWR_MGMT_1&0xff), &temp2);
    if (temp != temp2)
    {
		printk("error set sleep mode %d, %d \n", temp, temp2);
        return -1;
    }else{
		printk("set up sleep mode ok \n");
	}

    return 0;
}
int16_t inv_icm20948_get_fifo_counter(void)
{
    uint8_t data_blk[2];
	inv_icm20948_set_bank(IMU_FIFO_COUNTH);
	int ret = i2c_burst_read_dt(&dev_i2c, (uint8_t)(IMU_FIFO_COUNTH&0xff), data_blk, 2);
	if(ret != 0){
		printk("Failed to get fifo counter 0x%x at Reg. 0x%x\n",dev_i2c.addr,IMU_FIFO_COUNTH);
	}   
   // inv_icm20948_read_register_block(IMU_FIFO_COUNTH, data_blk, 2);
    return ((data_blk[0] << 8) | data_blk[1]);
}
int16_t inv_icm20948_reset_fifo(inv_icm20948_state *st)
{
    uint8_t temp;

    // disable interrupts
    inv_icm20948_write_register(IMU_INT_ENABLE, 0x00);
    inv_icm20948_write_register(IMU_INT_ENABLE_1, 0x00);
    inv_icm20948_write_register(IMU_INT_ENABLE_2, 0x00);
    inv_icm20948_write_register(IMU_INT_ENABLE_3, 0x00);


    // disable the sensor output to FIFO
    inv_icm20948_write_register(IMU_FIFO_EN_1, 0x00);
    inv_icm20948_write_register(IMU_FIFO_EN_2, 0x00);


    // disable fifo reading
    inv_icm20948_write_register(IMU_USER_CTRL, 0x00);


    // reset FIFO
    inv_icm20948_write_register(IMU_FIFO_RST, 0x1F);
    inv_icm20948_write_register(IMU_FIFO_RST, 0x00);

    // enable interrupt
    if (   st->chip_config->accl_fifo_enable
        || st->chip_config->gyro_fifo_enable
        || st->chip_config->magn_fifo_enable
        || st->chip_config->temp_fifo_enable) {
        inv_icm20948_write_register(IMU_INT_ENABLE_1, IMU_BIT_RAW_DATA_0_RDY_EN);
    }
    //inv_icm20948_write_register(IMU_INT_ENABLE_2, 0x01);    // enable for testing

    // enable FIFO reading and I2C master interface
    inv_icm20948_write_register(IMU_USER_CTRL, IMU_BIT_FIFO_EN);

    // enable sensor output to FIFO
    temp = 0;
    if (st->chip_config->gyro_fifo_enable)
        temp |= IMU_BIT_GYRO_FIFO_EN;
    if (st->chip_config->accl_fifo_enable)
        temp |= IMU_BIT_ACCEL_FIFO_EN;
    if (st->chip_config->temp_fifo_enable)
        temp |= IMU_BIT_TEMP_FIFO_EN;
    inv_icm20948_write_register(IMU_FIFO_EN_2, temp);

    // need to add support for temperature and magnetometer

    return 0;
}
int16_t inv_icm20948_set_power(inv_icm20948_state *st, bool power_on)
{
    int result;
    if (st->chip_config->enable != power_on) {
        result = inv_icm20948_set_sleep_mode(power_on == true ? false : true);
        if (result)
            return result;
        st->chip_config->enable = power_on;
        //if (power_on)
        //    msleep(INV_ICM20948_POWER_UP_TIME);
    }
    return 0;
}
int16_t inv_icm20948_set_sample_frequency(uint16_t rate)
{
    uint8_t divider;
    divider = (uint8_t)(1100 / rate) - 1;    // from ICM-20948 data sheet
    inv_icm20948_write_register(IMU_GYRO_SMPLRT_DIV, divider);
    return 0;
}
int16_t inv_icm20948_set_gyro_dlpf(inv_icm20948_gyro_filter_e rate)
{
    uint8_t temp;
    //temp = inv_icm20948_read_register(IMU_GYRO_CONFIG_1);  // get current reg value
    //set bank
	inv_icm20948_set_bank(IMU_GYRO_CONFIG_1);
	i2c_reg_read_byte_dt(&dev_i2c,(uint8_t)(IMU_GYRO_CONFIG_1&0xff), &temp);
	temp &= 0xc6;                                          // clear all dlpf bit
    if (rate != INV_ICM20948_GYRO_FILTER_12106HZ_NOLPF) {
        temp |= 0x01;                                      // set FCHOICE bit
        temp |= (rate & 0x07) << 3;                        // set DLPFCFG bits
    }
	// set new value
	inv_icm20948_set_bank(IMU_GYRO_CONFIG_1);
	i2c_reg_write_byte_dt(&dev_i2c,(uint8_t)(IMU_GYRO_CONFIG_1&0xff),temp);   
   // inv_icm20948_write_register(IMU_GYRO_CONFIG_1, temp);        
    return 0;
}
int16_t inv_icm20948_set_accel_dlpf(inv_icm20948_accel_filter_e rate)
{
    uint8_t temp;
	//set bank
	inv_icm20948_set_bank(IMU_ACCEL_CONFIG);
	i2c_reg_read_byte_dt(&dev_i2c,(uint8_t)(IMU_ACCEL_CONFIG&0xff), &temp);
   // temp = inv_icm20948_read_register(IMU_ACCEL_CONFIG);            // get current reg value
    temp &= 0xc6;                                          // clear all dlpf bits
    if (rate != INV_ICM20948_ACCEL_FILTER_1209HZ_NOLPF) {
        temp |= 0x01;                                      // set FCHOICE bit
        temp |= (rate & 0x07) << 3;                        // set DLPFCFG bits
    }
	   // set new value
	inv_icm20948_set_bank(IMU_ACCEL_CONFIG);
	i2c_reg_write_byte_dt(&dev_i2c,(uint8_t)(IMU_ACCEL_CONFIG&0xff),temp); 
    //inv_icm20948_write_register(IMU_ACCEL_CONFIG, temp);         
    return 0;
}
void inv_icm20948_config_gyro(uint8_t full_scale_select)
{
    uint8_t temp;
    // set the gyro full scale range
	inv_icm20948_set_bank(IMU_GYRO_CONFIG_1);
	i2c_reg_read_byte_dt(&dev_i2c,(uint8_t)(IMU_GYRO_CONFIG_1&0xff), &temp);
   // temp = inv_icm20948_read_register(IMU_GYRO_CONFIG_1);   // get current value of register
    temp &= 0xf9;                                  // clear bit4 and bit3
    temp |= ((full_scale_select & 0x03) << 1);     // set desired bits to set range
    
	   // set new value
	inv_icm20948_set_bank(IMU_GYRO_CONFIG_1);
	i2c_reg_write_byte_dt(&dev_i2c,(uint8_t)(IMU_GYRO_CONFIG_1&0xff),temp); 
	//inv_icm20948_write_register(IMU_GYRO_CONFIG_1, temp);
    printk("Gyroscope FSR: %s \n", INV_ICM20948_GYRO_FSR_ASCII[(full_scale_select & 0x03)]);
}
void inv_icm20948_config_accel(uint8_t full_scale_select)
{
    uint8_t temp;
    // set the accelerometer full scale range
	inv_icm20948_set_bank(IMU_ACCEL_CONFIG);
	i2c_reg_read_byte_dt(&dev_i2c,(uint8_t)(IMU_ACCEL_CONFIG&0xff), &temp);	
    //temp = inv_icm20948_read_register(IMU_ACCEL_CONFIG);    // get current value of register
    temp &= 0xf9;                                  // clear bit2 and bit1
    temp |= ((full_scale_select & 0x03) << 1);     // set desired bits to set range
    inv_icm20948_set_bank(IMU_ACCEL_CONFIG);
	i2c_reg_write_byte_dt(&dev_i2c,(uint8_t)(IMU_ACCEL_CONFIG&0xff),temp); 
	//inv_icm20948_write_register(IMU_ACCEL_CONFIG, temp);
    printk("Accelerometer FSR: %s \n", INV_ICM20948_ACCEL_FSR_ASCII[(full_scale_select & 0x03)]);
}
uint8_t inv_icm20948_get_device_id(void)
{
    return (inv_icm20948_read_register(IMU_WHO_AM_I));
}
int16_t inv_check_and_setup_chip(inv_icm20948_state *st)
{
    int16_t result;

    result = inv_icm20948_init(st);
    if (result)
        return -1;

    result = inv_icm20948_set_power(st, true);
    if (result)
        return -1;

    if (   st->chip_config->accl_fifo_enable == true
        || st->chip_config->gyro_fifo_enable == true
        || st->chip_config->magn_fifo_enable == true
        || st->chip_config->temp_fifo_enable == true )
    {
        st->chip_config->bytes_per_datum = 0;
        if (st->chip_config->accl_fifo_enable == true)
            st->chip_config->bytes_per_datum += 6;
        if (st->chip_config->gyro_fifo_enable == true)
            st->chip_config->bytes_per_datum += 6;
        if (st->chip_config->magn_fifo_enable == true)
            st->chip_config->bytes_per_datum += 6;
        if (st->chip_config->temp_fifo_enable == true)
            st->chip_config->bytes_per_datum += 2;
        inv_icm20948_reset_fifo(st);
    }
    else
    {
        //inv_icm20948_write_register(IMU_INT_ENABLE_1, IMU_BIT_RAW_DATA_0_RDY_EN);
		inv_icm20948_set_bank(IMU_INT_ENABLE_1);
		i2c_reg_write_byte_dt(&dev_i2c,(uint8_t)(IMU_INT_ENABLE_1&0xff),IMU_BIT_RAW_DATA_0_RDY_EN);
    }

    return 0;
}
/***********************************************************************/
/* read imu                                           */
/***********************************************************************/
uint8_t _CalculateChecksum(uint8_t *buffer, int length)
{
  uint16_t sum = 0;
  int i;
  uint8_t kq;
  for (i = 0; i < length; i++)
    sum += buffer[i];
  kq= (65536 - sum) % 256;
  return kq; 
}
void inv_icm20948_read_imu_fifo(inv_icm20948_state *st, IMU_DATA *imu_data)
{

    uint16_t i, fifo_count, bytes_per_datum;

    const uint8_t numbytes = 14 + 6 + 1;//Read Accel, gyro, temp, and 6 bytes of mag + crc
    uint8_t buff[numbytes];
    fifo_count = inv_icm20948_get_fifo_counter();

    bytes_per_datum = st->chip_config->bytes_per_datum;
    if (fifo_count >= bytes_per_datum) {
        //inv_icm20948_read_register_block(IMU_FIFO_R_W, data_blk, bytes_per_datum);
        
        inv_icm20948_set_bank(IMU_FIFO_R_W);
        int ret = i2c_burst_read_dt(&dev_i2c,  (uint8_t)(IMU_ACCEL_XOUT_H&0xff) , buff,numbytes);
		if(ret != 0){
			printk("Failed to read fifo data 0x%x at Reg. 0x%x\n",dev_i2c.addr,IMU_ACCEL_XOUT_H);
		}   
        //imu_data->time_stamp = k_uptime_get_32();//inv_icm20948_get_time_us();
       fifo_count -= bytes_per_datum;
    }
    if (fifo_count) {   
        // reset FIFO
		 inv_icm20948_set_bank(IMU_FIFO_RST);
		 i2c_reg_write_byte_dt(&dev_i2c,(uint8_t)(IMU_FIFO_RST&0xff),0x1F);
        //inv_icm20948_write_register(IMU_FIFO_RST, 0x1F);

		 inv_icm20948_set_bank(IMU_FIFO_RST);
		 i2c_reg_write_byte_dt(&dev_i2c,(uint8_t)(IMU_FIFO_RST&0xff),0x00);
        //inv_icm20948_write_register(IMU_FIFO_RST, 0x00);
    }

    //pagmt->magStat1 = buff[14];

    //pagmt->magStat2 = buff[22];
    i = 0;
    if (st->chip_config->accl_fifo_enable) {
            imu_data->ax = ((buff[0] << 8) | (buff[1] & 0xFF));
            imu_data->ay = ((buff[2] << 8) | (buff[3] & 0xFF));
            imu_data->az = ((buff[4] << 8) | (buff[5] & 0xFF));
    }
    if (st->chip_config->gyro_fifo_enable) {
            imu_data->gx = ((buff[6] << 8) | (buff[7] & 0xFF));
            imu_data->gy = ((buff[8] << 8) | (buff[9] & 0xFF));
            imu_data->gz = ((buff[10] << 8) | (buff[11] & 0xFF));
        
    }
    if (st->chip_config->temp_fifo_enable) {
        imu_data->temperature = ((buff[12] << 8) | (buff[13] & 0xFF));
    
    }
    if (st->chip_config->magn_fifo_enable) {
        imu_data->mx = ((buff[15] << 8) | (buff[14] & 0xFF)); //Mag data is read little endian
        imu_data->my =((buff[17] << 8) | (buff[16] & 0xFF));
        imu_data->mz =  ((buff[19] << 8) | (buff[18] & 0xFF));
    }

    buff[20] = _CalculateChecksum(buff, 20);
    printk("crc %d--", buff[20]);
    for(int i = 0; i < 21; i++){
        printk("%x",buff[i]);
        printk("-");
    }
    printk("\n");
    // const uint8_t testSize = 5;
    uint8_t bu[5] = "SUP\n";
    uart_tx(uart, buff, sizeof(buff), SYS_FOREVER_MS);

    // printk("%x - %x | %x  - %x | %x  - %x  ||| ", buff[0],buff[1],buff[2],buff[3],buff[4],buff[5] );
    // printk("%x - %x | %x  - %x | %x  - %x ||| ", buff[6],buff[7],buff[8],buff[9],buff[10],buff[11] );
    // printk("%x - %x ||| %x  - %x | %x  - %x | ", buff[12],buff[13],buff[14],buff[15],buff[16],buff[17] );
    // printk("%x - %x\n ", buff[18],buff[19] );
    // printk("%x - %x - %x |", imu_data->ax, imu_data->ay, imu_data->az);
    // printk("%x - %x - %x |", imu_data->gx, imu_data->gy, imu_data->gz);
    // printk("%x - %x - %x |", imu_data->mx, imu_data->my, imu_data->mz);
    // printk(" %x  \n", imu_data->temperature);	
    // printf("ax %x ay %x az %x | ", imu_data->ax, imu_data->ay, imu_data->az);
    // printf("gx %x gy %x gz %x | ", imu_data->gx, imu_data->gy, imu_data->gz);
    // printf("mx %x my %x mz %x |", imu_data->mx, imu_data->my, imu_data->mz);
    //  printf("Tem %x \n ",imu_data->temperature);
    data_fifo_ready = true;
}
void inv_icm20948_read_imu(IMU_DATA *imu_data)
{
    // This function reads the axis data directly from the registers.
    const uint8_t numbytes = 14 + 9;//Read Accel, gyro, temp, and 9 bytes of mag
    uint8_t buff[numbytes];

    // burst read starts at register ACCEL_XOUT_H for 14 8 bit registers
	inv_icm20948_set_bank(IMU_ACCEL_XOUT_H);
	int errr = i2c_burst_read_dt(&dev_i2c,(uint8_t)(IMU_ACCEL_XOUT_H&0xff),buff,numbytes);
	
    //int ret = i2c_write_read(&dev_i2c,addr, reg,1, block,count );
	if(errr != 0){
		    printk("er %d -",errr);
			printk("Failed to read imu 0x%x at Reg. 0x%x\n",dev_i2c.addr,IMU_ACCEL_XOUT_H);
		} 
    //imu_data->time_stamp = k_uptime_get_32();// inv_icm20948_get_time_us();
    //imu_data->deviceid = (uint32_t)inv_icm20948_get_device_id();

    imu_data->ax = ((buff[0] << 8) | (buff[1] & 0xFF));
    imu_data->ay = ((buff[2] << 8) | (buff[3] & 0xFF));
    imu_data->az = ((buff[4] << 8) | (buff[5] & 0xFF));

    imu_data->gx = ((buff[6] << 8) | (buff[7] & 0xFF));
    imu_data->gy = ((buff[8] << 8) | (buff[9] & 0xFF));
    imu_data->gz = ((buff[10] << 8) | (buff[11] & 0xFF));

    imu_data->temperature = ((buff[12] << 8) | (buff[13] & 0xFF));
    //pagmt->magStat1 = buff[14];
    imu_data->mx = ((buff[16] << 8) | (buff[15] & 0xFF)); //Mag data is read little endian
    imu_data->my = ((buff[18] << 8) | (buff[17] & 0xFF));
    imu_data->mz = ((buff[20] << 8) | (buff[19] & 0xFF));
    //pagmt->magStat2 = buff[22];

    // printk("%x - %x - %x |", imu_data->ax, imu_data->ay, imu_data->az);
    // printk("%x - %x - %x |", imu_data->gx, imu_data->gy, imu_data->gz);
    // printk("%x - %x - %x |", imu_data->mx, imu_data->my, imu_data->mz);
    // printk(" %x  \n", imu_data->temperature);	
    // printk("ax %x ay %x az %x -", imu_data->ax, imu_data->ay, imu_data->az);
    // printk("gx %x gy %x gz %x -", imu_data->gx, imu_data->gy, imu_data->gz);
    // printk("mx %x my %x mz %x -", imu_data->mx, imu_data->my, imu_data->mz);
    // printk("tem %x  \n", imu_data->temperature);
}
void inv_icm20948_read_accel_xyz(int16_t *ax, int16_t *ay, int16_t *az)
{
  uint8_t  data_blk[6];
	inv_icm20948_set_bank(IMU_ACCEL_XOUT_H);
   // inv_icm20948_read_register_block(IMU_ACCEL_XOUT_H, data_blk, 6);
	i2c_burst_read_dt(&dev_i2c,(uint8_t)(IMU_ACCEL_XOUT_H&0xff),data_blk,6);
    *ax = (data_blk[0] << 8) + data_blk[1];
    *ay = (data_blk[2] << 8) + data_blk[3];
    *az = (data_blk[4] << 8) + data_blk[5];
}
void inv_icm20948_read_gyro_xyz(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t  data_blk[6];

   // inv_icm20948_read_register_block(IMU_GYRO_XOUT_H, data_blk, 6);
	inv_icm20948_set_bank(IMU_GYRO_XOUT_H);
	i2c_burst_read_dt(&dev_i2c,(uint8_t)(IMU_GYRO_XOUT_H&0xff),data_blk,6);
    *gx = (data_blk[0] << 8) + data_blk[1];
    *gy = (data_blk[2] << 8) + data_blk[3];
    *gz = (data_blk[4] << 8) + data_blk[5];
}
void inv_icm20948_read_magn_xyz(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t  data_blk[6];

   // inv_icm20948_read_register_block(IMU_GYRO_XOUT_H, data_blk, 6);
	inv_icm20948_set_bank(IMU_GYRO_XOUT_H);
	i2c_burst_read_dt(&dev_i2c,(uint8_t)(IMU_GYRO_XOUT_H&0xff),data_blk,6);
    *mx = (data_blk[0] << 8) + data_blk[1];
    *my = (data_blk[2] << 8) + data_blk[3];
    *mz = (data_blk[4] << 8) + data_blk[5];
}
void inv_icm20948_temperature(int16_t *temperature)
{
    uint8_t  data_blk[2];

   // inv_icm20948_read_register_block(IMU_TEMP_OUT_H, data_blk, 2);
	inv_icm20948_set_bank(IMU_TEMP_OUT_H);
	i2c_burst_read_dt(&dev_i2c,(uint8_t)(IMU_TEMP_OUT_H&0xff),data_blk,2);
    *temperature = (data_blk[0] << 8) + data_blk[1];
}

/***********************************************************************/
/* init functions  in main()                                           */
/***********************************************************************/
int16_t inv_icm20948_init(inv_icm20948_state *st)
{
     uint8_t result, counter, temp, imu_device_id;
	int ret;
    // let's start by resetting the device
    counter = 0;
	printk("wake up \n");
	inv_icm20948_set_bank(IMU_PWR_MGMT_1);
	i2c_reg_write_byte_dt(&dev_i2c,(uint8_t)(IMU_PWR_MGMT_1&0xff),IMU_BIT_DEVICE_RESET);
    //inv_icm20948_write_register(IMU_PWR_MGMT_1, IMU_BIT_DEVICE_RESET);
    do {
        k_sleep(K_USEC(MS_INTERVAL));
       // inv_icm20948_sleep_us(10); // 10uS delay
	   inv_icm20948_set_bank(IMU_PWR_MGMT_1);
        ret = i2c_reg_read_byte_dt(&dev_i2c, (uint8_t)(IMU_PWR_MGMT_1&0xff),&temp);
		 if (ret < 0) {
			printk("Failed to read chip ID.\n");		
		}
    } while ((temp & IMU_BIT_DEVICE_RESET) && (counter++ < 1000));

    // device requires 100mS delay after power-up/reset
    //inv_icm20948_sleep_us(100000);    // 100mS delay
    k_sleep(K_MSEC(HUNDRED_INTERVAL));
	printk("get id \n");
    imu_device_id = inv_icm20948_get_device_id();
    if (imu_device_id != IMU_EXPECTED_WHOAMI){
		printk("not found id %x \n",imu_device_id);
        return -1;
	}else{
		printk("found id %x\n", imu_device_id);
	}

    // clear the sleep enable bit
    result = inv_icm20948_set_sleep_mode(false);
    if (result)
        return result;

    // setup low pass filters 
    result = inv_icm20948_set_gyro_dlpf(st->chip_config->gyro_dlpf);
    result = inv_icm20948_set_accel_dlpf(st->chip_config->accel_dlpf);

    // setup sample rate
    result = inv_icm20948_set_sample_frequency(st->chip_config->sample_rate);
    if (result)
        return result;

    // set the clock source
    temp = inv_icm20948_read_register(IMU_PWR_MGMT_1);
    temp &= 0xf8;
    temp |= 0x01;
    inv_icm20948_write_register(IMU_PWR_MGMT_1, temp);

    // set the gyro full scale range
    inv_icm20948_config_gyro(st->chip_config->gyro_fsr);

    // set the accelerometer full scale range
    inv_icm20948_config_accel(st->chip_config->accl_fsr);

    // set the sleep enable bit
   //inv_icm20948_set_sleep_mode(true);

    return 0;
}
