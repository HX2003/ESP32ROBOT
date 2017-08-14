#include <stdio.h>
#include "driver/i2c.h"
#define I2C_EXAMPLE_MASTER_SCL_IO    25    /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO    26    /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */

void i2c_master_init();
void scani2c();
