# PCA9555 Library
Platform independent library for the PCA9555 I/O expander. It allows you to configure, read and write to the I/O expander.

## Porting
1. Copy the PCA9555 folder to your project.
2. Add the .c and .h files to your project.
3. Add the .c file to the build process.
4. Edit the hal functions in pca9555_hal.c so that they suite your target platform. The hal folder contains an example implemnentation for the S32K144 microcontroller using either LPSPI or FlexI2C. It is enough to edit the functions `i2c_write`, `i2c_read` , `init_master_device` and `init_i2c` and `pcahal_delay_ms`. You can also edit the `__enter_critical` and `__exit_critical` if you need critical sections during read / write.
