#
# Makefile for industrial I/O accelerometer drivers
#

obj-$(CONFIG_HID_SENSOR_ACCEL_3D) += hid-sensor-accel-3d.o

obj-$(CONFIG_IIO_ST_ACCEL_3AXIS) += st_accel.o
st_accel-y := st_accel_core.o
st_accel-$(CONFIG_IIO_BUFFER) += st_accel_buffer.o

obj-$(CONFIG_IIO_ST_ACCEL_I2C_3AXIS) += st_accel_i2c.o
obj-$(CONFIG_IIO_ST_ACCEL_SPI_3AXIS) += st_accel_spi.o

obj-$(CONFIG_KXSD9)	+= kxsd9.o
obj-$(CONFIG_LIS2DG) += lis2dg_core.o lis2dg_buffer.o lis2dg_trigger.o lis2dg_i2c.o
obj-$(CONFIG_IIO_ST_LIS2DS12) += st_lis2ds12_core.o st_lis2ds12_buffer.o st_lis2ds12_trigger.o st_lis2ds12_i2c.o
