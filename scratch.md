### I2C

I2C像一个TCP包一样，本身包含一些元信息

有两条线，一条数据，一条时钟

开头7位(?似乎不是8位)作为设备识别码，高四位是固定的，低三位自定义

[I2C的有效结构](https://i-blog.csdnimg.cn/blog_migrate/68129dad9315781f57b03ae7d1d1ffc8.png)

### SSD1306

作为从机

通信的时候发送地址 + Control Byte + Data Byte

地址：0111100或0111101

Control Byte: Co + D/C + 0 * 6

Co: 0

D/C: 0命令，1数据

HAL用 `HAL_I2C_Mem_Write`

`HAL_I2C_Master_Transmit`好像也可以