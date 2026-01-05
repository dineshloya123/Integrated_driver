#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/bitops.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include<linux/serdev.h>
#include<linux/mod_devicetable.h>
#include<linux/property.h>
#include<linux/platform_device.h>
#include<linux/of_device.h>
#include<linux/spi/spi.h>
#include<linux/string.h>

// Registers of BMP280

#define BMP280_REG_ID         0xD0
#define BMP280_REG_RESET      0xE0
#define BMP280_REG_STATUS     0xF3
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_PRESS_MSB  0xF7  /* F7..F9 */
#define BMP280_REG_TEMP_MSB   0xFA  /* FA..FC */
#define BMP280_REG_CALIB      0x88  /* 0x88..0xA1 */


#define DEVICE_NAME "integrated_driver"
#define SLAVE_DEVICE_NAME ("ETX_OLED") // Oled display name
#define SSD1306_SLAVE_ADDR (0x3C) // oled display address
#define CLASS_NAME  "integrated_module"
#define SDSPI_BLOCK_SIZE 512
#define BMP280_I2C_ADDR_DEF  0x76   //BMP280 default address

static dev_t dev_num;
static struct cdev integrated_driver_cdev;
static struct class *integrated_driver_class;
static struct device *integrated_device;
static struct i2c_client *bmp_client;
static struct i2c_client *oled_client;
static struct serdev_device *uart_serdev;
static struct spi_device *sdspi_spi_device;
static char recv_buffer[64];
static size_t recv_pos=0;
static u64 sd_total_blocks=0;
static char data_buffer[512],spi_data_send[512],spi_data_recv[512];

/* Calibration data layout (per Bosch datasheet) */
struct bmp280_calib {
	u16 dig_T1;
	s16 dig_T2, dig_T3;
	u16 dig_P1;
	s16 dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
};

struct bmp280_calib calib; // structure variable for reading callibration data
s32 t_fine; // used for compensation  

/*..................................................SSD1306 Display Functions.........................................*/

// Symbol Table for oled display
const uint8_t symbol_table[6][5]={
        {0x00,0x00,0xF8,0X04,0X02},
        {0X04,0XF8,0X00,0XF8,0XF8},
        {0X00,0X00,0XFF,0X00,0XFC},
        {0X00,0XFF,0X00,0X7F,0X7F},
        {0X1E,0X21,0X5E,0XCF,0XCF},
        {0XEF,0X5E,0X21,0X1E,0X00},
};

//Characters Table for Oled Display
const uint8_t font_table[96][5] = {
  {0x00,0x00,0x00,0x00,0x00}, // SPACE
  {0x00,0x00,0x5F,0x00,0x00}, // !
  {0x00,0x07,0x00,0x07,0x00}, // "
  {0x14,0x7F,0x14,0x7F,0x14}, // #
  {0x24,0x2A,0x7F,0x2A,0x12}, // $
  {0x23,0x13,0x08,0x64,0x62}, // %
  {0x36,0x49,0x55,0x22,0x50}, // &
  {0x00,0x05,0x03,0x00,0x00}, // '
  {0x00,0x1C,0x22,0x41,0x00}, // (
  {0x00,0x41,0x22,0x1C,0x00}, // )
  {0x14,0x08,0x3E,0x08,0x14}, // *
  {0x08,0x08,0x3E,0x08,0x08}, // +
  {0x00,0x50,0x30,0x00,0x00}, // ,
  {0x08,0x08,0x08,0x08,0x08}, // -
  {0x00,0x60,0x60,0x00,0x00}, // .
  {0x20,0x10,0x08,0x04,0x02}, // /
  {0x3E,0x51,0x49,0x45,0x3E}, // 0
  {0x00,0x42,0x7F,0x40,0x00}, // 1
  {0x42,0x61,0x51,0x49,0x46}, // 2
  {0x21,0x41,0x45,0x4B,0x31}, // 3
  {0x18,0x14,0x12,0x7F,0x10}, // 4
  {0x27,0x45,0x45,0x45,0x39}, // 5
  {0x3C,0x4A,0x49,0x49,0x30}, // 6
  {0x01,0x71,0x09,0x05,0x03}, // 7
  {0x36,0x49,0x49,0x49,0x36}, // 8
  {0x06,0x49,0x49,0x29,0x1E}, // 9
  {0x00,0x36,0x36,0x00,0x00}, // :
  {0x00,0x56,0x36,0x00,0x00}, // ;
  {0x08,0x14,0x22,0x41,0x00}, // <
  {0x14,0x14,0x14,0x14,0x14}, // =
  {0x00,0x41,0x22,0x14,0x08}, // >
  {0x02,0x01,0x51,0x09,0x06}, // ?
  {0x32,0x49,0x79,0x41,0x3E}, // @
  {0x7E,0x11,0x11,0x11,0x7E}, // A
  {0x7F,0x49,0x49,0x49,0x36}, // B
  {0x3E,0x41,0x41,0x41,0x22}, // C
  {0x7F,0x41,0x41,0x22,0x1C}, // D
  {0x7F,0x49,0x49,0x49,0x41}, // E
  {0x7F,0x09,0x09,0x09,0x01}, // F
  {0x3E,0x41,0x49,0x49,0x7A}, // G
  {0x7F,0x08,0x08,0x08,0x7F}, // H
  {0x00,0x41,0x7F,0x41,0x00}, // I
  {0x20,0x40,0x41,0x3F,0x01}, // J
  {0x7F,0x08,0x14,0x22,0x41}, // K
  {0x7F,0x40,0x40,0x40,0x40}, // L
  {0x7F,0x02,0x04,0x02,0x7F}, // M
  {0x7F,0x04,0x08,0x10,0x7F}, // N
  {0x3E,0x41,0x41,0x41,0x3E}, // O
  {0x7F,0x09,0x09,0x09,0x06}, // P
  {0x3E,0x41,0x51,0x21,0x5E}, // Q
  {0x7F,0x09,0x19,0x29,0x46}, // R
  {0x46,0x49,0x49,0x49,0x31}, // S
  {0x01,0x01,0x7F,0x01,0x01}, // T
  {0x3F,0x40,0x40,0x40,0x3F}, // U
  {0x1F,0x20,0x40,0x20,0x1F}, // V
  {0x3F,0x40,0x38,0x40,0x3F}, // W
  {0x63,0x14,0x08,0x14,0x63}, // X
  {0x07,0x08,0x70,0x08,0x07}, // Y
  {0x61,0x51,0x49,0x45,0x43}, // Z
  {0x00,0x7F,0x41,0x41,0x00}, // [
  {0x02,0x04,0x08,0x10,0x20}, // Backslash
  {0x00,0x41,0x41,0x7F,0x00}, // ]
  {0x04,0x02,0x01,0x02,0x04}, // ^
  {0x40,0x40,0x40,0x40,0x40}, // _
  {0x00,0x01,0x02,0x04,0x00}, // `
  {0x20,0x54,0x54,0x54,0x78}, // a
  {0x7F,0x48,0x44,0x44,0x38}, // b
  {0x38,0x44,0x44,0x44,0x20}, // c
  {0x38,0x44,0x44,0x48,0x7F}, // d
  {0x38,0x54,0x54,0x54,0x18}, // e
  {0x08,0x7E,0x09,0x01,0x02}, // f
  {0x0C,0x52,0x52,0x52,0x3E}, // g
  {0x7F,0x08,0x04,0x04,0x78}, // h
  {0x00,0x44,0x7D,0x40,0x00}, // i
  {0x20,0x40,0x44,0x3D,0x00}, // j
  {0x7F,0x10,0x28,0x44,0x00}, // k
  {0x00,0x41,0x7F,0x40,0x00}, // l
  {0x7C,0x04,0x18,0x04,0x78}, // m
  {0x7C,0x08,0x04,0x04,0x78}, // n
  {0x38,0x44,0x44,0x44,0x38}, // o
  {0x7C,0x14,0x14,0x14,0x08}, // p
  {0x08,0x14,0x14,0x18,0x7C}, // q
  {0x7C,0x08,0x04,0x04,0x08}, // r
  {0x48,0x54,0x54,0x54,0x20}, // s
  {0x04,0x3F,0x44,0x40,0x20}, // t
  {0x3C,0x40,0x40,0x20,0x7C}, // u
  {0x1C,0x20,0x40,0x20,0x1C}, // v
  {0x3C,0x40,0x30,0x40,0x3C}, // w
  {0x44,0x28,0x10,0x28,0x44}, // x
  {0x0C,0x50,0x50,0x50,0x3C}, // y
  {0x44,0x64,0x54,0x4C,0x44}, // z
  {0x00,0x08,0x36,0x41,0x00}, // {
  {0x00,0x00,0x7F,0x00,0x00}, // |
  {0x00,0x41,0x36,0x08,0x00}, // }
  {0x08,0x08,0x2A,0x1C,0x08}, // ->
};

static int I2C_Write(struct i2c_client *client,unsigned char *buf, unsigned int len) //Function to send data over I2c
{
    int ret = i2c_master_send(client, buf, len);
     return ret;
}

static void SSD1306_Write(struct i2c_client *client,bool is_cmd, unsigned char data) //Function to Write data to SSD1306 Display
{
    unsigned char buf[2] = {0};
    int ret;

    if( is_cmd == true)  //is_cmd -> true=command, false=data
    {
        buf[0] = 0x00;
    }
    else
    {
        buf[0] = 0x40;
    }

    buf[1] = data;  //data -> data to be written

    ret = I2C_Write(client,buf, 2);
}
static void SSD1306_SetCursor(struct i2c_client *client,uint8_t x, uint8_t page) // Function to set page and coloumn address
{
    SSD1306_Write(client,true, 0xB0 + page);                    // Set page start address
    SSD1306_Write(client,true, 0x00 + (x & 0x0F));              // Set lower segment address bits
    SSD1306_Write(client,true, 0x10 + ((x >> 4) & 0x0F));       // Set higher segment address bits
}
static void SSD1306_WriteChar(struct i2c_client *client,char ch) //Function to write characters to SSD1306 Display
{
	const uint8_t *bitmap;
    if (ch < 0x20 || ch > 0x7E) ch = '?'; // Fallback
    	bitmap = font_table[ch - 0x20];

    for (int i = 0; i < 5; i++) 
    {
        SSD1306_Write(client,false, bitmap[i]);
    }

    SSD1306_Write(client,false, 0x00); // spacing between characters
}


static void SSD1306_WriteString(struct i2c_client *client,uint8_t x, uint8_t page, const char *str) //Function to Write string to SSD1306 Display
{
    SSD1306_SetCursor(client,x, page);

    while (*str) 
    {
        SSD1306_WriteChar(client,*str++);
    }
}
static void SSD1306_symbol(struct i2c_client *client) //Function to print symbol in SSD1306 Display
{
        int page=3;
        for(int i=0; i<6;i++)
        {
                const uint8_t *symbol=symbol_table[i];
                if(i%2==0)
                {
                        SSD1306_SetCursor(client,0,page++);
                }
                for(int j=0;j<5;j++)
                SSD1306_Write(client,false,symbol[j]);
        }
}
static int SSD1306_DisplayInit(struct i2c_client *client) // SSD1306 Display Init function
{
    msleep(100);               // delay

    SSD1306_Write(client,true, 0xAE); // Entire Display OFF
    SSD1306_Write(client,true, 0xD5); // Set Display Clock Divide Ratio and Oscillator Frequency
    SSD1306_Write(client,true, 0x80); // Default Setting for Display Clock Divide Ratio and Oscillator Frequency that is recommended
    SSD1306_Write(client,true, 0xA8); // Set Multiplex Ratio
    SSD1306_Write(client,true, 0x3F); // 64 COM lines
    SSD1306_Write(client,true, 0xD3); // Set display offset
    SSD1306_Write(client,true, 0x00); // 0 offset
    SSD1306_Write(client,true, 0x40); // Set first line as the start line of the display
    SSD1306_Write(client,true, 0x8D); // Charge pump
    SSD1306_Write(client,true, 0x14); // Enable charge dump during display on
    SSD1306_Write(client,true, 0x20); // Set memory addressing mode
    SSD1306_Write(client,true, 0x00); // Horizontal addressing mode
    SSD1306_Write(client,true, 0xA1); // Set segment remap with column address 127 mapped to segment 0
    SSD1306_Write(client,true, 0xC8); // Set com output scan direction, scan from com63 to com 0
    SSD1306_Write(client,true, 0xDA); // Set com pins hardware configuration
    SSD1306_Write(client,true, 0x12); // Alternative com pin configuration, disable com left/right remap
    SSD1306_Write(client,true, 0x81); // Set contrast control
    SSD1306_Write(client,true, 0x80); // Set Contrast to 128
    SSD1306_Write(client,true, 0xD9); // Set pre-charge period
    SSD1306_Write(client,true, 0xF1); // Phase 1 period of 15 DCLK, Phase 2 period of 1 DCLK
    SSD1306_Write(client,true, 0xDB); // Set Vcomh deselect level
    SSD1306_Write(client,true, 0x20); // Vcomh deselect level ~ 0.77 Vcc
    SSD1306_Write(client,true, 0xA4); // Entire display ON, resume to RAM content display
    SSD1306_Write(client,true, 0xA6); // Set Display in Normal Mode, 1 = ON, 0 = OFF
    SSD1306_Write(client,true, 0x2E); // Deactivate scroll
    SSD1306_Write(client,true, 0xAF); // Display ON in normal mode

    return 0;
}

static void SSD1306_Fill(struct i2c_client *client,unsigned char data)// Function to fill the display with a particular data
{
    unsigned int total  = 128 * 8;  // 8 pages x 128 segments x 8 bits of data
    unsigned int i      = 0;

    //Fill the Display
    for(i = 0; i < total; i++)
    {
        SSD1306_Write(client,false, data);
    }
}

static int ssd1306_probe(struct i2c_client *client) //probe function for ssd display
{

	pr_info("SSD1306 Display Probed successfully\n");
	oled_client=client;
	SSD1306_DisplayInit(client);
	SSD1306_Fill(client,0X00);
	//SSD1306_symbol(client);
	return 0;
}

static void ssd1306_remove(struct i2c_client *client) // Remove function for ssd display
{
	oled_client=NULL;
	SSD1306_Fill(client,0X00);
	pr_info("SSD1306 removed successfully\n");
}

//Match table for SSD1306 Display
static const struct of_device_id ssd_of_match[]={
	{.compatible ="myssd,myssd1306"},
	{}
};
MODULE_DEVICE_TABLE(of,ssd_of_match);

static struct i2c_driver oled_driver={
	.driver={
		.name="ssd1306",
		.of_match_table=ssd_of_match,
	},
	.probe=ssd1306_probe,
	.remove=ssd1306_remove,
};


/*.....................................BMP280 Sensor Functions.............................................*/

static int bmp280_read_calib(struct i2c_client *client, struct bmp280_calib *c) // Read calibration blob
{
	u8 b[24];
	int ret = i2c_smbus_read_i2c_block_data(client, BMP280_REG_CALIB, sizeof(b), b);
	if (ret < 0)
		return ret;

	c->dig_T1 = (u16)(b[1] << 8 | b[0]);
	c->dig_T2 = (s16)(b[3] << 8 | b[2]);
	c->dig_T3 = (s16)(b[5] << 8 | b[4]);

	c->dig_P1 = (u16)(b[7]  << 8 | b[6]);
	c->dig_P2 = (s16)(b[9]  << 8 | b[8]);
	c->dig_P3 = (s16)(b[11] << 8 | b[10]);
	c->dig_P4 = (s16)(b[13] << 8 | b[12]);
   	c->dig_P5 = (s16)(b[15] << 8 | b[14]);
	c->dig_P6 = (s16)(b[17] << 8 | b[16]);
	c->dig_P7 = (s16)(b[19] << 8 | b[18]);
	c->dig_P8 = (s16)(b[21] << 8 | b[20]);
	c->dig_P9 = (s16)(b[23] << 8 | b[22]);

	return 0;
}

static s32 bmp280_compensate_T(struct bmp280_calib *d, s32 adc_T) // Compensation formulas from Bosch datasheet (integer variant) 
{
	s32 var1, var2, T;
	var1 = ((((adc_T >> 3) - ((s32)d->dig_T1 << 1))) * ((s32)d->dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((s32)d->dig_T1)) * ((adc_T >> 4) - ((s32)d->dig_T1))) >> 12) * (s32)d->dig_T3) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8; /* temperature in 0.01 °C */
	return T;
}

static u32 bmp280_compensate_P(struct bmp280_calib *d, s32 adc_P)
{
	s64 var1, var2, p;

	var1 = ((s64)t_fine) - 128000;
	var2 = var1 * var1 * (s64)d->dig_P6;
	var2 = var2 + ((var1 * (s64)d->dig_P5) << 17);
	var2 = var2 + (((s64)d->dig_P4) << 35);
	var1 = ((var1 * var1 * (s64)d->dig_P3) >> 8) + ((var1 * (s64)d->dig_P2) << 12);
	var1 = (((((s64)1) << 47) + var1)) * ((s64)d->dig_P1) >> 33;

	if (var1 == 0)
		return 0; /* avoid division by zero */

	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((s64)d->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((s64)d->dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((s64)d->dig_P7) << 4);
	return (u32)p; /* pressure in Q24.8 format (Pa * 256) */
}

static int bmp280_take_measurement(struct bmp280_calib *d, int *temp_c_x100, int *press_hpa_x100)
{
	int ret;
	u8 buf[6];
	s32 adc_T, adc_P;
	u32 p_q24_8;
	int t_x100;
	int p_pa;

	/* ctrl_meas: osrs_t=x1 (001), osrs_p=x1 (001), mode=forced (01) => 0b00100101 = 0x25 */
	ret = i2c_smbus_write_byte_data(bmp_client, BMP280_REG_CTRL_MEAS, 0x25);
	if (ret < 0)
		return ret;

	/* Max conversion ~7.5ms at x1; wait a safe 10ms */
	msleep(10);

	/* Read pressure(3 bytes) + temperature(3 bytes) */
	ret = i2c_smbus_read_i2c_block_data(bmp_client, BMP280_REG_PRESS_MSB, 6, buf);
	if (ret < 0)
		return ret;

	adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
	adc_T = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);

	t_x100 = bmp280_compensate_T(d, adc_T);               /* 0.01 °C */
	p_q24_8 = bmp280_compensate_P(d, adc_P);              /* Q24.8 Pa */
	p_pa = (int)(p_q24_8 >> 8);                           /* Pa */
	*temp_c_x100 = t_x100;
	*press_hpa_x100 = (p_pa * 100) / 100; /* convert Pa -> hPa*100 (1 hPa = 100 Pa) */
	return 0;
}

static int bmp280_probe(struct i2c_client *client)
{
	bmp_client=client;
	bmp280_read_calib(bmp_client,&calib);
	pr_info("Probed BMP280 Sensor successfully\n");
	return 0;
}


static void bmp280_remove(struct i2c_client *client)
{

	pr_info("BMP280 removed successfully\n");
}

// Match table for BMP280 sensor
static const struct of_device_id bmp280_of_match[] = {
    { .compatible = "myown,mybmp280" },
    { }
};
MODULE_DEVICE_TABLE(of, bmp280_of_match);

static struct i2c_driver bmp280_driver = {
	.driver = {
		.name = "bmp280",
		.of_match_table=bmp280_of_match,
	},
	.probe    = bmp280_probe,
	.remove   = bmp280_remove,
};
/*....................................... UART DEVICE FUNCTIONS.....................................................*/

static size_t serdev_echo_recv(struct serdev_device *serdev, const unsigned char *buffer, size_t size)// Receive data from UART 
{
        size_t i;
        for(i=0;i<size && recv_pos<sizeof(recv_buffer)-1;i++)
        {
                char c=buffer[i];

                if(c=='\n'|| c=='\r')
                {
                        if(recv_pos>0)
                        {
                        recv_buffer[recv_pos]='\0';
                        recv_pos=0;
                        }
                        continue;
                }
               		 recv_buffer[recv_pos++]=c;
        }
        return  size;
}
static const struct serdev_device_ops serdev_echo_ops = {
        .receive_buf = serdev_echo_recv,
};

static int serdev_echo_probe(struct serdev_device *serdev) {
        int status;

        serdev_device_set_client_ops(serdev, &serdev_echo_ops);
        status = serdev_device_open(serdev);
        if(status) {
                printk("serdev_echo - Error opening serial port!\n");
                return -status;
        }

        serdev_device_set_baudrate(serdev, 9600);
        serdev_device_set_flow_control(serdev, false);
        serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	uart_serdev=serdev;
        printk("Probed Uart sucessfully\n");
        return 0;
}
static void serdev_echo_remove(struct serdev_device *serdev) {
        printk("Removed UART Device successfully\n");
        serdev_device_close(serdev);
}

//Match Table for UART driver
static struct of_device_id serdev_echo_ids[] = {
        {
                .compatible = "brightlight,echodev",
        }, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, serdev_echo_ids);

static struct serdev_device_driver serdev_echo_driver = {
        .probe = serdev_echo_probe,
        .remove = serdev_echo_remove,
        .driver = {
                .name = "serdev-echo",
                .of_match_table = serdev_echo_ids,
        },
};


/*....................................SPI Functions...............................................................................*/

static void sdspi_send_dummy_clocks(struct spi_device *spi)
{
    u8 dummy = 0xFF;
    int i;
    for (i = 0; i < 10; i++)
        spi_write(spi, &dummy, 1);  // 80 clocks
    pr_info("sdspi: dummy clocks sent\n");
}

static int sdspi_send_cmd0(struct spi_device *spi)// CMD0 to reset the card, response =0x01
{
    u8 cmd[] = {0x40, 0, 0, 0, 0, 0x95}; 
    u8 r1 = 0xFF; //response
    int retries = 100;

    spi_write(spi, cmd, 6);

    while (retries--){
        spi_read(spi, &r1, 1);
        if (r1 != 0xFF) break; //Initial value is changed, so response received successfully
        msleep(1);
    }

    if (r1 != 0x01) {
        pr_err("sdspi: CMD0 failed, resp=0x%02x\n", r1);
        return -EIO;
    }

    pr_info("sdspi: CMD0 success, card in idle state\n");
    return 0;
}

static int sdspi_send_cmd8(struct spi_device *spi)// to check the card version and voltage that it supports, response=0x01
{
    u8 cmd[] = {0x40 | 8, 0x00, 0x00, 0x01, 0xAA, 0x87};  // CMD8
    u8 r1 = 0xFF;       // R1 response
    u8 r7[4] = {0};     // R7 response
    int retries = 10;
    int i;

    spi_write(spi, cmd, 6);

    //Read R1 response
    while(retries--) {
        spi_read(spi, &r1, 1);
        if (r1 != 0xFF) break;
        msleep(1);
    }

    if (r1 != 0x01) {
        pr_err("sdspi: CMD8 failed, r1=0x%02x\n", r1);
        return -EIO;
    }

    // Now read 4 more bytes (R7-specific response)
    for (i = 0; i < 4; i++)
        spi_read(spi, &r7[i], 1);

    if (r7[2] != 0x01 || r7[3] != 0xAA) {
        pr_err("sdspi: CMD8 echo check failed, got: %02x %02x %02x %02x\n",
               r7[0], r7[1], r7[2], r7[3]);
        return -EIO;
    }

    pr_info("sdspi: CMD8 success, card supports SD v2.0+, check pattern matched\n");
    return 0;
}
static int sdspi_send_cmd55(struct spi_device *spi)//To indicate the next command is application command, response=0x01
{
    u8 cmd[] = {0x40 | 55, 0, 0, 0, 0, 0x01}; // CMD55, CRC=0x65
    u8 r1 = 0xFF;
    int ret, retries = 100; // Retry up to 100 times (~10ms total)

    // Send CMD55
    ret = spi_write(spi, cmd, 6);
    if (ret) {
        pr_err("sdspi: CMD55 transfer failed\n");
        return ret;
    }

    // Wait for R1 response
    while (retries--) {
        ret = spi_read(spi, &r1, 1);
        if (ret)
                return ret;
        if ((r1 & 0x80) == 0) // Valid response
            break;
        udelay(100); // Wait a bit
    }

    if (r1 > 0x01) {
        pr_err("sdspi: CMD55 invalid R1=0x%02x\n", r1);
        return -EIO;
    }

    // Send dummy clock after CMD55
    u8 dummy = 0xFF;
    spi_write(spi, &dummy, 1);

    msleep(1); // Delay before ACMD41
    pr_info("sdspi: CMD55 success, r1=0x%02x\n", r1);
    return 0;
}
static int sdspi_send_acmd41(struct spi_device *spi)// card initialization, response=0x00
{
    int retries = 200;
    u8 cmd[] = {0x40 | 41, 0x40, 0x00, 0x00, 0x00, 0x01}; // ACMD41, HCS=1
    u8 r1 = 0xFF;
    int ret;

    while (retries--){
        ret = sdspi_send_cmd55(spi);
        if (ret)
            return ret;

        // Send ACMD41
        ret = spi_write(spi, cmd, 6);
        if (ret)
            return ret;

        // Wait for valid R1
        int tries = 100;
        while (tries--) {
            ret = spi_read(spi, &r1, 1);
            if (ret)
                return ret;
            if ((r1 & 0x80) == 0)
                break;
            udelay(100);
        }

        if (r1 == 0x00)
            break;

        pr_debug("sdspi: ACMD41 busy, r1=0x%02x\n", r1);
        msleep(50);
    }

    if (r1 != 0x00) {
        pr_err("sdspi: ACMD41 failed, r1=0x%02x\n", r1);
        return -EIO;
    }

    pr_info("sdspi: ACMD41 success, card initialized\n");
    return 0;
}
static int sdspi_send_cmd58(struct spi_device *spi, u8 *ocr)//command to read OCR(operating condition register) to confirm initialization, response=0x00
{
    u8 cmd[6]={0x40 | 58, 0x00, 0x00, 0x00, 0x00, 0x01};
    u8 response;
    int ret, retries = 10;

    // Send CMD58
    ret = spi_write(spi, cmd, 6);
    if (ret) {
        pr_err("sdspi: Failed to send CMD58\n");
        return ret;
    }

    // Wait for R1 response
    do {
        ret = spi_read(spi, &response, 1);
        if (ret)
            return ret;
        retries--;
    } while ((response & 0x80) && retries);

    if (retries == 0) {
        pr_err("sdspi: CMD58 response timeout\n");
        return -EIO;
    }

    if ((response != 0x00) && (response != 0x01)) {
        pr_err("sdspi: CMD58 invalid R1 response: 0x%02x\n", response);
        return -EIO;
    }

    // Read 4 OCR bytes
    ret = spi_read(spi, ocr, 4);
    if (ret) {
        pr_err("sdspi: Failed to read OCR\n");
        return ret;
    }

    // Combine OCR bytes into 32-bit value
    u32 ocr_val = (ocr[0] << 24) | (ocr[1] << 16) | (ocr[2] << 8) | ocr[3];

    // Print OCR for debug
    pr_info("sdspi: CMD58 OCR: %02x %02x %02x %02x\n", ocr[0], ocr[1], ocr[2], ocr[3]);

    // Voltage range check (2.7–3.6V bits: 23–15)
    if (!(ocr_val & 0x00FF8000)) {
        pr_err("sdspi: Card does NOT support 2.7–3.6V voltage range\n");
        return -EIO;
    }

    // Power-up complete bit (bit 31)
    if (!(ocr_val & 0x80000000)) {
        pr_warn("sdspi: Card is still busy (power-up not complete)\n");
    }

    // Card Capacity Status (bit 30)
    if (ocr_val & 0x40000000)
        pr_info("sdspi: Card is SDHC/SDXC\n");
    else
        pr_info("sdspi: Card is SDSC (Standard Capacity)\n");

    return 0;
}

static int sdspi_send_cmd17(struct spi_device *spi, u32 block_addr, u8 *buffer)// Reading the block from SD card
{
    u8 cmd[6];
    u8 response = 0xFF;
    u8 token = 0xFF;
    int ret;
    int i;

    // CMD17 packet (READ_SINGLE_BLOCK)
    cmd[0] = 0x40 | 17;
    cmd[1] = (block_addr >> 24) & 0xFF;
    cmd[2] = (block_addr >> 16) & 0xFF;
    cmd[3] = (block_addr >> 8) & 0xFF;
    cmd[4] = block_addr & 0xFF;
    cmd[5] = 0x01;  // dummy CRC


    // Send CMD17
    ret = spi_write(spi, cmd, 6);
    if (ret)
        return ret;

    // Read R1 response (up to 8 retries)
    for (i = 0; i < 8; i++) {
        ret = spi_read(spi, &response, 1);
        if (ret)
            return ret;
        if (!(response & 0x80))
            break;
    }

    if (response != 0x00) {
        pr_err("sdspi: CMD17 failed, R1=0x%02x\n", response);
        return -EIO;
    }

    // Wait for start token (0xFE), up to 1000 attempts, max time is 100ms which is aprox to 5000 bytes
    for (i = 0; i < 1000; i++) {
        ret = spi_read(spi, &token, 1);
        if (ret)
            return ret;
        if (token != 0xFF)
            break;
    }

    if (token != 0xFE) {
        pr_err("sdspi: CMD17 failed, expected start token 0xFE, got 0x%02x\n", token);
        return -EIO;
    }

    // Read 512 bytes
    ret = spi_read(spi, buffer, 512);
    if (ret)
        return ret;

    // Read and discard 2-byte CRC
    u8 crc[2];
    ret = spi_read(spi, crc, 2);
    if (ret)
        return ret;

    // Wait for the sd card to exit from busy state
    u8 busy;
    do {
        spi_read(spi, &busy, 1);
    } while (busy != 0xFF);

    // Send extra 8 clocks after deasserting CS (simulate CS_DISABLE)
    u8 idle[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    spi_write(spi, idle, 8);

    return 0;
}
static int sdspi_send_cmd24(struct spi_device *spi, u32 block_addr, const u8 *data)
{
    u8 cmd[6];
    u8 response;
    int retries, ret;

    // CMD24: Write Single Block
    cmd[0] = 0x40 | 24;

    // For SDHC/SDXC cards, block addressing is used
    cmd[1] = (block_addr >> 24) & 0xFF;
    cmd[2] = (block_addr >> 16) & 0xFF;
    cmd[3] = (block_addr >> 8) & 0xFF;
    cmd[4] = block_addr & 0xFF;

    cmd[5] = 0x01; // Dummy CRC

    // Send 1 byte gap
    u8 gap = 0xFF;
    spi_write(spi, &gap, 1);

    // Send CMD24
    ret = spi_write(spi, cmd, 6);
    if (ret)
        return ret;

    // Wait for R1 response
    retries = 8;
    do {
        ret = spi_read(spi, &response, 1);
        if (ret)
            return ret;
        if ((response & 0x80) == 0)
            break;
        udelay(100);
    } while (--retries);

    if (response != 0x00) {
        pr_err("CMD24 failed, R1 = 0x%02X\n", response);
        return -EIO;
    }

    // Send data start token (0xFE)
    u8 token = 0xFE;
    ret = spi_write(spi, &token, 1);
    if (ret)
        return ret;

    // Send 512 bytes data block
    ret = spi_write(spi, data, 512);
    if (ret)
        return ret;

    // Send 2 bytes dummy CRC
    u8 crc[2] = { 0xFF, 0xFF };
    ret = spi_write(spi, crc, 2);
    if (ret)
        return ret;

    // Read data response token
    ret = spi_read(spi, &response, 1);
    if (ret)
        return ret;
    // Check if data accepted (0x05)
    if ((response & 0x1F) != 0x05) {
        pr_err("Write rejected, response = 0x%02X\n", response);
        return -EIO;
    }

    // Wait for card to complete the write (busy wait)
    retries = 100000;
    do {
        ret = spi_read(spi, &response, 1);
        if (ret)
            return ret;
        if (response == 0xFF)
            break;
        udelay(10);
    } while (--retries);

    if (retries == 0) {
        pr_err("Timeout waiting for write complete\n");
        return -ETIMEDOUT;
    }

    // Send 8 idle clocks to ensure proper card release
    u8 idle[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    ret = spi_write(spi, idle, 8);

    return ret;
}
static int sdspi_card_init(struct spi_device *spi)
{
    int ret;
    u8 ocr[4];
    int retries = 10;

    sdspi_send_dummy_clocks(spi);

    ret = sdspi_send_cmd0(spi);
    if (ret) return ret;

    ret = sdspi_send_cmd8(spi);
    if (ret) return ret;

    do {
        ret = sdspi_send_acmd41(spi);
        if (ret) return ret;

        msleep(10);

        ret = sdspi_send_cmd58(spi, ocr);
        if (ret)
            pr_warn("sdspi: CMD58 failed during power-up polling\n");

        u32 ocr_val = (ocr[0] << 24) | (ocr[1] << 16) | (ocr[2] << 8) | ocr[3];
        if (ocr_val & 0x80000000) // Card has powered up (bit 31 set)
            break;

    } while (--retries);

    if (retries == 0) {
        pr_err("sdspi: Timeout waiting for card power-up\n");
        return -ETIMEDOUT;
    }

    return 0;
}
static int sd_probe(struct spi_device *spi)
{
	int ret;
	sdspi_spi_device=spi;
	spi->mode=SPI_MODE_0;
	spi->bits_per_word=8;
	spi->max_speed_hz=400000;
	ret=spi_setup(spi);
	sd_total_blocks=(u64)32*1024*1024*1024/512;
	if(ret)
	{
		pr_err("sdspi: SPI_setup_failed\n");
		return ret;
	}
	ret=sdspi_card_init(spi);
	if(ret)
		pr_warn("sdspi:card init failed\n");
	return 0;
}
static void sd_remove(struct spi_device *spi)
{
	pr_info("sd card device removed\n");
}

// Match table for BMP280 sensor
// Match table for SPI device IDs
static const struct spi_device_id sdspi_ids[] = {
    { "mysdspi", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, sdspi_ids);

static const struct of_device_id sdspi_of_match[] = {
    { .compatible = "myown,mysdspi" },
    { }
};
MODULE_DEVICE_TABLE(of, sdspi_of_match);

static struct spi_driver sdspi_driver={
	.driver={
		.name="sdspi",
		.of_match_table=sdspi_of_match,
	},
	.probe=sd_probe,
	.remove=sd_remove,
	.id_table=sdspi_ids,
};

/*.....................................Device File Operations.......................................................................*/
static ssize_t Device_file_read(struct file *file,char __user *buf,size_t len,loff_t *ppos)
{
	size_t data_len = strnlen(data_buffer, sizeof(data_buffer));

	if (*ppos >= data_len)
		return 0;

	if (len > data_len - *ppos)
		len = data_len - *ppos;

	if (copy_to_user(buf, data_buffer + *ppos, len))
		return -EFAULT;

	*ppos += len;
	return len;
}
static ssize_t Device_file_write(struct file *file,const char __user *buf,size_t len,loff_t *ppos)
{
	char  user_msg[128];
	int   temp_x100, press_x100;
	int   ret;
	size_t bytes;
	int   block_num;

	/* Limit copy to our buffer size - 1 for NUL */
	bytes = min(len, sizeof(user_msg) - 1);

	if (copy_from_user(user_msg, buf, bytes))
		return -EFAULT;

	user_msg[bytes] = '\0';   /* Make sure it is NUL terminated */

	/* --------- BMP280: print temp ---------- */
	if (strcmp(user_msg, "PRINT_TMP") == 0) {
		ret = bmp280_take_measurement(&calib, &temp_x100, &press_x100);
		if (ret < 0)
			return ret;

		snprintf(data_buffer, sizeof(data_buffer),"Temp=%d.%02d C", temp_x100 / 100, temp_x100 % 100);
	}
	/* --------- BMP280: print pressure ---------- */
	else if (strcmp(user_msg, "PRINT_PRE") == 0) {
		ret = bmp280_take_measurement(&calib, &temp_x100, &press_x100);
		if (ret < 0)
			return ret;

		snprintf(data_buffer, sizeof(data_buffer),"Press=%d.%02d hpa", press_x100 / 100, press_x100 % 100);
	}
	/* --------- BMP280: display both on SSD1306 ---------- */
	else if (strcmp(user_msg, "PRINT_DIS") == 0) {
		char temp_buf[32];
		char pre_buf[32];

		ret = bmp280_take_measurement(&calib, &temp_x100, &press_x100);
		if (ret < 0)
			return ret;

		snprintf(temp_buf, sizeof(temp_buf),"Temp=%d.%02d C", temp_x100 / 100, temp_x100 % 100);
		snprintf(pre_buf, sizeof(pre_buf),"Press=%d.%02d hpa", press_x100 / 100, press_x100 % 100);

		SSD1306_symbol(oled_client);
		SSD1306_WriteString(oled_client, 15, 3, temp_buf);
		SSD1306_WriteString(oled_client, 15, 5, pre_buf);
	}
	/* --------- SD card write: "WRITE_SD <block> <text>" ---------- */
	else if (strncmp(user_msg, "WRITE_SD", 8) == 0) {
    		char *p = user_msg + 8;
    		char blk_str[16];
    		int i = 0;
    		char *data_start;

   		 pr_info("WRITE_SD: raw command = '%s'\n", user_msg);

    		/* Skip spaces after WRITE_SD */
    		while (*p == ' ' || *p == '\t')
        		p++;

   		/* Copy only digits of block number into blk_str */
    		while (*p && *p != ' ' && *p != '\t' && i < (int)sizeof(blk_str) - 1) {
        		if (*p < '0' || *p > '9') {
            			pr_err("WRITE_SD: non-digit in block number: '%c'\n", *p);
            			return -EINVAL;
        		}	
        		blk_str[i++] = *p;
        		p++;
    		}
    		blk_str[i] = '\0';

    		if (i == 0) {
        	pr_err("WRITE_SD: missing block number\n");
        	return -EINVAL;
    		}

    		ret = kstrtoint(blk_str, 10, &block_num);
    		if (ret) {
        	pr_err("WRITE_SD: kstrtoint failed for '%s', ret=%d\n", blk_str, ret);
        	return -EINVAL;
    		}

   		 pr_info("WRITE_SD: parsed block_num=%d\n", block_num);

    		/* Skip spaces before data */
    		while (*p == ' ' || *p == '\t')
        		p++;

   		 data_start = p;   /* Rest of user_msg is data */

    		pr_info("WRITE_SD: data_start='%s'\n", data_start);

    		memset(spi_data_send, 0, SDSPI_BLOCK_SIZE);
    		strscpy(spi_data_send, data_start, SDSPI_BLOCK_SIZE);

    		pr_info("WRITE_SD: calling sdspi_send_cmd24...\n");

    		ret = sdspi_send_cmd24(sdspi_spi_device, block_num,(const u8 *)spi_data_send);
    		if (ret < 0) {
        		pr_err("WRITE_SD: sdspi_send_cmd24 returned %d\n", ret);
        		return ret;   /* Notice: we return ret, *not* -EINVAL */
    		}

    		pr_info("WRITE_SD: write successful\n");
	}
	/* --------- SD card read: "READ_SD <block>" ---------- */
	else if (strncmp(user_msg, "READ_SD", 7) == 0) {
		char *p = user_msg;

		/* pattern: READ_SD <block> */
		p += 7;
		while (*p == ' ' || *p == '\t')
			p++;

		ret = kstrtoint(p, 10, &block_num);
		if (ret)
			return -EINVAL;

		ret = sdspi_send_cmd17(sdspi_spi_device, block_num, spi_data_recv);
		if (ret < 0)
			return ret;

		strscpy(data_buffer, spi_data_recv, sizeof(data_buffer));
	}
	/* --------- UART: "UART_CHECK <text>" ---------- */

	else if (strncmp(user_msg, "UART_CHECK", 10) == 0) 
	{
	       	char *p = user_msg;
	       	size_t send_len;
	       	memset(recv_buffer, 0, sizeof(recv_buffer));
	       recv_pos=0;
	       p += 10;
	       while (*p == ' ' || *p == '\t')
		       p++; /* p now points to text to send over UART (can be empty) */
	       send_len = strnlen(p, bytes - (p - user_msg));
	       if (send_len > 0) 
	       { 
		       serdev_device_write_buf(uart_serdev, p, send_len); } 
	       else 
	       { 
		       /* If no extra text, send a default string or nothing */
		       const char *default_msg = "UART_CHECK";
		       serdev_device_write_buf(uart_serdev, default_msg,strlen(default_msg)); 
	       }
	       msleep(50); /* wait for data to arrive via UART */
	       strscpy(data_buffer, recv_buffer, sizeof(data_buffer)); 
	}

	else {
		pr_err("Unknown command from user: %s\n", user_msg);
		return -EINVAL;
	     }
	if (ppos)
    		*ppos = 0;
        return len;

}
static int Device_file_open(struct inode *inode, struct file *file)
{
	pr_info("Device File opened\n");
    	return 0;
}

static int Device_file_release(struct inode *inode, struct file *file)
{
	SSD1306_Fill(oled_client,0X00);
	pr_info("Device file Closed\n");
   	return 0;
}


static const struct file_operations Driver_fops = {
    .owner   = THIS_MODULE,
    .read    = Device_file_read,
    .write   = Device_file_write,
    .open    = Device_file_open,
    .release = Device_file_release,
};



static int __init all_init(void)
{
	int ret;
	pr_info("\n\n Loading Integrated Driver\n");
	ret=i2c_add_driver(&bmp280_driver);
	if(ret)
	{
		return ret;
	}
	ret=i2c_add_driver(&oled_driver);
	if(ret)
	{
		return ret;
	}
	ret=serdev_device_driver_register(&serdev_echo_driver);
	if(ret)
	{
		return ret;
	}
	ret=spi_register_driver(&sdspi_driver);
	if(ret)
	{
		return ret;
	}
	pr_info("BMP280+SSD1306+SD CARD +UART module loaded\n");

    // Allocate device number
	ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
	if (ret < 0)
    		return ret;

	// Init cdev
	cdev_init(&integrated_driver_cdev, &Driver_fops);

	ret = cdev_add(&integrated_driver_cdev, dev_num, 1);
	if (ret < 0) 
	{
    		unregister_chrdev_region(dev_num, 1);
    		return ret;
	}	

	// Create class and device node
	if (!(integrated_driver_class)) 
	{
    		integrated_driver_class = class_create (CLASS_NAME);
    		if(IS_ERR(integrated_driver_class))
		{
    			cdev_del(&integrated_driver_cdev);
    			unregister_chrdev_region(dev_num, 1);
    			return PTR_ERR(&integrated_driver_cdev);
		}		

		integrated_device = device_create(integrated_driver_class, NULL, dev_num, NULL, DEVICE_NAME);
		if (IS_ERR(integrated_device)) 
		{
    			class_destroy(integrated_driver_class);
    			cdev_del(&integrated_driver_cdev);
   			unregister_chrdev_region(dev_num, 1);
    			return PTR_ERR(integrated_device);
		}
	}	
	pr_info("Integrated driver char device created: /dev/%s\n", DEVICE_NAME);
	return 0;
}

static void __exit all_exit(void)
{
	pr_info("\n\nUnloading Integrated Driver\n");
	i2c_del_driver(&bmp280_driver);
	i2c_del_driver(&oled_driver);
	serdev_device_driver_unregister(&serdev_echo_driver);
	spi_unregister_driver(&sdspi_driver);
	if(integrated_driver_class)
	{	
		device_destroy(integrated_driver_class,dev_num);
		class_destroy(integrated_driver_class);
		cdev_del(&integrated_driver_cdev);
		unregister_chrdev_region(dev_num, 1);
		pr_info("Char device file deleted\n");
	}
	pr_info("Integrated Driver Unloaded\n");
}

module_init(all_init);
module_exit(all_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dinesh");
MODULE_DESCRIPTION("Integrated Device Driver");

