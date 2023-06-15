// SPDX-License-Identifier: GPL-2.0-only
/*
 * hdzerocam.c ST hdzerocam CMOS image sensor driver
 *
 * Copyright (c) 2011 Analog Devices Inc.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>


#define MAX_FRAME_RATE  60
#define MAX_WIDTH 1280
#define MAX_HEIGHT 720
//#define SENSOR_PCLK_RATE 55296000
//#define SENSOR_LINK_FREQ 110592000
#define SENSOR_PCLK_RATE 150000000 //74250000 //46080000
#define SENSOR_LINK_FREQ 300000000

static const s64 hdzero_link_freq_menu[] = {
	SENSOR_LINK_FREQ,
};

struct hdzerocam {
    struct i2c_client	*client;
	struct v4l2_subdev sd;
    struct media_pad	pad;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *pclk_ctrl;
	struct v4l2_ctrl *link_freq;
	struct v4l2_fract frame_rate;
	struct v4l2_mbus_framefmt fmt;
	unsigned ce_pin;
};

static const struct hdzerocam_format {
	u32 mbus_code;
	enum v4l2_colorspace colorspace;
} hdzerocam_formats[] = {
	{
		.mbus_code      = MEDIA_BUS_FMT_UYVY8_1X16,  // Corresponds to PDFORMAT_YUV422_8BIT and PDATAF_MODE1. Change to 2x8 for 8 bit bus (PDFATAF_MODE0
		.colorspace     = V4L2_COLORSPACE_DEFAULT,
	},
};

static const struct v4l2_mbus_framefmt hdzerocam_default_fmt = {
	.width = 1280,
	.height = 720,
	.code = MEDIA_BUS_FMT_UYVY8_1X16,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_DEFAULT,
};

static inline struct hdzerocam *to_hdzerocam(struct v4l2_subdev *sd)
{
	return container_of(sd, struct hdzerocam, sd);
}
static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct hdzerocam, hdl)->sd;
}

///////////////////////////////// hdzero based stuff /////////////////////////////////////////////////////////////

#define RUNCAM_MICRO_V1 0x42
#define RUNCAM_MICRO_V2 0x44
#define RUNCAM_NANO_90  0x46

#define CAMERA_SETTING_NUM 16
#define ACTIVE_PROFILE 3 // 2 seems to be HV Flip. 3 is called default

typedef enum {
    CAMERA_TYPE_UNKNOW,
    CAMERA_TYPE_RESERVED,        // include foxeer digisight v3
    CAMERA_TYPE_OUTDATED,        // include runcam(orange)
    CAMERA_TYPE_RUNCAM_MICRO_V1, // include hdz nano v1
    CAMERA_TYPE_RUNCAM_MICRO_V2, // include hzd nano v2 / hdz nano lite
    CAMERA_TYPE_RUNCAM_NANO_90,
} camera_type_e;

const uint8_t runcam_micro_v1_attribute[CAMERA_SETTING_NUM][4] = {
    // brightness
    {1, 0x40, 0xC0, 0x80},
    // sharpness
    {1, 0x00, 0x02, 0x01},
    // contrast
    {1, 0x00, 0x02, 0x01},
    // saturation
    {1, 0x00, 0x06, 0x03},
    // shutter speed
    {0, 0x00, 0x20, 0x00},
    // wb mode
    {1, 0x00, 0x01, 0x00},
    // wb red
    {1, 0x00, 0xff, 0xc7},
    // wb blue
    {1, 0x00, 0xff, 0xca},
    // hv flip
    {0, 0x00, 0x01, 0x00},
    // night mode
    {0, 0x00, 0x01, 0x01},
    // led mode
    {1, 0x00, 0x01, 0x00},
    // video fmt
    {0, 0x00, 0x00, 0x00},

    {0, 0x00, 0x00, 0x00},
    {0, 0x00, 0x00, 0x00},
    {0, 0x00, 0x00, 0x00},
    {0, 0x00, 0x00, 0x00},
};

const uint8_t runcam_micro_v2_attribute[CAMERA_SETTING_NUM][4] = {
    // brightness
    {1, 0x40, 0xC0, 0x80},
    // sharpness
    {1, 0x00, 0x02, 0x01},
    // contrast
    {1, 0x00, 0x02, 0x01},
    // saturation
    {1, 0x00, 0x06, 0x05},
    // shutter speed
    {1, 0x00, 0x20, 0x00},
    // wb mode
    {1, 0x00, 0x01, 0x00},
    // wb red
    {1, 0x00, 0xff, 0xc7},
    // wb blue
    {1, 0x00, 0xff, 0xca},
    // hv flip
    {1, 0x00, 0x01, 0x00},
    // night mode
    {1, 0x00, 0x01, 0x01},
    // led mode
    {1, 0x00, 0x01, 0x00},
    // video fmt
    {1, 0x00, 0x03, 0x00},

    {0, 0x00, 0x00, 0x00},
    {0, 0x00, 0x00, 0x00},
    {0, 0x00, 0x00, 0x00},
    {0, 0x00, 0x00, 0x00},
};

const uint8_t runcam_nano_90_attribute[CAMERA_SETTING_NUM][4] = {
    // brightness
    {1, 0x40, 0xC0, 0x80},
    // sharpness
    {1, 0x00, 0x02, 0x01},
    // contrast
    {1, 0x00, 0x02, 0x01},
    // saturation
    {1, 0x00, 0x06, 0x05},
    // shutter speed
    {1, 0x00, 0x20, 0x00},
    // wb mode
    {1, 0x00, 0x01, 0x00},
    // wb red
    {1, 0x00, 0xff, 0xc7},
    // wb blue
    {1, 0x00, 0xff, 0xca},
    // hv flip
    {1, 0x00, 0x01, 0x00},
    // night mode
    {1, 0x00, 0x01, 0x01},
    // led mode
    {1, 0x00, 0x01, 0x00},
    // video fmt
    {1, 0x00, 0x03, 0x00},

    {0, 0x00, 0x00, 0x00},
    {0, 0x00, 0x00, 0x00},
    {0, 0x00, 0x00, 0x00},
    {0, 0x00, 0x00, 0x00},
};

camera_type_e camera_type = CAMERA_TYPE_UNKNOW;


/////////////////////////////////////////////////////////////////
// runcam I2C
uint8_t RUNCAM_Write(struct v4l2_subdev *sd, uint32_t addr, uint32_t val) {
    uint8_t value;
    uint8_t buf[8] = {0};
    uint8_t ret;
    struct hdzerocam *hdzero= to_hdzerocam(sd);
	struct i2c_client *client = hdzero->client;
    
    buf[0] = 0x12; // write cmd

    buf[1] = (addr >> 16) & 0xFF; // ADDR[23:16]
    buf[2] = (addr >> 8) & 0xFF; // ADDR[15:8]
    buf[3] = value = addr & 0xFF; // ADDR[7:0]
    buf[4] = (val >> 24) & 0xFF; // DATA[31:24]
    buf[5] = (val >> 16) & 0xFF; // DATA[23:16]
    buf[6] = (val >> 8) & 0xFF; // DATA[15:8]
    buf[7] = value = val & 0xFF; // DATA[7:0]
    
    
    if(i2c_master_send(client, buf, 8) > 0)
    {
        ret = 0;
    }
    else
    {
        ret = 1;
        printk(KERN_ERR "%s(%d): HDZero ERROR on I2C Send!\n", __func__, __LINE__);
    }
#ifdef _DEBUG_RUNCAM
    debugf("\r\nRUNCAM_Write: %d, %d, %d", (uint16_t)cam_id, (uint16_t)addr, (uint16_t)val);
#else
    msleep(10);
#endif

    return ret;
}

uint32_t RUNCAM_Read(struct v4l2_subdev *sd, uint32_t addr) {
    uint32_t ret = 0;
    uint8_t buf[4] = {0};
    uint8_t buf2[4] = {0};

    struct hdzerocam *hdzero= to_hdzerocam(sd);
	struct i2c_client *client = hdzero->client;
    
	struct i2c_msg msgs[2];
    buf[0] = 0x13; // read cmd

    buf[1] = (addr >> 16) & 0xFF; // ADDR[23:16]
    buf[2] = (addr >> 8) & 0xFF; // ADDR[15:8]
    buf[3] = addr & 0xFF; // ADDR[7:0]


	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 4;
	msgs[0].buf = (u8 *)buf;

	/* Read data from register */
	msgs[1].addr = client->addr | 0x01;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 4;
	msgs[1].buf = buf2;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
    {
        printk(KERN_ERR "%s(%d): HDZero could not read register!\n", __func__, __LINE__);
		return 0;
    }
    ret = be32_to_cpu(buf2);
#ifdef _DEBUG_RUNCAM
    debugf("\r\nRUNCAM_Read: %d, %d: %d", (uint16_t)cam_id, (uint16_t)addr, (uint16_t)ret);
#else
    msleep(10);
#endif

    return ret;
}


void runcam_brightness(struct v4l2_subdev *camera_device, uint8_t val, uint8_t led_mode) {
    uint32_t d;
    uint32_t val_32;

   // camera_setting_reg_set[0] = val;
   // camera_setting_reg_set[10] = led_mode;
    printk(KERN_ERR "%s(%d): HDZero \n", __func__, __LINE__);
    if (camera_type == CAMERA_TYPE_RUNCAM_MICRO_V1)
        d = 0x0452004e;
    else if (camera_type == CAMERA_TYPE_RUNCAM_MICRO_V2)
        d = 0x04500050;
    else // if (camera_type == CAMERA_TYPE_RUNCAM_NANO_90)
        d = 0x04480048;

    if (led_mode)
        d |= 0x00002800;
    else
        d |= 0x00004800;

    val_32 = (uint32_t)val;

    // indoor
    d += val_32;

    d -= runcam_micro_v1_attribute[0][ACTIVE_PROFILE];
    // outdoor
    d += (val_32 << 16);
    d -= ((uint32_t)runcam_micro_v1_attribute[0][ACTIVE_PROFILE] << 16);

    RUNCAM_Write(camera_device, 0x50, d);
#ifdef _DEBUG_RUNCAM
    debugf("\r\nRUNCAM brightness:0x%02x, led_mode:%02x", (uint16_t)val, (uint16_t)led_mode);
#endif
}

void runcam_sharpness(struct v4l2_subdev *camera_device, uint8_t val) {
    uint32_t d;

    //camera_setting_reg_set[1] = val;
    printk(KERN_ERR "%s(%d): HDZero \n", __func__, __LINE__);
    if (val == 0) {
        if (camera_type == RUNCAM_MICRO_V1)
            d = 0x03FF0100;
        else // if (camera_type == RUNCAM_MICRO_V2 || camera_type == RUNCAM_NANO_90)
            d = 0x03FF0000;
        RUNCAM_Write(camera_device, 0x0003C4, d);
        RUNCAM_Write(camera_device, 0x0003CC, 0x0A0C0E10);
        RUNCAM_Write(camera_device, 0x0003D8, 0x0A0C0E10);
    } else if (val == 1) {
        RUNCAM_Write(camera_device, 0x0003C4, 0x03FF0000);
        RUNCAM_Write(camera_device, 0x0003CC, 0x14181C20);
        RUNCAM_Write(camera_device, 0x0003D8, 0x14181C20);
    } else if (val == 2) {
        RUNCAM_Write(camera_device, 0x0003C4, 0x03FF0000);
        RUNCAM_Write(camera_device, 0x0003CC, 0x28303840);
        RUNCAM_Write(camera_device, 0x0003D8, 0x28303840);
    }
#ifdef _DEBUG_RUNCAM
    debugf("\r\nRUNCAM sharpness:0x%02x", (uint16_t)val);
#endif
}

void runcam_contrast(struct v4l2_subdev *camera_device, uint8_t val) {
    uint32_t d;

    //camera_setting_reg_set[2] = val;
    printk(KERN_ERR "%s(%d): HDZero \n", __func__, __LINE__);
    if (camera_type == RUNCAM_MICRO_V1)
        d = 0x46484A4C;
    else // if (camera_type == RUNCAM_MICRO_V2 || camera_type == RUNCAM_NANO_90)
        d = 0x36383a3c;

    if (val == 0) // low
        d -= 0x06040404;
    else if (val == 1) // normal
        ;
    else if (val == 2) // high
        d += 0x04040404;

    RUNCAM_Write(camera_device, 0x00038C, d);
#ifdef _DEBUG_RUNCAM
    debugf("\r\nRUNCAM contrast:%02x", (uint16_t)val);
#endif
}

uint8_t runcam_saturation(struct v4l2_subdev *camera_device, uint8_t val) {
    uint8_t ret = 1;
    uint32_t d;
    printk(KERN_ERR "%s(%d): HDZero \n", __func__, __LINE__);
    //camera_setting_reg_set[3] = val;

    // initial
    if (camera_type == RUNCAM_MICRO_V1)
        d = 0x20242626;
    else // if (camera_type == RUNCAM_MICRO_V2 || camera_type == RUNCAM_NANO_90)
        d = 0x24282c30;

    if (val == 0)
        d = 0x00000000;
    else if (val == 1)
        d -= 0x18181414;
    else if (val == 2)
        d -= 0x14141010;
    else if (val == 3)
        d -= 0x0c0c0808;
    else if (val == 4)
        d -= 0x08080404;
    else if (val == 5)
        d += 0x00000000;
    else if (val == 6)
        d += 0x04041418;

    ret = RUNCAM_Write(camera_device, 0x0003A4, d);

#ifdef _DEBUG_RUNCAM
    debugf("\r\nRUNCAM saturation:%02x", (uint16_t)val);
#endif
    return ret;
}

void runcam_wb(struct v4l2_subdev *camera_device, uint8_t wbMode, uint8_t wbRed, uint8_t wbBlue) {
    uint32_t wbRed_u32 = 0x02000000;
    uint32_t wbBlue_u32 = 0x00000000;
    printk(KERN_ERR "%s(%d): HDZero \n", __func__, __LINE__);
    //camera_setting_reg_set[5] = wbMode;
    //camera_setting_reg_set[6] = wbRed;
    //camera_setting_reg_set[7] = wbBlue;

    if (wbMode) {
        wbRed_u32 += ((uint32_t)wbRed << 2);
        wbBlue_u32 += ((uint32_t)wbBlue << 2);
        wbRed_u32++;
        wbBlue_u32++;
    }

    if (wbMode) { // MWB
        RUNCAM_Write(camera_device, 0x0001b8, 0x020b007b);
        RUNCAM_Write(camera_device, 0x000204, wbRed_u32);
        RUNCAM_Write(camera_device, 0x000208, wbBlue_u32);
    } else { // AWB
        RUNCAM_Write(camera_device, 0x0001b8, 0x020b0079);
    }
#ifdef _DEBUG_RUNCAM
    debugf("\r\nRUNCAM wb:red(%02x),blue(%02x),mode(%02x)",
           (uint16_t)wbRed, (uint16_t)wbBlue, (uint16_t)wbMode);
#endif
}

void runcam_hv_flip(struct v4l2_subdev *camera_device, uint8_t val) {
    if (camera_type != CAMERA_TYPE_RUNCAM_MICRO_V2 && camera_type != CAMERA_TYPE_RUNCAM_NANO_90)
        return;
    printk(KERN_ERR "%s(%d): HDZero \n", __func__, __LINE__);
    //camera_setting_reg_set[8] = val;

    if (val == 0)
        RUNCAM_Write(camera_device, 0x000040, 0x0022ffa9);
    else if (val == 1)
        RUNCAM_Write(camera_device, 0x000040, 0x002effa9);
#ifdef _DEBUG_RUNCAM
    debugf("\r\nRUNCAM hvFlip:%02x", (uint16_t)val);
#endif
}

void runcam_night_mode(struct v4l2_subdev *camera_device, uint8_t val) {
    /*
        0: night mode off
        1: night mode on
    */
    if (camera_type != CAMERA_TYPE_RUNCAM_MICRO_V2 && camera_type != CAMERA_TYPE_RUNCAM_NANO_90)
        return;
    printk(KERN_ERR "%s(%d): HDZero \n", __func__, __LINE__);
    //camera_setting_reg_set[9] = val;

    if (val == 0) { // Max gain off
        RUNCAM_Write(camera_device, 0x000070, 0x10000040);
        RUNCAM_Write(camera_device, 0x000718, 0x30002900);
        RUNCAM_Write(camera_device, 0x00071c, 0x32003100);
        RUNCAM_Write(camera_device, 0x000720, 0x34003300);
    } else if (val == 1) { // Max gain on
        RUNCAM_Write(camera_device, 0x000070, 0x10000040);
        RUNCAM_Write(camera_device, 0x000718, 0x28002700);
        RUNCAM_Write(camera_device, 0x00071c, 0x29002800);
        RUNCAM_Write(camera_device, 0x000720, 0x29002900);
    }
#ifdef _DEBUG_RUNCAM
    debugf("\r\nRUNCAM nightMode:%02x", (uint16_t)val);
#endif
}

void runcam_video_format(struct v4l2_subdev *camera_device, uint8_t val) {
    /*
    RUNCAM_MICRO_V2:
        0: 1280x720@60 4:3
        1: 1280x720@60 16:9 crop
        2: 1280x720@60 16:9 full
        3: 1920x1080@30
    RUNCAM_NANO_90:
        0: 720x540@90 4:3
        1: 720x540@90 4:3 crop // NANO 90 DEMO CAMERA DOESN"T SUPPORT
        2: 720x540@60 4:3
        3: 960x720@60 4:3
    */
    //camera_setting_reg_set[11] = val;
    printk(KERN_ERR "%s(%d): HDZero \n", __func__, __LINE__);
    if (camera_type == CAMERA_TYPE_RUNCAM_MICRO_V1)
        return;
    else if (camera_type == CAMERA_TYPE_RUNCAM_MICRO_V2) {
        if (val == 0)
            RUNCAM_Write(camera_device, 0x000008, 0x0008910B);
        else if (val == 1)
            RUNCAM_Write(camera_device, 0x000008, 0x00089102);
        else if (val == 2)
            RUNCAM_Write(camera_device, 0x000008, 0x00089110);
        else if (val == 3) // 1080p30
            RUNCAM_Write(camera_device, 0x000008, 0x81089106);

        if (val == 3) // 1080p30
            RUNCAM_Write(camera_device, 0x000034, 0x00014441);
        else
            RUNCAM_Write(camera_device, 0x000034, 0x00012941);

    } else if (camera_type == CAMERA_TYPE_RUNCAM_NANO_90) {
        if (val == 0)
            RUNCAM_Write(camera_device, 0x000008, 0x8008811d);
        else if (val == 1)
            RUNCAM_Write(camera_device, 0x000008, 0x83088120);
        else if (val == 2)
            RUNCAM_Write(camera_device, 0x000008, 0x8108811e);
        else if (val == 3)
            RUNCAM_Write(camera_device, 0x000008, 0x8208811f);
    }
#ifdef _DEBUG_RUNCAM
    debugf("\r\nRUNCAM video format:%02x", (uint16_t)val);
#endif
}

void runcam_shutter(struct v4l2_subdev *camera_device, uint8_t val) {
    uint32_t dat = 0;

#ifdef _DEBUG_RUNCAM
    debugf("\r\nRUNCAM shutter:%02x", (uint16_t)val);
#endif
    printk(KERN_ERR "%s(%d): HDZero \n", __func__, __LINE__);
    //camera_setting_reg_set[4] = val;
    if (camera_type == CAMERA_TYPE_RUNCAM_MICRO_V1) {
        RUNCAM_Write(camera_device, 0x00006c, 0x000004a6);
        msleep(50);
        RUNCAM_Write(camera_device, 0x000044, 0x80019229);
        msleep(50);
        return;
    } else {
        if (val == 0) { // auto
            if (camera_type == CAMERA_TYPE_RUNCAM_MICRO_V2)
                dat = 0x460;
            else if (camera_type == CAMERA_TYPE_RUNCAM_NANO_90)
                dat = 0x447;
        } else // manual
            dat = (uint32_t)(val)*25;

        RUNCAM_Write(camera_device, 0x00006c, dat);
        msleep(50);
        RUNCAM_Write(camera_device, 0x000044, 0x80009629);
        msleep(50);
    }
}

void runcam_type_detect(struct v4l2_subdev *camera_device) {
    //uint8_t i, j;
    if (!RUNCAM_Write(camera_device, 0x50, 0x0452484E)) {
        camera_type = CAMERA_TYPE_RUNCAM_MICRO_V1;
        //camera_device_type = RUNCAM_MICRO_V1;
        printk(KERN_ERR "%s(%d): HDZero Detected Runcam Micro V1!\n", __func__, __LINE__);
        #if 0
        
        for (i = 0; i < CAMERA_SETTING_NUM; i++) {
            for (j = 0; j < 4; j++)
                camera_attribute[i][j] = runcam_micro_v1_attribute[i][j];
        }
        #endif
    } 
#if 0
// FIXME as detect is in the dts we may not be able to do this??    
    else if (!RUNCAM_Write(RUNCAM_MICRO_V2, 0x50, 0x0452484E)) {
        camera_type = CAMERA_TYPE_RUNCAM_MICRO_V2;
        camera_device = RUNCAM_MICRO_V2;
        
        for (i = 0; i < CAMERA_SETTING_NUM; i++) {
            for (j = 0; j < 4; j++)
                camera_attribute[i][j] = runcam_micro_v2_attribute[i][j];
        }
    } else if (!RUNCAM_Write(RUNCAM_NANO_90, 0x50, 0x04484848)) {
        camera_type = CAMERA_TYPE_RUNCAM_NANO_90;
        camera_device = RUNCAM_NANO_90;
        for (i = 0; i < CAMERA_SETTING_NUM; i++) {
            for (j = 0; j < 4; j++)
                camera_attribute[i][j] = runcam_nano_90_attribute[i][j];
        }
    }
#endif
    else
    {
        printk(KERN_ERR "%s(%d): HDZero Could not find camera!\n", __func__, __LINE__);
    }
}

void camera_settings_set(struct v4l2_subdev *camera_device)
{
    printk(KERN_ERR "%s(%d): HDZero\n", __func__, __LINE__);
    runcam_brightness(camera_device, runcam_micro_v1_attribute[ACTIVE_PROFILE][0], runcam_micro_v1_attribute[ACTIVE_PROFILE][10]); // include led_mode
    runcam_sharpness(camera_device, runcam_micro_v1_attribute[ACTIVE_PROFILE][1]);
    runcam_contrast(camera_device, runcam_micro_v1_attribute[ACTIVE_PROFILE][2]);
    runcam_saturation(camera_device, runcam_micro_v1_attribute[ACTIVE_PROFILE][3]);
    runcam_shutter(camera_device, runcam_micro_v1_attribute[ACTIVE_PROFILE][4]);
    runcam_wb(camera_device, runcam_micro_v1_attribute[ACTIVE_PROFILE][5], runcam_micro_v1_attribute[ACTIVE_PROFILE][6], runcam_micro_v1_attribute[ACTIVE_PROFILE][7]);
    runcam_hv_flip(camera_device, runcam_micro_v1_attribute[ACTIVE_PROFILE][8]);
    runcam_night_mode(camera_device, runcam_micro_v1_attribute[ACTIVE_PROFILE][9]);
    runcam_video_format(camera_device, runcam_micro_v1_attribute[ACTIVE_PROFILE][11]);
}

void camera_init(struct v4l2_subdev *camera_device) {
    printk(KERN_ERR "%s(%d): HDZero\n", __func__, __LINE__);
    //runcam_type_detect(camera_device); // This just wraps runcam_type_detect(). Turns out all cameras are runcams or compatible (Foxeer isn't very clear). Then it just polls i2c to see what responds. That magic number is pretty helpful
    //camera_settings_set(camera_device); // If camera is different from last time, set camera_profile_eep to 0 and write to eeprom. Then initialise default values from the top of runcam.c into the flash memory.

    //RUNCAM_Write(camera_device, 0x000694, 0x00000130); // Reset the ISP
}

////////////////////////////////////// v4l2 stuff /////////////////////////////////////////////////////////////////////////////////////////
static int hdzerocam_s_ctrl(struct v4l2_ctrl *ctrl)
{
	//struct v4l2_subdev *sd = to_sd(ctrl);

	switch (ctrl->id) {
#if 0
        case V4L2_CID_CONTRAST:
		
		break;
	case V4L2_CID_SATURATION:
		
        break;
	case V4L2_CID_HFLIP:
		
        break;
	case V4L2_CID_VFLIP:
		
        break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

static int hdzerocam_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(hdzerocam_formats))
    {
        
        printk(KERN_ERR "%s(%d): No mbus code\n", __func__, __LINE__);
		return -EINVAL;
    }
    
    printk(KERN_ERR "%s(%d): Mbus code done\n", __func__, __LINE__);
	code->code = hdzerocam_formats[code->index].mbus_code;
	return 0;
}

static int hdzerocam_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct hdzerocam *sensor = to_hdzerocam(sd);
	int index;

    printk(KERN_ERR "%s(%d)\n", __func__, __LINE__);
    
	if (format->pad)
    {
        printk(KERN_ERR "%s(%d): Pad invalid\n", __func__, __LINE__);
		return -EINVAL;
    }

	for (index = 0; index < ARRAY_SIZE(hdzerocam_formats); index++)
		if (hdzerocam_formats[index].mbus_code == fmt->code)
			break;
	if (index >= ARRAY_SIZE(hdzerocam_formats)) {
        
		/* default to first format */
		index = 0;
		fmt->code = hdzerocam_formats[0].mbus_code;
	}

	/* sensor mode is VGA */
	if (fmt->width > MAX_WIDTH)
		fmt->width = MAX_WIDTH;
	if (fmt->height > MAX_HEIGHT)
		fmt->height = MAX_HEIGHT;
	//fmt->width = fmt->width & (~3);
	//fmt->height = fmt->height & (~3);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = hdzerocam_formats[index].colorspace;
    
    printk(KERN_ERR "%s(%d): HDZero set format to %dx%d\n", __func__, __LINE__, fmt->width, fmt->height);
	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
        struct v4l2_mbus_framefmt *try_fmt =
                v4l2_subdev_get_try_format(&sensor->sd, cfg, format->pad);
        if(try_fmt)
        {
            *try_fmt = format->format;
        }
        else
            printk(KERN_ERR "%s(%d): No try fmt!!!!!\n", __func__, __LINE__);
        
        printk(KERN_ERR "%s(%d): Format try\n", __func__, __LINE__);
		return 0;
	}
	sensor->fmt = *fmt;
    
    printk(KERN_ERR "%s(%d): Set format\n", __func__, __LINE__);

	return 0;
}

static int hdzerocam_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct hdzerocam *sensor = to_hdzerocam(sd);

    printk(KERN_ERR "%s(%d)\n", __func__, __LINE__);
	if (format->pad)
    {
        printk(KERN_ERR "%s(%d): Err Invalid pad\n", __func__, __LINE__);
		return -EINVAL;
    }
	format->format = sensor->fmt;
	return 0;
}

static int hdzerocam_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *ival)
{
	struct hdzerocam *sensor = to_hdzerocam(sd);

    printk(KERN_ERR "%s(%d)\n", __func__, __LINE__);
	ival->interval.numerator = sensor->frame_rate.denominator;
	ival->interval.denominator = sensor->frame_rate.numerator;
	return 0;
}

static int hdzerocam_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *ival)
{
	struct hdzerocam *sensor = to_hdzerocam(sd);
	struct v4l2_fract *tpf = &ival->interval;


    printk(KERN_ERR "%s(%d)\n", __func__, __LINE__);
	if (tpf->numerator == 0 || tpf->denominator == 0
		|| (tpf->denominator > tpf->numerator * MAX_FRAME_RATE)) {
		/* reset to max frame rate */
		tpf->numerator = 1;
		tpf->denominator = MAX_FRAME_RATE;
	}
    
    printk(KERN_ERR "%s(%d): HDZero set framerate to %d/%d\n", __func__, __LINE__, tpf->denominator, tpf->numerator);
	sensor->frame_rate.numerator = tpf->denominator;
	sensor->frame_rate.denominator = tpf->numerator;
	return 0;
}

static int hdzerocam_s_stream(struct v4l2_subdev *sd, int enable)
{
    
    printk(KERN_ERR "%s(%d)\n", __func__, __LINE__);
	if (enable)
    {
    }
    else
	{

    }
	udelay(100);
	return 0;
}


static int hdzero_s_power(struct v4l2_subdev *sd, int on)
{
    return 0;
}

static const struct v4l2_ctrl_ops hdzerocam_ctrl_ops = {
	.s_ctrl = hdzerocam_s_ctrl,
};

static const struct v4l2_subdev_core_ops hdzerocam_core_ops = {

    .subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
    .s_power		= hdzero_s_power,
};

static const struct v4l2_subdev_video_ops hdzerocam_video_ops = {
	.s_frame_interval = hdzerocam_s_frame_interval,
	.g_frame_interval = hdzerocam_g_frame_interval,
	.s_stream = hdzerocam_s_stream,
};

static const struct v4l2_subdev_pad_ops hdzerocam_pad_ops = {
	.enum_mbus_code = hdzerocam_enum_mbus_code,
	.get_fmt = hdzerocam_get_fmt,
	.set_fmt = hdzerocam_set_fmt,
};

static const struct v4l2_subdev_ops hdzerocam_ops = {
	.core = &hdzerocam_core_ops,
	.video = &hdzerocam_video_ops,
	.pad = &hdzerocam_pad_ops,
};

static void hdzerocam_fill_fmt(const struct v4l2_mbus_framefmt *mode,
			    struct v4l2_mbus_framefmt *fmt)
{
    printk(KERN_ERR "%s(%d): HDZero fill\n", __func__, __LINE__);
	fmt->code = mode->code;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
}

static int hdzerocam_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *try_fmt;

    printk(KERN_ERR "%s(%d)\n", __func__, __LINE__);
	try_fmt = v4l2_subdev_get_try_format(sd, fh->pad, 0);
	/* Initialize try_fmt */
	hdzerocam_fill_fmt(&hdzerocam_default_fmt, try_fmt);

	return 0;
}

static const struct v4l2_subdev_internal_ops hdzerocam_internal_ops = {
	.open = hdzerocam_open,
};

static int hdzerocam_probe(struct i2c_client *client)
{
	struct hdzerocam *sensor;
	struct v4l2_subdev *sd;
	struct v4l2_ctrl_handler *hdl;
int ret;

    printk(KERN_ERR "%s(%d): HDZero start\n", __func__, __LINE__);
	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk(KERN_ERR "%s(%d): HDZero probe no i2c functionality\n", __func__, __LINE__);
		return -EIO;
    }
    
	/* wait 100ms before any further i2c writes are performed */
	msleep(100);

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
    
    printk(KERN_ERR "%s(%d): HDZero alloc\n", __func__, __LINE__);
    
	if (sensor == NULL)
		return -ENOMEM;

    printk(KERN_ERR "%s(%d)\n", __func__, __LINE__);
	sd = &sensor->sd;
    printk(KERN_ERR "%s(%d) client=%p\n", __func__, __LINE__, client);
	v4l2_i2c_subdev_init(sd, client, &hdzerocam_ops);
    sensor->client = client;
    
	/* set frame rate */
	sensor->frame_rate.numerator = MAX_FRAME_RATE;
	sensor->frame_rate.denominator = 1;

	sensor->fmt = hdzerocam_default_fmt;

    printk(KERN_ERR "%s(%d): HDZero Init camera\n", __func__, __LINE__);
    camera_init(sd);
    
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	sensor->sd.internal_ops = &hdzerocam_internal_ops;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

    sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
    
    printk(KERN_ERR "%s(%d): media pads init\n", __func__, __LINE__);
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret < 0)
    {
        printk(KERN_ERR "%s(%d): media_entity_pads_init failed\n", __func__, __LINE__);
    }
    printk(KERN_ERR "%s(%d): v4l2_async_register_subdev\n", __func__, __LINE__);
    
	ret = v4l2_async_register_subdev_sensor_common(&sensor->sd);
	if (ret) {
        printk(KERN_ERR "%s(%d):v4l2 async register subdev failed\n", __func__, __LINE__);
		return 0;
	}


	hdl = &sensor->hdl;
	v4l2_ctrl_handler_init(hdl, 1);
	    
    sensor->pclk_ctrl = v4l2_ctrl_new_std(hdl,
                      &hdzerocam_ctrl_ops,
                      V4L2_CID_PIXEL_RATE,
                      SENSOR_PCLK_RATE, SENSOR_PCLK_RATE,
                      1, SENSOR_PCLK_RATE);
    if (sensor->pclk_ctrl)
		sensor->pclk_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
        
    sensor->link_freq =
		v4l2_ctrl_new_int_menu(hdl, &hdzerocam_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
                       ARRAY_SIZE(hdzero_link_freq_menu) - 1, 0,
				       hdzero_link_freq_menu);
	if (sensor->link_freq)
		sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
    
    printk(KERN_ERR "%s(%d): HDZero v4l2 controls init\n", __func__, __LINE__);
	/* hook the control handler into the driver */
	sd->ctrl_handler = hdl;
	if (hdl->error) 
    {
    	int err = hdl->error;
        printk(KERN_ERR "%s(%d): Could not init controls\n", __func__, __LINE__);

		v4l2_ctrl_handler_free(hdl);
		return err;
	}
    
    printk(KERN_ERR "%s(%d): HDZero controls setup\n", __func__, __LINE__);
	/* initialize the hardware to the default control values */
	ret = v4l2_ctrl_handler_setup(hdl);
	if (ret)
    {
		v4l2_ctrl_handler_free(hdl);
        
        printk(KERN_ERR "%s(%d): HDZero Control setup failed\n", __func__, __LINE__);
    }
    
    printk(KERN_ERR "%s(%d): HDZero Init complete\n", __func__, __LINE__);
	return ret;
}

static int hdzerocam_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
    return 0;
}


static const struct of_device_id hdzerocam_of_match[] = {
	{.compatible = "hdzerocam"},
	{},
};

static const struct i2c_device_id hdzerocam_id[] = {
	{"hdzerocam", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, hdzerocam_id);

static struct i2c_driver hdzerocam_driver = {
	.driver = {
		.name   = "hdzerocam",
		.of_match_table = hdzerocam_of_match
	},
	.probe_new      = hdzerocam_probe,
	.remove         = hdzerocam_remove,
	.id_table       = hdzerocam_id,
};

//module_i2c_driver(hdzerocam_driver);

static int __init sensor_mod_init(void)
{
    printk(KERN_ERR "%s(%d)\n", __func__, __LINE__);
	return i2c_add_driver(&hdzerocam_driver);
}

static void __exit sensor_mod_exit(void)
{
    printk(KERN_ERR "%s(%d)\n", __func__, __LINE__);
	i2c_del_driver(&hdzerocam_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("hdzerocam sensor driver");
MODULE_AUTHOR("Pete Allen <peter.allenm@gmail.com>");
MODULE_LICENSE("GPL v2");
