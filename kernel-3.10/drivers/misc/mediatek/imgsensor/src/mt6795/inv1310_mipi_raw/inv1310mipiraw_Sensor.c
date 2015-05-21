/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 INV1310mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "inv1310mipiraw_Sensor.h"



/****************************Modify following Strings for debug****************************/
#define PFX "INV1310_camera_sensor"

#define LOG_1 LOG_INF("INV1310,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 1920*1080@25fps,88MPix/s; video 1920*1080@25fps,88Mpix/s; capture 12M@4.5fps, 67Mpix/s\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)    xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

#define MIPI_SETTLEDELAY_AUTO     0
#define MIPI_SETTLEDELAY_MANNUAL  1
//#define IHDR_USED

static imgsensor_info_struct imgsensor_info = {
    .sensor_id = 0x1310, //hzmadd: for sensor id matching
    
    //.checksum_value = 0x215125a0,
    .checksum_value = 0x4ff3b7e6,
    
    .pre = { //hzmadd:
        .pclk = 88000000,              //record different mode's pclk
        .linelength = 3000,             //record different mode's linelength
        .framelength = 1173,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 1920,       //record different mode's width of grabwindow
        .grabwindow_height = 1080,      //record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 30,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 250,
    },
    .cap = {   // hzmadd 25  fps  capture
        .pclk = 67000000,
        .linelength = 4848,
        .framelength = 3069,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4000,
        .grabwindow_height = 3000,
        .mipi_data_lp2hs_settle_dc = 30,
        .max_framerate = 45,
    },
    .cap1 = {    // 24 fps  capture
        .pclk = 348000000,
        .linelength = 4572,
        .framelength = 3146,
        .startx = 4,
        .starty = 4,
        .grabwindow_width = 4176,//4192,
        .grabwindow_height = 3088,//3104,
        .mipi_data_lp2hs_settle_dc = 30,
        .max_framerate = 240,
    },
    .cap2 = {    //15 fps  capture
        .pclk = 231000000,
        .linelength = 4572,
        .framelength = 3146,
        .startx = 4,
        .starty = 4,
        .grabwindow_width = 4176,//4192,
        .grabwindow_height = 3088,//3104,
        .mipi_data_lp2hs_settle_dc = 30,
        .max_framerate = 150,
    },
    .normal_video = { // hzmadd
        .pclk = 88000000,
        .linelength = 3000,
        .framelength = 1173,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1920,
        .grabwindow_height = 1080,
        .mipi_data_lp2hs_settle_dc = 30,
        .max_framerate = 250,
    },
    .hs_video = {     // 120 fps
        .pclk = 432000000,
        .linelength = 4572,
        .framelength = 786,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 30,
        .max_framerate = 1200,
    },
    .slim_video = {
        .pclk = 184000000,//231270000,
        .linelength = 4572,
        .framelength = 1312,//1640,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 30,
        .max_framerate = 300,
    },
    .custom1 = {
        .pclk = 231270000,              //record different mode's pclk
        .linelength = 4572,             //record different mode's linelength
        .framelength = 1640,            //record different mode's framelength
        .startx = 2,                    //record different mode's startx of grabwindow
        .starty = 2,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2088,//2096,        //record different mode's width of grabwindow
        .grabwindow_height = 1544,//1552,       //record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 30,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 300,
        
    },
    .custom2 = {
        .pclk = 231270000,              //record different mode's pclk
        .linelength = 4572,             //record different mode's linelength
        .framelength = 1640,            //record different mode's framelength
        .startx = 2,                    //record different mode's startx of grabwindow
        .starty = 2,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2088,//2096,        //record different mode's width of grabwindow
        .grabwindow_height = 1544,//1552,       //record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 30,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 300,
        
    },
    .custom3 = {
        .pclk = 231270000,              //record different mode's pclk
        .linelength = 4572,             //record different mode's linelength
        .framelength = 1640,            //record different mode's framelength
        .startx = 2,                    //record different mode's startx of grabwindow
        .starty = 2,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2088,//2096,        //record different mode's width of grabwindow
        .grabwindow_height = 1544,//1552,       //record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 30,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 300,
        
    },
    .custom4 = {
        .pclk = 231270000,              //record different mode's pclk
        .linelength = 4572,             //record different mode's linelength
        .framelength = 1640,            //record different mode's framelength
        .startx = 2,                    //record different mode's startx of grabwindow
        .starty = 2,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2088,//2096,        //record different mode's width of grabwindow
        .grabwindow_height = 1544,//1552,       //record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 30,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 300,
        
    },
    .custom5 = {
        .pclk = 231270000,              //record different mode's pclk
        .linelength = 4572,             //record different mode's linelength
        .framelength = 1640,            //record different mode's framelength
        .startx = 2,                    //record different mode's startx of grabwindow
        .starty = 2,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2088,//2096,        //record different mode's width of grabwindow
        .grabwindow_height = 1544,//1552,       //record different mode's height of grabwindow
        /*   following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario   */
        .mipi_data_lp2hs_settle_dc = 30,
        /*   following for GetDefaultFramerateByScenario()  */
        .max_framerate = 300,
        
    },
    .margin = 10,
    .min_shutter = 1,
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 1,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,
    .ihdr_support = 0,    //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
    .sensor_mode_num = 10,   //support sensor mode num
    
    .cap_delay_frame = 2,
    .pre_delay_frame = 2,
    .video_delay_frame = 5,
    .hs_video_delay_frame = 5,
    .slim_video_delay_frame = 5,
    .custom1_delay_frame = 2,
    .custom2_delay_frame = 2,
    .custom3_delay_frame = 2,
    .custom4_delay_frame = 2,
    .custom5_delay_frame = 2,
    
    .isp_driving_current = ISP_DRIVING_2MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANNUAL,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
    .mclk = 24,
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    .i2c_addr_table = {0x20, 0xff}, //hzmadd: add i2c addr (0x10 write/0x11 read)
    .i2c_speed = 300, // i2c read/write speed
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,             //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3D0,                   //current shutter   // Danbo ??
    .gain = 0x100,                      //current gain     // Danbo ??
    .dummy_pixel = 0,                   //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 250,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,      //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x20,
    .update_sensor_otp_awb = 0,
    .update_sensor_otp_lsc = 0,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] =
{
    { 4224, 3008, 1152, 964, 1920, 1080, 1920, 1080, 0000, 0000, 1920, 1080, 0000, 0000, 1920, 1080},  // hzmadd Preview 1920*1080
    { 4224, 3008, 112, 4, 4000, 3000, 4000, 3000, 0000, 0000, 4000, 3000, 0000, 0000, 4000, 3000},  // hzmadd capture 1920*1080
    { 4224, 3008, 1152, 964, 1920, 1080, 1920, 1080, 0000, 0000, 1920, 1080, 0000, 0000, 1920, 1080},  // hzmadd video 1920*1080
    { 4208, 3120,  824,  840, 2560, 1440, 1280,  720, 0000, 0000, 1280,  720, 0000, 0000, 1280,  720}, //hight speed video
    { 4208, 3120,  824,  840, 2560, 1440, 1280,  720, 0000, 0000, 1280,  720, 0000, 0000, 1280,  720}, // slim video
    { 4208, 3120, 0000, 0000, 4208, 3120, 2104, 1560, 0000, 0000, 2104, 1560, 0007, 0007, 2088, 1544}, // Custom1 (default use preview)
    { 4208, 3120, 0000, 0000, 4208, 3120, 2104, 1560, 0000, 0000, 2104, 1560, 0007, 0007, 2088, 1544}, // Custom2
    { 4208, 3120, 0000, 0000, 4208, 3120, 2104, 1560, 0000, 0000, 2104, 1560, 0007, 0007, 2088, 1544}, // Custom3
    { 4208, 3120, 0000, 0000, 4208, 3120, 2104, 1560, 0000, 0000, 2104, 1560, 0007, 0007, 2088, 1544}, // Custom4
    { 4208, 3120, 0000, 0000, 4208, 3120, 2104, 1560, 0000, 0000, 2104, 1560, 0007, 0007, 2088, 1544}, // Custom5
};// slim video
//#define INV1310_OTP_Enable 1

// Gain Index
#define MaxGainIndex (71)
static kal_uint16 sensorGainMapping[MaxGainIndex][2] ={
    {71  ,25 },
    {76  ,42 },
    {83  ,59 },
    {89  ,73 },
    {96  ,85 },
    {102 ,96 },
    {108 ,105},
    {115 ,114},
    {121 ,121},
    {128 ,128},
    {134 ,134},
    {140 ,140},
    {147 ,145},
    {153 ,149},
    {160 ,154},
    {166 ,158},
    {172 ,161},
    {179 ,164},
    {185 ,168},
    {192 ,171},
    {200 ,174},
    {208 ,177},
    {216 ,180},
    {224 ,183},
    {232 ,185},
    {240 ,188},
    {248 ,190},
    {256 ,192},
    {264 ,194},
    {272 ,196},
    {280 ,197},
    {288 ,199},
    {296 ,201},
    {304 ,202},
    {312 ,203},
    {320 ,205},
    {328 ,206},
    {336 ,207},
    {344 ,208},
    {352 ,209},
    {360 ,210},
    {368 ,211},
    {376 ,212},
    {384 ,213},
    {390 ,214},
    {399 ,215},
    {409 ,216},
    {419 ,217},
    {431 ,218},
    {442 ,219},
    {455 ,220},
    {467 ,221},
    {481 ,222},
    {496 ,223},
    {512 ,224},
    {528 ,225},
    {545 ,226},
    {565 ,227},
    {584 ,228},
    {606 ,229},
    {630 ,230},
    {655 ,231},
    {682 ,232},
    {712 ,233},
    {744 ,234},
    {780 ,235},
    {819 ,236},
    {862 ,237},
    {910 ,238},
    {963 ,239},
    {1024,240}
};

extern bool inv_opt_update(BYTE update_sensor_otp_awb, BYTE update_sensor_otp_lsc);
extern void inv_opt_clear_flag(void);

//#if INV1310_OTP_Enable
void write_inv_sensor_16(kal_uint16 addr, kal_uint16 para)
{
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}
//#endif

kal_uint16 read_inv_sensor(kal_uint32 addr)
{
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    
    kal_uint16 get_byte=0;
    
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
    
    return get_byte;
}

void write_inv_sensor(kal_uint32 addr, kal_uint32 para)
{
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

kal_uint16 read_inv_sensor_8(kal_uint16 addr)
{
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}


void write_inv_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    kdSetI2CSpeed(imgsensor_info.i2c_speed); // Add this func to set i2c speed by each sensor
    
    char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy()
{
    LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    
    write_inv_sensor_8(0x3002, 0x01);
    write_inv_sensor_16(0x3200, imgsensor.frame_length);
    write_inv_sensor_16(0x3202, imgsensor.line_length);
    write_inv_sensor_8(0x3002, 0x00);
    
}   /*  set_dummy  */

static kal_uint32 return_sensor_id()
{
    return ((read_inv_sensor_8(0x3010) << 8) | read_inv_sensor_8(0x3011)); //hzmadd: for sensor id matching
}

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;
    
    LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate,min_framelength_en);
    
    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    //dummy_line = frame_length - imgsensor.min_frame_length;
    //if (dummy_line < 0)
    //imgsensor.dummy_line = 0;
    //else
    //imgsensor.dummy_line = dummy_line;
    //imgsensor.frame_length = frame_length + imgsensor.dummy_line;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
    {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    //set_dummy();
}   /*  set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
{
    kal_uint16 realtime_fps = 0;
    kal_uint32 frame_length = 0;
    
    /* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
    // if shutter bigger than frame_length, should extend frame length first
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
    
    // Framelength should be an even number
    shutter = (shutter >> 1) << 1;
    imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
    
    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
        {
            set_max_framerate(296,0);
        }
        else if(realtime_fps >= 147 && realtime_fps <= 150)
        {
            set_max_framerate(146,0);
        }
    }

    // update frame length, line length and shutter
    write_inv_sensor_8(0x3002, 0x01);
    write_inv_sensor_16(0x3200, imgsensor.frame_length);
    write_inv_sensor_16(0x3202, imgsensor.line_length);
    write_inv_sensor_16(0x3204, shutter);
    write_inv_sensor_8(0x3002, 0x00);
    LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    LOG_INF("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);
    LOG_INF("frame_length = %d ", frame_length);
}   /*  write_shutter  */


/*************************************************************************
 * FUNCTION
 *   set_shutter
 *
 * DESCRIPTION
 *   This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *   iShutter : exposured lines
 *
 * RETURNS
 *   None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
    unsigned long flags;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
    
    write_shutter(shutter);
}   /*  set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
    //dingej1 2015.05.09 begin
    /*
     kal_uint8 iI;
     for (iI = 0; iI < (MaxGainIndex-1); iI++) {
        if(gain <= sensorGainMapping[iI][0]){
            break;
        }
     }
     */
    /*
     if(gain != sensorGainMapping[iI][0])
     {
        //SENSORDB("Gain mapping don't correctly:%d %d \n", gain, sensorGainMapping[iI][0]);
        return sensorGainMapping[iI][1];
     }
     else return (kal_uint16)gain;
     */

    //dingej1 2015.05.09 begin
    /*
        return sensorGainMapping[iI][1];
     */
    //ref :case CMD_SENSOR_SET_ESHUTTER_GAIN:
    //mtk_6595/vendor/mediatek/proprietary/platform/mt6795/hardware/mtkcam/hal/sensor/imgsensor_drv.cpp
    /*
     I do not know which formula the invisage use about gain.
     So I don't know the relation is linear or nonlinear.
     So the draft just use linear relation.
     */
    
    kal_uint16 iReg = 0x00;
    iReg = gain << 1;
    return iReg;
    //dingej1 end
}

/*************************************************************************
 * FUNCTION
 *   set_gain
 *
 * DESCRIPTION
 *   This function is to set global gain to sensor.
 *
 * PARAMETERS
 *   iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *   the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;

    if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
        LOG_INF("Error gain setting");
        
        if (gain < BASEGAIN)
            gain = BASEGAIN;
        else if (gain > 32 * BASEGAIN)
            gain = 32 * BASEGAIN;
    }
    
    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
    
    write_inv_sensor(0x3002, 0x01);
    write_inv_sensor_16(0x3910, reg_gain); // GreenR
    write_inv_sensor_16(0x3912, reg_gain); // Red
    write_inv_sensor_16(0x3914, reg_gain); // Blue
    write_inv_sensor_16(0x3916, reg_gain); // GreenB
    write_inv_sensor(0x3002, 0x00);
    
    return gain;
}   /*  set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
    LOG_INF("image_mirror = %d\n", image_mirror);
    
    /********************************************************
     *
     *   0x0101 Sensor mirror flip
     *
     *   ISP and Sensor flip or mirror register bit should be the same!!
     *
     ********************************************************/
    kal_uint8  iTemp;
    
    iTemp = read_inv_sensor(0x3208);
    iTemp&= ~0x03; //Clear the mirror and flip bits.
    
    switch (image_mirror) {
        case IMAGE_NORMAL:
            write_inv_sensor_8(0x3208, iTemp);    //Set normal
            break;
        case IMAGE_H_MIRROR:
            write_inv_sensor_8(0x3208, iTemp | 0x01); //Set mirror
            break;
        case IMAGE_V_MIRROR:
            write_inv_sensor_8(0x3208, iTemp | 0x02); //Set flip
            break;
        case IMAGE_HV_MIRROR:
            write_inv_sensor_8(0x3208, iTemp | 0x03); //Set mirror and flip
            break;
        default:
            LOG_INF("Error image_mirror setting\n");
    }
    
}


/*************************************************************************
 * FUNCTION
 *   night_mode
 *
 * DESCRIPTION
 *   This function night mode of sensor.
 *
 * PARAMETERS
 *   bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *   None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void night_mode(kal_bool enable)
{
    /*No Need to implement this function*/
}   /*  night_mode  */

//hzmadd
static void sensor_init(void)
{
    LOG_INF("E\n");
    write_inv_sensor(0x3000, 0x00);        // mode_select - STOP STREAMING 0x00
    //msleep(10);                // 50 ms delay to allow pll to settle after powerup and clock enabling
    write_inv_sensor(0x3002, 0x00);         // grouped_parameter_hold                   0x00
    write_inv_sensor(0x3003, 0x00);         // unlock                                   0x00
    write_inv_sensor(0x3004, 0x00);         // corrupted_frame_control                  0x00
    write_inv_sensor(0x3007, 0x4E);         // cfa_config                               0x4E
    write_inv_sensor_16(0x3008, 0x0040);       // data_pedestal                            0x0040
    write_inv_sensor_16(0x300E, 0x00FF);       // isp_bypass_ctrl                          0x0000
    write_inv_sensor(0x3014, 0x10);         // cci_slave_addr_0                         0x10
    write_inv_sensor(0x3016, 0x36);         // cci_slave_addr_1                         0x36
    write_inv_sensor(0x3020, 0x01);         // enable_auto_trigger                      0x01
    write_inv_sensor(0x3021, 0x00);         // enable_cci_trigger                       0x00
    write_inv_sensor(0x3023, 0x00);         // enable_external_trigger                  0x00
    write_inv_sensor(0x3024, 0x00);         // external_trigger_type                    0x00
    write_inv_sensor_16(0x3026, 0x0000);       // trigger_delay                            0x0000
    write_inv_sensor(0x3030, 0x00);         // io_ctrl                                  0x00
    write_inv_sensor(0x3042, 0x7F);         // global_interrupt_mask                    0x7F
    write_inv_sensor_16(0x3080, 0x0A0A);       // csi2_data_format                         0x0C0C
    write_inv_sensor(0x3082, 0x03);         // csi2_lane_mode                           0x00
    write_inv_sensor(0x3083, 0x00);         // csi2_vc_id                               0x00
    write_inv_sensor(0x3084, 0x30);         // csi2_10_to_8_dt                          0x30
    write_inv_sensor(0x3085, 0x00);         // csi2_ctrl                                0x00
    write_inv_sensor(0x3086, 0x02);         // csi2_t_hs_prepare                        0x0A
    write_inv_sensor(0x3087, 0x60);         // csi2_t_clk_pre                           0x01
    write_inv_sensor(0x3088, 0x01);         // csi2_t_clk_post                          0x01
    write_inv_sensor(0x3089, 0x01);         // csi2_t_hs_exit                           0x01
    write_inv_sensor_16(0x308D, 0x0000);     // csi2_t_wakeup                            0x000000
    write_inv_sensor(0x308F, 0x00);     // csi2_t_wakeup                            0x000000
    write_inv_sensor(0x3090, 0x00);         // csi2_edl_dt                              0x00
    write_inv_sensor(0x3091, 0x89);         // csi2_trigger_dt                          0x89
    write_inv_sensor(0x3100, 0x02);         // pre_pll_clk_div                          0x01
    write_inv_sensor_16(0x3102, 0x0038);       // pll_multiplier                           0x0001
    write_inv_sensor(0x3104, 0x10);         // vt_sys_clk_div                           0x01
    write_inv_sensor(0x3105, 0x01);         // vt_pix_clk_div                           0x01
    write_inv_sensor(0x3106, 0x02);         // op_sys_clk_div                           0x01
    write_inv_sensor(0x3107, 0x05);         // op_pix_clk_div                           0x04
    write_inv_sensor(0x3110, 0x01);         // pll_ctrl                                 0x0F
    write_inv_sensor(0x3111, 0x01);         // clk_ctrl1                                0x01
    write_inv_sensor(0x3112, 0x20);         // clk_ctrl0                                0x20
    write_inv_sensor(0x3115, 0x00);         // pll_event_clear                          0x00
    write_inv_sensor(0x3116, 0x03);         // pll_event_mask                           0x03
    write_inv_sensor(0x3117, 0x00);         // clk_ana_delay                            0x00
    write_inv_sensor(0x3118, 0x00);         // clk_cp_delay                             0x00
    write_inv_sensor(0x3119, 0x01);         // cp_divider                               0x01
    write_inv_sensor(0x311A, 0x0F);         // clk_standby_delay                        0x0F       (15)
    write_inv_sensor(0x311B, 0x02);         // csi2_esc_clk_ctrl                        0x02
    write_inv_sensor_16(0x3200, 0x0C45);       // frame_length_lines                       0x0BFE     (3070)
    write_inv_sensor_16(0x3202, 0x1108);       // line_length_pck                          0x11BC     (4540)
    write_inv_sensor_16(0x3204, 0x0C45);       // coarse_integration_time                  0x0001
    write_inv_sensor_16(0x3206, 0x0000);       // fine_integration_time                    0x0000
    write_inv_sensor(0x3208, 0x00);    // image_orientation                        0x00
    write_inv_sensor_16(0x320A, 0x0070);       // x_addr_start                             0x0070     (112)
    write_inv_sensor_16(0x320C, 0x0000);       // y_addr_start                             0x0004
    write_inv_sensor_16(0x320E, 0x100F);       // x_addr_end                               0x100F     (4111)
    write_inv_sensor_16(0x3210, 0x0BFF);       // y_addr_end                               0x0BBB     (3003)
    write_inv_sensor_16(0x3212, 0x0FA0);       // x_output_size                            0x0FA0     (4000)
    write_inv_sensor_16(0x3214, 0x0C00);       // y_output_size                            0x0BB8     (3000)
    write_inv_sensor_16(0x3216, 0x0001);       // x_even_inc                               0x0001
    write_inv_sensor_16(0x3218, 0x0001);       // x_odd_inc                                0x0001
    write_inv_sensor_16(0x321A, 0x0001);       // y_even_inc                               0x0001
    write_inv_sensor_16(0x321C, 0x0001);       // y_odd_inc                                0x0001
    write_inv_sensor_16(0x321E, 0x0064);       // x_addr_offset                            0x0064     (100)
    write_inv_sensor_16(0x3220, 0x0040);       // y_addr_offset                            0x0064     (100)
    write_inv_sensor(0x3222, 0x00);    // bin_control                              0x00
    write_inv_sensor_16(0x3226, 0x0002);       // y_even_inc_bin                           0x0002
    write_inv_sensor_16(0x3228, 0x0002);       // y_odd_inc_bin                            0x0002
    write_inv_sensor_16(0x322A, 0x0001);       // x_even_inc_ref                           0x0001
    write_inv_sensor_16(0x322C, 0x0001);       // x_odd_inc_ref                            0x0001
    write_inv_sensor_16(0x322E, 0x0001);       // y_even_inc_ref                           0x0001
    write_inv_sensor_16(0x3230, 0x0001);       // y_odd_inc_ref                            0x0001
    write_inv_sensor_16(0x3232, 0x0000);       // first_ref_row                            0x0004
    write_inv_sensor_16(0x3234, 0x0040);       // num_ref_rows                             0x0044     (68)
    write_inv_sensor_16(0x3236, 0x0004);       // first_ref_col                            0x0004
    write_inv_sensor_16(0x3238, 0x0040);       // num_ref_cols                             0x0044     (68)
    write_inv_sensor(0x323A, 0x01);         // show_ref_rows                            0x00
    write_inv_sensor(0x323C, 0x01);         // show_ref_cols                            0x00
    write_inv_sensor_16(0x323E, 0x0001);       // num_lead_edl_rows                        0x0001
    write_inv_sensor_16(0x3240, 0x0000);       // num_trail_edl_rows                       0x0000
    write_inv_sensor(0x3242, 0x01);    // lead_edl_mode                            0x01
    write_inv_sensor(0x3244, 0x00);    // trail_edl_mode                           0x00
    write_inv_sensor_16(0x3246, 0x0080);       // lead_edl_limit                           0x0080     (128)
    write_inv_sensor_16(0x3248, 0x0080);       // trail_edl_limit                          0x0080     (128)
    write_inv_sensor(0x324A, 0x01);    // show_edl_rows                            0x01
    write_inv_sensor(0x3300, 0xFF);    // gpio_config                              0xFF
    write_inv_sensor(0x3301, 0x00);    // gpio_in_sel                              0x00
    write_inv_sensor(0x3302, 0x00);    // gpio_out_sel                             0x00
    write_inv_sensor(0x3303, 0x00);    // gpio_ctrl                                0x00
    write_inv_sensor_16(0x3304, 0x0000);       // gpio1_clk_select                         0x0000
    write_inv_sensor_16(0x3306, 0x0000);       // gpio3_clk_select                         0x0000
    write_inv_sensor_16(0x3308, 0xFFFF);       // gpio1_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x330A, 0xFFF0);       // gpio1_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x330C, 0xFFFF);       // gpio1_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x330E, 0xFFF0);       // gpio1_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3310, 0xFFFF);       // gpio2_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x3312, 0xFFF0);       // gpio2_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x3314, 0xFFFF);       // gpio2_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x3316, 0xFFF0);       // gpio2_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3318, 0xFFFF);       // gpio3_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x331A, 0xFFF0);       // gpio3_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x331C, 0xFFFF);       // gpio3_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x331E, 0xFFF0);       // gpio3_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3320, 0xFFFF);       // gpio4_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x3322, 0xFFF0);       // gpio4_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x3324, 0xFFFF);       // gpio4_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x3326, 0xFFF0);       // gpio4_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3328, 0xFFFF);       // gpio1_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x332A, 0xFFF0);       // gpio1_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x332C, 0xFFFF);       // gpio2_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x332E, 0xFFF0);       // gpio2_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x3330, 0xFFFF);       // gpio3_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x3332, 0xFFF0);       // gpio3_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x3334, 0xFFFF);       // gpio4_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x3336, 0xFFF0);       // gpio4_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x3400, 0x0000);       // tg_control                               0x0000
    write_inv_sensor_16(0x3402, 0x1555);       // col_park_addr                            0x1555     (5461)
    write_inv_sensor_16(0x3404, 0x0FFF);       // row_park_addr                            0x0FFF     (4095)
    write_inv_sensor(0x3406, 0x00);    // gs_ctrl                                  0x00
    write_inv_sensor_16(0x3408, 0xFFFF);       // gs_open_row                              0xFFFF
    write_inv_sensor_16(0x340A, 0xFFFF);       // gs_open_col                              0xFFFF
    write_inv_sensor_16(0x340C, 0xFFFF);       // gs_close_row                             0xFFFF
    write_inv_sensor_16(0x340E, 0xFFFF);       // gs_close_col                             0xFFFF
    write_inv_sensor_16(0x3422, 0x0007);       // framecnt_range0_end                      0x0007
    write_inv_sensor_16(0x3424, 0x0004);       // framecnt_frame_valid_start               0x0004
    write_inv_sensor_16(0x3426, 0x0004);       // framecnt_frame_valid_end                 0x0004
    write_inv_sensor_16(0x3428, 0xFFFF);       // lp_col_pulse_start                       0xFFFF
    write_inv_sensor_16(0x342A, 0xFFF0);       // lp_col_pulse_end                         0xFFF0
    write_inv_sensor_16(0x342C, 0xFFFF);       // lp_row_pulse_start                       0xFFFF
    write_inv_sensor_16(0x342E, 0xFFF0);       // lp_row_pulse_end                         0xFFF0
    write_inv_sensor(0x3441, 0x37);         // initphase_range0_end                     0x37       (55)
    write_inv_sensor(0x3442, 0x00);         // initphase_latch_reset_at_startup_start   0x00
    write_inv_sensor(0x3443, 0xFF);         // initphase_latch_reset_at_startup_end     0xFF       (255)
    write_inv_sensor(0x3451, 0x7F);         // readphase_range0_end                     0xD7       (215)
    write_inv_sensor(0x3452, 0x02);         // readphase_row_select_pulse_1_start       0x21       (33)
    write_inv_sensor(0x3453, 0x77);         // readphase_row_select_pulse_1_end         0xAB       (171)
    write_inv_sensor(0x3454, 0xFF);         // readphase_row_select_pulse_2_start       0xF0       (240)
    write_inv_sensor(0x3455, 0xFE);         // readphase_row_select_pulse_2_end         0xFF       (255)
    write_inv_sensor(0x3456, 0x2E);         // readphase_row_reset_pulse_1_start        0x58       (88)
    write_inv_sensor(0x3457, 0x48);         // readphase_row_reset_pulse_1_end          0x75       (117)
    write_inv_sensor(0x3458, 0xFF);         // readphase_row_reset_pulse_2_start        0xA8       (168)
    write_inv_sensor(0x3459, 0xFE);         // readphase_row_reset_pulse_2_end          0xD0       (208)
    write_inv_sensor(0x345A, 0xFF);         // readphase_en_global_reset_start          0xFF       (255)
    write_inv_sensor(0x345B, 0xFE);         // readphase_en_global_reset_end            0xFE       (254)
    write_inv_sensor(0x345C, 0x79);         // readphase_latch_reset_pulse1_start       0xD1       (209)
    write_inv_sensor(0x345D, 0x7B);         // readphase_latch_reset_pulse1_end         0xD3       (211)
    write_inv_sensor(0x345E, 0xFF);         // readphase_latch_reset_pulse2_start       0xFF       (255)
    write_inv_sensor(0x345F, 0xFE);         // readphase_latch_reset_pulse2_end         0xFE       (254)
    write_inv_sensor(0x3460, 0xFF);         // readphase_gate_charge_pump_start         0xFF       (255)
    write_inv_sensor(0x3461, 0xFE);         // readphase_gate_charge_pump_end           0xFE       (254)
    write_inv_sensor(0x3462, 0x00);         // readphase_en_all_col_for_sampling_start  0x02
    write_inv_sensor(0x3463, 0x78);         // readphase_en_all_col_for_sampling_end    0xCF       (207)
    write_inv_sensor(0x3464, 0xFF);         // readphase_manual_path_a_enable_start     0x04
    write_inv_sensor(0x3465, 0xFE);         // readphase_manual_path_a_enable_end       0xCD       (205)
    write_inv_sensor(0x3466, 0xFF);         // readphase_manual_path_b_enable_start     0x04
    write_inv_sensor(0x3467, 0xFE);         // readphase_manual_path_b_enable_end       0xCD       (205)
    write_inv_sensor(0x3468, 0x04);         // readphase_samp_sig_path_a_1_start        0x28       (40)
    write_inv_sensor(0x3469, 0x27);         // readphase_samp_sig_path_a_1_end          0x4E       (78)
    write_inv_sensor(0x346A, 0xFF);         // readphase_samp_sig_path_a_2_start        0xFF       (255)
    write_inv_sensor(0x346B, 0xFE);         // readphase_samp_sig_path_a_2_end          0xFE       (254)
    write_inv_sensor(0x346C, 0x4B);         // readphase_samp_rst_path_a_start          0x7E       (126)
    write_inv_sensor(0x346D, 0x76);         // readphase_samp_rst_path_a_end            0xA4       (164)
    write_inv_sensor(0x346E, 0x04);         // readphase_samp_sig_path_b_1_start        0x28       (40)
    write_inv_sensor(0x346F, 0x27);         // readphase_samp_sig_path_b_1_end          0x4E       (78)
    write_inv_sensor(0x3470, 0xFF);         // readphase_samp_sig_path_b_2_start        0xFF       (255)
    write_inv_sensor(0x3471, 0xFE);         // readphase_samp_sig_path_b_2_end          0xFE       (254)
    write_inv_sensor(0x3472, 0x4B);         // readphase_samp_rst_path_b_start          0x7E       (126)
    write_inv_sensor(0x3473, 0x76);         // readphase_samp_rst_path_b_end            0xA4       (164)
    write_inv_sensor(0x3474, 0xFF);         // readphase_disable_col_bias_1_start       0xFF       (255)
    write_inv_sensor(0x3475, 0xFE);         // readphase_disable_col_bias_1_end         0xFE       (254)
    write_inv_sensor(0x3476, 0xFF);         // readphase_disable_col_bias_2_start       0xFF       (255)
    write_inv_sensor(0x3477, 0xFE);         // readphase_disable_col_bias_2_end         0xFE       (254)
    write_inv_sensor(0x3478, 0xFF);         // readphase_disable_col_bias_3_start       0xFF       (255)
    write_inv_sensor(0x3479, 0xFE);         // readphase_disable_col_bias_3_end         0xFE       (254)
    write_inv_sensor(0x347A, 0x29);         // readphase_en_hard_reset_start            0x50       (80)
    write_inv_sensor(0x347B, 0x35);         // readphase_en_hard_reset_end              0xCF       (207)
    write_inv_sensor(0x347C, 0xFF);         // readphase_en_low_noise_reset_start       0xFF       (255)
    write_inv_sensor(0x347D, 0xFE);         // readphase_en_low_noise_reset_end         0xFE       (254)
    write_inv_sensor(0x347E, 0xFF);         // readphase_set_low_noise_comps_start      0x00
    write_inv_sensor(0x347F, 0xFE);         // readphase_set_low_noise_comps_end        0xFF       (255)
    write_inv_sensor(0x3480, 0xFF);         // readphase_reset_low_noise_comps_start    0xFF       (255)
    write_inv_sensor(0x3481, 0xFE);         // readphase_reset_low_noise_comps_end      0xFE       (254)
    write_inv_sensor(0x3482, 0xFF);         // readphase_en_black_sun_clamp_top_start   0xFF       (255)
    write_inv_sensor(0x3483, 0xFE);         // readphase_en_black_sun_clamp_top_end     0xFE       (254)
    write_inv_sensor(0x3484, 0xFF);         // readphase_en_black_sun_clamp_bot_start   0xFF       (255)
    write_inv_sensor(0x3485, 0xFE);         // readphase_en_black_sun_clamp_bot_end     0xFE       (254)
    write_inv_sensor(0x3486, 0xFF);         // readphase_pulse_sf_out_1_start           0x10       (16)
    write_inv_sensor(0x3487, 0xFE);         // readphase_pulse_sf_out_1_end             0x1F       (31)
    write_inv_sensor(0x3488, 0xFF);         // readphase_pulse_sf_out_2_start           0x8F       (143)
    write_inv_sensor(0x3489, 0xFE);         // readphase_pulse_sf_out_2_end             0x9F       (159)
    write_inv_sensor(0x348A, 0xFF);         // readphase_pulse_sf_out_3_start           0xD0       (208)
    write_inv_sensor(0x348B, 0xFE);         // readphase_pulse_sf_out_3_end             0xDF       (223)
    write_inv_sensor(0x348C, 0xFF);         // readphase_inject_sig_at_sf_out_start     0xFF       (255)
    write_inv_sensor(0x348D, 0xFE);         // readphase_inject_sig_at_sf_out_end       0xFE       (254)
    write_inv_sensor(0x348E, 0xFF);         // readphase_inject_rst_at_sf_out_start     0xFF       (255)
    write_inv_sensor(0x348F, 0xFE);         // readphase_inject_rst_at_sf_out_end       0xFE       (254)
    write_inv_sensor_16(0x3490, 0x007F);       // fine_itime_offset                        0x0050     (80)
    write_inv_sensor(0x3493, 0x7F);         // resetphase_range0_end                    0x4F       (79)
    write_inv_sensor(0x3494, 0x02);         // resetphase_row_select_pulse_1_start      0x00
    write_inv_sensor(0x3495, 0x77);         // resetphase_row_select_pulse_1_end        0xFF       (255)
    write_inv_sensor(0x3496, 0xFF);         // resetphase_row_select_pulse_2_start      0xFF       (255)
    write_inv_sensor(0x3497, 0xFE);         // resetphase_row_select_pulse_2_end        0xFE       (254)
    write_inv_sensor(0x3498, 0x2E);         // resetphase_row_reset_pulse_start         0x25       (37)
    write_inv_sensor(0x3499, 0x48);         // resetphase_row_reset_pulse_end           0x42       (66)
    write_inv_sensor(0x349A, 0xFF);         // resetphase_en_global_reset_start         0xFF       (255)
    write_inv_sensor(0x349B, 0xFE);         // resetphase_en_global_reset_end           0xFE       (254)
    write_inv_sensor(0x349C, 0x02);         // resetphase_latch_reset_pulse1_start      0x21       (33)
    write_inv_sensor(0x349D, 0x04);         // resetphase_latch_reset_pulse1_end        0x23       (35)
    write_inv_sensor(0x349E, 0xFF);         // resetphase_latch_reset_pulse2_start      0xFF       (255)
    write_inv_sensor(0x349F, 0xFE);         // resetphase_latch_reset_pulse2_end        0xFE       (254)
    write_inv_sensor(0x34A0, 0xFF);         // resetphase_gate_charge_pump_start        0xFF       (255)
    write_inv_sensor(0x34A1, 0xFE);         // resetphase_gate_charge_pump_end          0xFE       (254)
    write_inv_sensor(0x34A2, 0xFF);         // resetphase_en_all_col_for_sampling_start 0xFF       (255)
    write_inv_sensor(0x34A3, 0xFE);         // resetphase_en_all_col_for_sampling_end   0xFE       (254)
    write_inv_sensor(0x34A4, 0xFF);         // resetphase_samp_sig_path_a_start         0xFF       (255)
    write_inv_sensor(0x34A5, 0xFE);         // resetphase_samp_sig_path_a_end           0xFE       (254)
    write_inv_sensor(0x34A6, 0xFF);         // resetphase_samp_rst_path_a_start         0xFF       (255)
    write_inv_sensor(0x34A7, 0xFE);         // resetphase_samp_rst_path_a_end           0xFE       (254)
    write_inv_sensor(0x34A8, 0xFF);         // resetphase_samp_sig_path_b_start         0xFF       (255)
    write_inv_sensor(0x34A9, 0xFE);         // resetphase_samp_sig_path_b_end           0xFE       (254)
    write_inv_sensor(0x34AA, 0xFF);         // resetphase_samp_rst_path_b_start         0xFF       (255)
    write_inv_sensor(0x34AB, 0xFE);         // resetphase_samp_rst_path_b_end           0xFE       (254)
    write_inv_sensor(0x34AC, 0xFF);         // resetphase_disable_col_bias_1_start      0xFF       (255)
    write_inv_sensor(0x34AD, 0xFE);         // resetphase_disable_col_bias_1_end        0xFE       (254)
    write_inv_sensor(0x34AE, 0xFF);         // resetphase_disable_col_bias_2_start      0xFF       (255)
    write_inv_sensor(0x34AF, 0xFE);         // resetphase_disable_col_bias_2_end        0xFE       (254)
    write_inv_sensor(0x34B0, 0xFF);         // resetphase_disable_col_bias_3_start      0xFF       (255)
    write_inv_sensor(0x34B1, 0xFE);         // resetphase_disable_col_bias_3_end        0xFE       (254)
    write_inv_sensor(0x34B2, 0x29);         // resetphase_en_hard_reset_start           0x50       (80)
    write_inv_sensor(0x34B3, 0x35);         // resetphase_en_hard_reset_end             0x6F       (111)
    write_inv_sensor(0x34B4, 0xFF);         // resetphase_en_low_noise_reset_start      0xFF       (255)
    write_inv_sensor(0x34B5, 0xFE);         // resetphase_en_low_noise_reset_end        0xFE       (254)
    write_inv_sensor(0x34B6, 0xFF);         // resetphase_set_low_noise_comps_start     0x00
    write_inv_sensor(0x34B7, 0xFE);         // resetphase_set_low_noise_comps_end       0xFF       (255)
    write_inv_sensor(0x34B8, 0xFF);         // resetphase_reset_low_noise_comps_start   0xFF       (255)
    write_inv_sensor(0x34B9, 0xFE);         // resetphase_reset_low_noise_comps_end     0xFE       (254)
    write_inv_sensor(0x34BA, 0xFF);         // resetphase_en_black_sun_clamp_top_start  0xFF       (255)
    write_inv_sensor(0x34BB, 0xFE);         // resetphase_en_black_sun_clamp_top_end    0xFE       (254)
    write_inv_sensor(0x34BC, 0xFF);         // resetphase_en_black_sun_clamp_bot_start  0xFF       (255)
    write_inv_sensor(0x34BD, 0xFE);         // resetphase_en_black_sun_clamp_bot_end    0xFE       (254)
    write_inv_sensor(0x34BE, 0xFF);         // resetphase_pulse_sf_out_1_start          0xFF       (255)
    write_inv_sensor(0x34BF, 0xFE);         // resetphase_pulse_sf_out_1_end            0xFE       (254)
    write_inv_sensor(0x34C0, 0xFF);         // resetphase_pulse_sf_out_2_start          0xFF       (255)
    write_inv_sensor(0x34C1, 0xFE);         // resetphase_pulse_sf_out_2_end            0xFE       (254)
    write_inv_sensor(0x34C2, 0xFF);         // resetphase_pulse_sf_out_3_start          0xFF       (255)
    write_inv_sensor(0x34C3, 0xFE);         // resetphase_pulse_sf_out_3_end            0xFE       (254)
    write_inv_sensor(0x34C4, 0xFF);         // resetphase_inject_sig_at_sf_out_start    0xFF       (255)
    write_inv_sensor(0x34C5, 0xFE);         // resetphase_inject_sig_at_sf_out_end      0xFE       (254)
    write_inv_sensor(0x34C6, 0xFF);         // resetphase_inject_rst_at_sf_out_start    0xFF       (255)
    write_inv_sensor(0x34C7, 0xFE);         // resetphase_inject_rst_at_sf_out_end      0xFE       (254)
    write_inv_sensor(0x34D0, 0x00);         // tgout_format_global_shutter              0x00
    write_inv_sensor(0x34D1, 0x00);         // tgout_format_global_reset                0x00
    write_inv_sensor(0x34D2, 0x00);         // tgout_format_row_select                  0x00
    write_inv_sensor(0x34D3, 0x00);         // tgout_format_row_reset                   0x00
    write_inv_sensor(0x34D4, 0x00);         // tgout_format_latch_reset_at_startup      0x00
    write_inv_sensor(0x34D5, 0x00);         // tgout_format_latch_reset                 0x00
    write_inv_sensor(0x34D6, 0x00);         // tgout_format_gate_charge_pump            0x00
    write_inv_sensor(0x34D8, 0x00);         // tgout_format_col_switch_left_top         0x00
    write_inv_sensor(0x34D9, 0x00);         // tgout_format_col_switch_left_bot         0x00
    write_inv_sensor(0x34DA, 0x00);         // tgout_format_col_switch_left_dig_top     0x00
    write_inv_sensor(0x34DB, 0x00);         // tgout_format_col_switch_left_dig_bot     0x00
    write_inv_sensor(0x34DC, 0x00);         // tgout_format_en_all_col_for_sampling_top 0x00
    write_inv_sensor(0x34DD, 0x00);         // tgout_format_en_all_col_for_sampling_bot 0x00
    write_inv_sensor(0x34DE, 0x00);         // tgout_format_samp_sig_path_a_top         0x00
    write_inv_sensor(0x34DF, 0x00);         // tgout_format_samp_sig_path_a_bot         0x00
    write_inv_sensor(0x34E0, 0x00);         // tgout_format_samp_sig_path_b_top         0x00
    write_inv_sensor(0x34E1, 0x00);         // tgout_format_samp_sig_path_b_bot         0x00
    write_inv_sensor(0x34E2, 0x00);         // tgout_format_samp_rst_path_a_top         0x00
    write_inv_sensor(0x34E3, 0x00);         // tgout_format_samp_rst_path_a_bot         0x00
    write_inv_sensor(0x34E4, 0x00);         // tgout_format_samp_rst_path_b_top         0x00
    write_inv_sensor(0x34E5, 0x00);         // tgout_format_samp_rst_path_b_bot         0x00
    write_inv_sensor(0x34E6, 0x00);         // tgout_format_disable_col_bias_top        0x00
    write_inv_sensor(0x34E7, 0x00);         // tgout_format_disable_col_bias_bot        0x00
    write_inv_sensor(0x34E8, 0x00);         // tgout_format_en_hard_reset_top           0x00
    write_inv_sensor(0x34E9, 0x00);         // tgout_format_en_hard_reset_bot           0x00
    write_inv_sensor(0x34EA, 0x00);         // tgout_format_en_low_noise_reset_top      0x00
    write_inv_sensor(0x34EB, 0x00);         // tgout_format_en_low_noise_reset_bot      0x00
    write_inv_sensor(0x34EC, 0x00);         // tgout_format_set_low_noise_comps_top     0x00
    write_inv_sensor(0x34ED, 0x00);         // tgout_format_set_low_noise_comps_bot     0x00
    write_inv_sensor(0x34EE, 0x00);         // tgout_format_reset_low_noise_comps_top   0x00
    write_inv_sensor(0x34EF, 0x00);         // tgout_format_reset_low_noise_comps_bot   0x00
    write_inv_sensor(0x34F0, 0x00);         // tgout_format_en_black_sun_clamp_top      0x00
    write_inv_sensor(0x34F1, 0x00);         // tgout_format_en_black_sun_clamp_bot      0x00
    write_inv_sensor(0x34F2, 0x00);         // tgout_format_pulse_sf_out_top            0x00
    write_inv_sensor(0x34F3, 0x00);         // tgout_format_pulse_sf_out_bot            0x00
    write_inv_sensor(0x34F4, 0x00);         // tgout_format_inject_sig_at_sf_out_top    0x00
    write_inv_sensor(0x34F5, 0x00);         // tgout_format_inject_sig_at_sf_out_bot    0x00
    write_inv_sensor(0x34F6, 0x00);         // tgout_format_inject_rst_at_sf_out_top    0x00
    write_inv_sensor(0x34F7, 0x00);         // tgout_format_inject_rst_at_sf_out_bot    0x00
    write_inv_sensor(0x3600, 0x00);         // pd_positive_filmbias_dac                 0x01
    write_inv_sensor(0x3601, 0x7F);         // pos_filmbias_threshold_adjust            0x00
    write_inv_sensor(0x3602, 0x00);         // en_float_filmbias                        0x00
    write_inv_sensor(0x3603, 0x00);         // en_internal_filmbias                     0x00
    write_inv_sensor(0x3604, 0x00);         // pd_rst_noise_dac                         0x01
    write_inv_sensor(0x3605, 0x00);         // rst_noise_threshold_adjust               0x00
    write_inv_sensor(0x3610, 0x00);         // pd_bg_inv_top                            0x01
    write_inv_sensor(0x3611, 0x00);         // pd_bg_inv_bot                            0x01
    write_inv_sensor(0x3612, 0x00);         // pd_iqa_iptat_gen                         0x03
    write_inv_sensor(0x3613, 0x00);         // pd_bias_gen                              0x03
    write_inv_sensor(0x3614, 0x01);         // sel_inv_bg_top                           0x00
    write_inv_sensor(0x3615, 0x01);         // sel_inv_bg_bot                           0x00
    write_inv_sensor(0x3616, 0x01);         // sel_inv_iptat_7p5uA_top                  0x00
    write_inv_sensor(0x3617, 0x01);         // sel_inv_iptat_7p5uA_bot                  0x00
    write_inv_sensor(0x3618, 0x00);         // ictrl_global_bias_top                    0x00
    write_inv_sensor(0x3619, 0x00);         // ictrl_global_bias_bot                    0x00
    write_inv_sensor(0x361A, 0x00);         // vbg_inv_trim_top                         0x00
    write_inv_sensor(0x361B, 0x00);         // vbg_inv_trim_bot                         0x00
    write_inv_sensor(0x3620, 0x00);         // pd_cpump                                 0x01
    write_inv_sensor(0x3621, 0x00);         // ictrl_cpump                              0x00
    write_inv_sensor(0x3622, 0x00);         // en_gnd_filmbias                          0x00
    write_inv_sensor(0x3623, 0xFF);         // prog_cpump                               0x00
    write_inv_sensor(0x3624, 0x00);         // cpump_hyst_ctrl                          0x00
    write_inv_sensor(0x3630, 0x00);         // pd_blacksun_dac                          0x01
    write_inv_sensor(0x3632, 0x00);         // blacksun_threshold_adjust_top            0x00
    write_inv_sensor(0x3633, 0x00);         // blacksun_threshold_adjust_bot            0x00
    write_inv_sensor(0x3640, 0x01);         // pd_adft_out_buf                          0x03
    write_inv_sensor(0x3642, 0x01);         // ictrl_col_bias_top                       0x01
    write_inv_sensor(0x3643, 0x01);         // ictrl_col_bias_bot                       0x01
    write_inv_sensor(0x3644, 0x00);         // ictrl_comp_top                           0x00
    write_inv_sensor(0x3645, 0x00);         // rst_noise_taper_n_adj                    0x00
    write_inv_sensor(0x3646, 0x00);         // ictrl_pullup_top                         0x00
    write_inv_sensor(0x3647, 0x00);         // rst_noise_taper_p_adj                    0x00
    write_inv_sensor(0x3648, 0x00);         // sel_adft_top                             0x00
    write_inv_sensor(0x3649, 0x00);         // sel_adft_bot                             0x00
    write_inv_sensor(0x364A, 0x00);         // adft_1_ch_sel_top                        0x00
    write_inv_sensor(0x364B, 0x00);         // adft_1_ch_sel_bot                        0x00
    write_inv_sensor(0x364E, 0x00);         // set_sf_out_to_gnd_top                    0x00
    write_inv_sensor(0x364F, 0x00);         // set_sf_out_to_gnd_bot                    0x00
    write_inv_sensor(0x3650, 0x00);         // set_sf_out_to_pixpwr_top                 0x00
    write_inv_sensor(0x3651, 0x00);         // set_sf_out_to_pixpwr_bot                 0x00
    write_inv_sensor(0x3652, 0x00);         // monitor_sf_out_top                       0x00
    write_inv_sensor(0x3653, 0x00);         // monitor_sf_out_bot                       0x00
    write_inv_sensor(0x3654, 0x00);         // en_temp_monitor_pad_top                  0x00
    write_inv_sensor(0x3655, 0x00);         // en_temp_monitor_pad_bot                  0x00
    write_inv_sensor(0x3656, 0x00);         // short_hard_reset_top                     0x00
    write_inv_sensor(0x3657, 0x00);         // short_hard_reset_bot                     0x00
    write_inv_sensor(0x3658, 0x02);         // analog_aux_top                           0x00
    write_inv_sensor(0x3659, 0x02);         // analog_aux_bot                           0x00
    write_inv_sensor_16(0x3660, 0xAAAA);       // digital_aux                              0xAAAA
    write_inv_sensor(0x3700, 0x00);         // pd_cds                                   0x03
    write_inv_sensor(0x3701, 0x00);         // pd_cds_vcm                               0x0F
    write_inv_sensor(0x3702, 0x00);         // pd_cds_ref_buf_ctrl                      0x0F
    write_inv_sensor(0x3704, 0x2A);         // ictrl_cds_top                            0x00
    write_inv_sensor(0x3705, 0x2A);         // ictrl_cds_bot                            0x00
    write_inv_sensor(0x3706, 0x02);         // vcmi_trim_top                            0x00
    write_inv_sensor(0x3707, 0x02);         // vcmi_trim_bot                            0x00
    write_inv_sensor(0x3708, 0x00);         // clk_delay_to_cds_top                     0x00
    write_inv_sensor(0x3709, 0x00);         // clk_delay_to_cds_bot                     0x00
    write_inv_sensor(0x370A, 0x01);         // cds_gain_top                             0x00
    write_inv_sensor(0x370B, 0x01);         // cds_gain_bot                             0x00
    write_inv_sensor(0x370C, 0x08);         // cds_ctrl_top                             0x00
    write_inv_sensor(0x370D, 0x08);         // cds_ctrl_bot                             0x00
    write_inv_sensor(0x370E, 0x91);         // cds_control                              0x11
    write_inv_sensor_16(0x3711, 0x0000);     // keep_on_cds_amp_top                      0x000000
    write_inv_sensor(0x3713, 0x00);     // keep_on_cds_amp_top                      0x000000
    write_inv_sensor_16(0x3715, 0x0000);     // keep_on_cds_amp_bot                      0x000000
    write_inv_sensor(0x3717, 0x00);     // keep_on_cds_amp_bot                      0x000000
    write_inv_sensor_16(0x3719, 0x0000);     // keep_off_cds_amp_top                     0x01FFFF
    write_inv_sensor(0x3721, 0x00);     // keep_off_cds_amp_top                     0x01FFFF
    write_inv_sensor_16(0x371D, 0x0000);     // keep_off_cds_amp_bot                     0x01FFFF
    write_inv_sensor(0x371F, 0x00);     // keep_off_cds_amp_bot                     0x01FFFF
    write_inv_sensor(0x3800, 0x88);         // afe_ctrl_0_top                           0x77
    write_inv_sensor(0x3801, 0x88);         // afe_ctrl_0_bot                           0x77
    write_inv_sensor(0x3802, 0x00);         // afe_ctrl_1_top                           0x00
    write_inv_sensor(0x3803, 0x00);         // afe_ctrl_1_bot                           0x00
    write_inv_sensor(0x3804, 0x00);         // afe_ctrl_2_top                           0x00
    write_inv_sensor(0x3805, 0x00);         // afe_ctrl_2_bot                           0x00
    write_inv_sensor(0x3806, 0x00);         // afe_ctrl_3_top                           0x00
    write_inv_sensor(0x3807, 0x00);         // afe_ctrl_3_bot                           0x00
    write_inv_sensor(0x3808, 0x00);         // afe_ctrl_4_top                           0x00
    write_inv_sensor(0x3809, 0x00);         // afe_ctrl_4_bot                           0x00
    write_inv_sensor(0x380A, 0x00);         // afe_ctrl_5_top                           0x00
    write_inv_sensor(0x380B, 0x00);         // afe_ctrl_5_bot                           0x00
    write_inv_sensor(0x380C, 0x00);         // afe_ctrl_6_top                           0x00
    write_inv_sensor(0x380D, 0x00);         // afe_ctrl_6_bot                           0x00
    write_inv_sensor(0x380E, 0x00);         // afe_ctrl_7_top                           0x00
    write_inv_sensor(0x380F, 0x00);         // afe_ctrl_7_bot                           0x00
    write_inv_sensor(0x3810, 0x02);         // afe_ctrl_8_top                           0x00
    write_inv_sensor(0x3811, 0x02);         // afe_ctrl_8_bot                           0x00
    write_inv_sensor(0x3812, 0x00);         // clk_delay_to_adc_top                     0x00
    write_inv_sensor(0x3813, 0x00);         // clk_delay_to_adc_bot                     0x00
    write_inv_sensor(0x3814, 0x0F);         // invert_adc_clk                           0x00
    write_inv_sensor(0x3815, 0x0F);         // analog_chain_latency                     0x11       (17)
    write_inv_sensor(0x3816, 0x00);         // adc_channel_mode                         0x00
    write_inv_sensor(0x3817, 0x00);         // adc_data_capture_control                 0x00
    write_inv_sensor(0x3900, 0x00);         // gain_mode                                0x00
    write_inv_sensor(0x3901, 0x00);         // analog_gain_global                       0x00
    write_inv_sensor(0x3902, 0x00);         // analog_gain_greenR                       0x00
    write_inv_sensor(0x3903, 0x00);         // analog_gain_red                          0x00
    write_inv_sensor(0x3904, 0x00);         // analog_gain_blue                         0x00
    write_inv_sensor(0x3905, 0x00);         // analog_gain_greenB                       0x00
    write_inv_sensor_16(0x3910, 0x0100);       // digital_gain_greenR                      0x0100     (256)
    write_inv_sensor_16(0x3912, 0x0100);       // digital_gain_red                         0x0100     (256)
    write_inv_sensor_16(0x3914, 0x0100);       // digital_gain_blue                        0x0100     (256)
    write_inv_sensor_16(0x3916, 0x0100);       // digital_gain_greenB                      0x0100     (256)
    write_inv_sensor(0x3A00, 0x00);    // blc_mode                                 0x00
    write_inv_sensor_16(0x3A02, 0x0080);       // blc_target                               0x0080     (128)
    write_inv_sensor_16(0x3A04, 0x0040);       // blc_window                               0x0040     (64)
    write_inv_sensor_16(0x3A06, 0x0FFF);       // blc_threshold                            0x0FFF
    write_inv_sensor(0x3A08, 0x01);    // blc_update_ctrl                          0x01
    write_inv_sensor(0x3A09, 0x14);    // blc_settle_time                          0x14       (20)
    write_inv_sensor(0x3A0A, 0x01);    // blc_adjust_rate                          0x01
    write_inv_sensor(0x3A0B, 0x00);    // blc_max_rows                             0x00
    write_inv_sensor_16(0x3A0C, 0x0000);       // blc_autostop                             0x0000
    write_inv_sensor_16(0x3A10, 0x0100);       // blc_dac_ch0_top                          0x0100
    write_inv_sensor_16(0x3A12, 0x0100);       // blc_dac_ch1_top                          0x0100
    write_inv_sensor_16(0x3A14, 0x0100);       // blc_dac_ch0_bot                          0x0100
    write_inv_sensor_16(0x3A16, 0x0100);       // blc_dac_ch1_bot                          0x0100
    write_inv_sensor(0x3A1A, 0x03);    // blc_event_mask                           0x03
    write_inv_sensor(0x3A40, 0x0E);    // rtn_mode                                 0x0E       (14)
    write_inv_sensor_16(0x3A42, 0x0080);       // rtn_target                               0x0080     (128)
    write_inv_sensor_16(0x3A44, 0x0020);       // rtn_min                                  0x0020     (32)
    write_inv_sensor_16(0x3A46, 0x0200);       // rtn_max                                  0x0200     (512)
    write_inv_sensor_16(0x3A48, 0x0000);       // rtn_invalid_cols                         0x0000
    write_inv_sensor_16(0x3A80, 0x6021);       // fpn_mode                                 0x6021
    write_inv_sensor_16(0x3A84, 0x0040);       // fpn_target                               0x0040     (64)
    write_inv_sensor_16(0x3A86, 0x0010);       // fpn_min                                  0x0010     (16)
    write_inv_sensor_16(0x3A88, 0x0200);       // fpn_max                                  0x0200     (512)
    write_inv_sensor_16(0x3A8A, 0x0020);       // fpn_initial_step_size                    0x0020     (32)
    write_inv_sensor_16(0x3A8C, 0x0080);       // fpn_num_ref_row_step                     0x0080     (128)
    write_inv_sensor(0x3A92, 0x00);    // fpn_event_clear                          0x00
    write_inv_sensor(0x3A94, 0x03);    // fpn_event_mask                           0x03
    write_inv_sensor_16(0x3A96, 0x0000);       // fpn_mem_status                           0x0000
    write_inv_sensor_16(0x3B00, 0x0000);       // test_pattern_mode                        0x0000
    write_inv_sensor_16(0x3B02, 0x0000);       // test_data_red                            0x0000
    write_inv_sensor_16(0x3B04, 0x0000);       // test_data_greenR                         0x0000
    write_inv_sensor_16(0x3B06, 0x0000);       // test_data_blue                           0x0000
    write_inv_sensor_16(0x3B08, 0x0000);       // test_data_greenB                         0x0000
    write_inv_sensor_16(0x3B0A, 0x0000);       // horizontal_cursor_width                  0x0000
    write_inv_sensor_16(0x3B0C, 0x0000);       // horizontal_cursor_position               0x0000
    write_inv_sensor_16(0x3B0E, 0x0000);       // vertical_cursor_width                    0x0000
    write_inv_sensor_16(0x3B10, 0x0000);       // vertical_cursor_position                 0x0000
    write_inv_sensor_16(0x3D00, 0x0500);       // fifo_water_mark_pixels                   0x0400     (1024)
    write_inv_sensor(0x3D04, 0x00);    // bypass_output_fifo                       0x00
    write_inv_sensor(0x3D07, 0x00);    // output_fifo_event1_clear                 0x00
    write_inv_sensor(0x3D08, 0x07);    // output_fifo_event1_mask                  0x07
    write_inv_sensor(0x3D0B, 0x00);    // output_fifo_event2_clear                 0x00
    write_inv_sensor(0x3D0C, 0x3F);    // output_fifo_event2_mask                  0x3F
    write_inv_sensor_16(0x3E00, 0x0000);       // otp_addr                                 0x0000
    write_inv_sensor(0x3E02, 0x00);    // otp_wr_data                              0x00
    write_inv_sensor(0x3E04, 0x00);    // otp_config                               0x00
    write_inv_sensor(0x3E05, 0x00);    // otp_cmd                                  0x00
    write_inv_sensor_16(0x3E08, 0x0000);       // otp_mr_wr_data                           0x0000
    write_inv_sensor_16(0x3E0A, 0x0000);       // otp_mra_wr_data                          0x0000
    write_inv_sensor_16(0x3E0C, 0x0000);       // otp_mrb_wr_data                          0x0000
    write_inv_sensor_16(0x3E14, 0x0000);       // otp_mr_wr_pgm_rd_data                    0x0000
    write_inv_sensor_16(0x3E16, 0x0000);       // otp_mra_wr_pgm_rd_data                   0x0000
    write_inv_sensor_16(0x3E18, 0x0000);       // otp_mrb_wr_pgm_rd_data                   0x0000
    write_inv_sensor_16(0x3E1A, 0x0000);       // otp_ctl                                  0x0000
    write_inv_sensor_16(0x3E1C, 0x0000);       // otp_prog_pulse_width_cnt                 0x0000
    write_inv_sensor_16(0x3E1E, 0x0000);       // otp_prog_soak_pulse_width_cnt            0x0000
    write_inv_sensor_16(0x3E20, 0x0000);       // otp_prog_recovery_width_cnt              0x0000
    write_inv_sensor_16(0x3E22, 0x0000);       // otp_read_recovery_width_cnt              0x0000
    write_inv_sensor_16(0x3E24, 0x0000);       // otp_other_recovery_width_cnt             0x0000
    write_inv_sensor_16(0x3E26, 0x0000);       // otp_chg_pump_rdy_cnt                     0x0000
    write_inv_sensor_16(0x3E28, 0x0000);       // otp_transaction_cntr_max                 0x0000
    write_inv_sensor(0x3F00, 0x88);    // mbist_rm_fpn_top                         0x88
    write_inv_sensor(0x3F01, 0x88);    // mbist_rm_fpn_bot                         0x88
    write_inv_sensor(0x3F02, 0xDD);    // mbist_rm_oi                              0xDD
    write_inv_sensor_16(0x3F72, 0x0005);       // mbist_seed_fpn_top                       0x0005
    write_inv_sensor_16(0x3F76, 0x0000);       // mbist_wr_data_fpn_top                    0x0000
    write_inv_sensor_16(0x3F7A, 0x0000);       // mbist_address_fpn_top                    0x0000
    write_inv_sensor_16(0x3F7C, 0x137F);       // mbist_depth_fpn_top                      0x137F     (4991)
    write_inv_sensor_16(0x3F7E, 0x4060);       // mbist_ctrl_fpn_top                       0x4060
    write_inv_sensor_16(0x3F82, 0x0005);       // mbist_seed_fpn_bot                       0x0005
    write_inv_sensor_16(0x3F86, 0x0000);       // mbist_wr_data_fpn_bot                    0x0000
    write_inv_sensor_16(0x3F8A, 0x0000);       // mbist_address_fpn_bot                    0x0000
    write_inv_sensor_16(0x3F8C, 0x137F);       // mbist_depth_fpn_bot                      0x137F     (4991)
    write_inv_sensor_16(0x3F8E, 0x4060);       // mbist_ctrl_fpn_bot                       0x4060
    write_inv_sensor_16(0x3F94, 0x0000);   // mbist_seed_oi                            0x00000005
    write_inv_sensor_16(0x3F96, 0x0005);   // mbist_seed_oi                            0x00000005
    write_inv_sensor_16(0x3F98, 0x0000);   // mbist_wr_data_oi                         0x00000000
    write_inv_sensor_16(0x3F9A, 0x0000);   // mbist_wr_data_oi                         0x00000000
    write_inv_sensor_16(0x3FA0, 0x0000);       // mbist_address_oi                         0x0000
    write_inv_sensor_16(0x3FA2, 0x03FE);       // mbist_depth_oi                           0x03FE     (1022)
    write_inv_sensor_16(0x3FA4, 0xC060);       // mbist_ctrl_oi                            0xC060
    write_inv_sensor(0x3FA8, 0x07);         // mbist_ctrl                               0x07
    // Turn on the pll and enable its clock as the system clock
    write_inv_sensor(0x3110, 0x00);         // pll_ctrl                                 0x0F
    msleep(10);               // 50 ms delay to allow pll to settle after powerup and clock enabling
    
    // Start streaming, NOTE: won't be necessary in the next version of the sensor, but is required for the new modules and the DUT board for now
    write_inv_sensor(0x3000, 0x01);        // mode_select                                0x00

    //reset film
    ////write_inv_sensor(0x34D0, 0x03);
    ////msleep(2000);
    ////write_inv_sensor(0x34D0, 0x00);
    ////msleep(500);
    msleep(250);
    
    //stop streaming and wait for start
    write_inv_sensor(0x3000, 0x00);        // mode_select                                0x00
    
    imgsensor.update_sensor_otp_awb = 0; // Init to 0
    imgsensor.update_sensor_otp_lsc = 0; // Init to 0
}   /*  sensor_init  */


//hzmadd
static void preview_setting(void)   //PreviewSetting
{
    printk("hzmtest start x 2->0, y 2->0 \n");
    // Program the sensor registers
    write_inv_sensor(0x3000, 0x00);        // mode_select - STOP STREAMING 0x00
    msleep(250);                // 50 ms delay to allow pll to settle after powerup and clock enabling
    write_inv_sensor(0x3002, 0x00);         // grouped_parameter_hold                   0x00
    write_inv_sensor(0x3003, 0x01);         // unlock                                   0x00
    write_inv_sensor(0x3004, 0x00);         // corrupted_frame_control                  0x00
    write_inv_sensor(0x3007, 0x4E);         // cfa_config                               0x4E
    write_inv_sensor_16(0x3008, 0x0040);       // pedestal								  0x10
    write_inv_sensor_16(0x300E, 0x0000);       // isp_bypass_ctrl                          0x0000
    write_inv_sensor(0x3014, 0x10);       // cci_slave_addr_0                         0x10
    write_inv_sensor(0x3016, 0x36);       // cci_slave_addr_1                         0x36
    write_inv_sensor(0x3020, 0x01);       // enable_auto_trigger                      0x01
    write_inv_sensor(0x3021, 0x00);       // enable_cci_trigger                       0x00
    write_inv_sensor(0x3023, 0x00);       // enable_external_trigger                  0x00
    write_inv_sensor(0x3024, 0x00);       // external_trigger_type                    0x00
    write_inv_sensor_16(0x3026, 0x0000);       // trigger_delay                            0x0000
    write_inv_sensor(0x3030, 0x00);       // io_ctrl                                  0x00
    write_inv_sensor(0x3042, 0x7F);       // global_interrupt_mask                    0x7F
    write_inv_sensor_16(0x3080, 0x0A0A);       // csi2_data_format                         0x0C0C
    write_inv_sensor(0x3082, 0x03);         // csi2_lane_mode                           0x00
    write_inv_sensor(0x3083, 0x00);         // csi2_vc_id                               0x00
    write_inv_sensor(0x3084, 0x30);         // csi2_10_to_8_dt                          0x30
    write_inv_sensor(0x3085, 0x00);         // csi2_ctrl                                0x00
    write_inv_sensor(0x3086, 0x02);         // csi2_t_hs_prepare                        0x0A
    write_inv_sensor(0x3087, 0x60);         // csi2_t_clk_pre                           0x01
    write_inv_sensor(0x3088, 0x01);         // csi2_t_clk_post                          0x01
    write_inv_sensor(0x3089, 0x01);         // csi2_t_hs_exit                           0x01
    write_inv_sensor_16(0x308D, 0x0000);     // csi2_t_wakeup                            0x000000
    write_inv_sensor(0x308F, 0x00);     // csi2_t_wakeup                            0x000000
    write_inv_sensor(0x3090, 0x00);         // csi2_edl_dt                              0x00
    write_inv_sensor(0x3091, 0x89);         // csi2_trigger_dt                          0x89
    write_inv_sensor(0x3100, 0x03);         // pre_pll_clk_div                          0x01
    write_inv_sensor_16(0x3102, 0x0042);       // pll_multiplier                           0x0001
    write_inv_sensor(0x3104, 0x0C);         // vt_sys_clk_div                           0x01
    write_inv_sensor(0x3105, 0x01);         // vt_pix_clk_div                           0x01
    write_inv_sensor(0x3106, 0x02);         // op_sys_clk_div                           0x01
    write_inv_sensor(0x3107, 0x05);         // op_pix_clk_div                           0x04
    write_inv_sensor(0x3110, 0x01);         // pll_ctrl                                 0x0F
    write_inv_sensor(0x3111, 0x01);         // clk_ctrl1                                0x01
    write_inv_sensor(0x3112, 0x20);         // clk_ctrl0                                0x20
    write_inv_sensor(0x3115, 0x00);         // pll_event_clear                          0x00
    write_inv_sensor(0x3116, 0x03);         // pll_event_mask                           0x03
    write_inv_sensor(0x3117, 0x00);         // clk_ana_delay                            0x00
    write_inv_sensor(0x3118, 0x00);         // clk_cp_delay                             0x00
    write_inv_sensor(0x3119, 0x01);         // cp_divider                               0x01
    write_inv_sensor(0x311A, 0x0F);         // clk_standby_delay                        0x0F       (15)
    write_inv_sensor(0x311B, 0x02);         // csi2_esc_clk_ctrl                        0x02
    write_inv_sensor_16(0x3200, 0x0495);       // frame_length_lines                       0x0BFE     (3070)
    write_inv_sensor_16(0x3202, 0x0BB8);       // line_length_pck                          0x11BC     (4540)
    write_inv_sensor_16(0x3204, 0x0493);       // coarse_integration_time                  0x0001
    write_inv_sensor_16(0x3206, 0x0000);       // fine_integration_time                    0x0000
    write_inv_sensor(0x3208, 0x00);       // image_orientation                        0x00
    write_inv_sensor_16(0x320A, 0x0480);       // x_addr_start                             0x0070     (112)
    write_inv_sensor_16(0x320C, 0x03C4);       // y_addr_start                             0x0004
    write_inv_sensor_16(0x320E, 0x0BFF);       // x_addr_end                               0x100F     (4111)
    write_inv_sensor_16(0x3210, 0x07FB);       // y_addr_end                               0x0BBB     (3003)
    write_inv_sensor_16(0x3212, 0x0780);       // x_output_size                            0x0FA0     (4000)
    write_inv_sensor_16(0x3214, 0x0438);       // y_output_size                            0x0BB8     (3000)
    write_inv_sensor_16(0x3216, 0x0001);       // x_even_inc                               0x0001
    write_inv_sensor_16(0x3218, 0x0001);       // x_odd_inc                                0x0001
    write_inv_sensor_16(0x321A, 0x0001);       // y_even_inc                               0x0001
    write_inv_sensor_16(0x321C, 0x0001);       // y_odd_inc                                0x0001
    write_inv_sensor_16(0x321E, 0x0064);       // x_addr_offset                            0x0064     (100)
    write_inv_sensor_16(0x3220, 0x0064);       // y_addr_offset                            0x0064     (100)
    write_inv_sensor(0x3222, 0x00);       // bin_control                              0x00
    write_inv_sensor_16(0x3226, 0x0002);       // y_even_inc_bin                           0x0002
    write_inv_sensor_16(0x3228, 0x0002);       // y_odd_inc_bin                            0x0002
    write_inv_sensor_16(0x322A, 0x0001);       // x_even_inc_ref                           0x0001
    write_inv_sensor_16(0x322C, 0x0001);       // x_odd_inc_ref                            0x0001
    write_inv_sensor_16(0x322E, 0x0001);       // y_even_inc_ref                           0x0001
    write_inv_sensor_16(0x3230, 0x0001);       // y_odd_inc_ref                            0x0001
    write_inv_sensor_16(0x3232, 0x0004);       // first_ref_row                            0x0004
    write_inv_sensor_16(0x3234, 0x0040);       // num_ref_rows                             0x0044     (68)
    write_inv_sensor_16(0x3236, 0x0004);       // first_ref_col                            0x0004
    write_inv_sensor_16(0x3238, 0x0040);       // num_ref_cols                             0x0044     (68)
    write_inv_sensor(0x323A, 0x00);       // show_ref_rows                            0x00
    write_inv_sensor(0x323C, 0x00);       // show_ref_cols                            0x00
    write_inv_sensor_16(0x323E, 0x0000);       // num_lead_edl_rows                        0x0001
    write_inv_sensor_16(0x3240, 0x0000);       // num_trail_edl_rows                       0x0000
    write_inv_sensor(0x3242, 0x00);       // lead_edl_mode                            0x01
    write_inv_sensor(0x3244, 0x00);       // trail_edl_mode                           0x00
    write_inv_sensor_16(0x3246, 0x0080);       // lead_edl_limit                           0x0080     (128)
    write_inv_sensor_16(0x3248, 0x0080);       // trail_edl_limit                          0x0080     (128)
    write_inv_sensor(0x324A, 0x00);       // show_edl_rows                            0x01
    write_inv_sensor(0x3300, 0xF0);       // gpio_config                              0xFF
    write_inv_sensor(0x3301, 0x00);       // gpio_in_sel                              0x00
    write_inv_sensor(0x3302, 0x0F);       // gpio_out_sel                             0x00
    write_inv_sensor(0x3303, 0x3C);       // gpio_ctrl                                0x00
    write_inv_sensor_16(0x3304, 0x0000);       // gpio1_clk_select                         0x0000
    write_inv_sensor_16(0x3306, 0x0000);       // gpio3_clk_select                         0x0000
    write_inv_sensor_16(0x3308, 0xFFFF);       // gpio1_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x330A, 0xFFF0);       // gpio1_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x330C, 0xFFFF);       // gpio1_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x330E, 0xFFF0);       // gpio1_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3310, 0x0184);       // gpio2_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x3312, 0x01DC);       // gpio2_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x3314, 0xFFFF);       // gpio2_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x3316, 0xFFF0);       // gpio2_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3318, 0xFFFF);       // gpio3_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x331A, 0xFFF0);       // gpio3_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x331C, 0xFFFF);       // gpio3_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x331E, 0xFFF0);       // gpio3_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3320, 0xFFFF);       // gpio4_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x3322, 0xFFF0);       // gpio4_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x3324, 0xFFFF);       // gpio4_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x3326, 0xFFF0);       // gpio4_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3328, 0xFFFF);       // gpio1_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x332A, 0xFFF0);       // gpio1_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x332C, 0xFFFF);       // gpio2_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x332E, 0xFFF0);       // gpio2_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x3330, 0xFFFF);       // gpio3_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x3332, 0xFFF0);       // gpio3_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x3334, 0xFFFF);       // gpio4_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x3336, 0xFFF0);       // gpio4_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x3400, 0x000A);       // tg_control                               0x0000
    write_inv_sensor_16(0x3402, 0x0064);       // col_park_addr                            0x1555     (5461)
    write_inv_sensor_16(0x3404, 0x0064);       // row_park_addr                            0x0FFF     (4095)
    write_inv_sensor(0x3406, 0x00);       // gs_ctrl                                  0x00
    write_inv_sensor_16(0x3408, 0xFFFF);       // gs_open_row                              0xFFFF
    write_inv_sensor_16(0x340A, 0xFFFF);       // gs_open_col                              0xFFFF
    write_inv_sensor_16(0x340C, 0xFFFF);       // gs_close_row                             0xFFFF
    write_inv_sensor_16(0x340E, 0xFFFF);       // gs_close_col                             0xFFFF
    write_inv_sensor_16(0x3422, 0x0007);       // framecnt_range0_end                      0x0007
    write_inv_sensor_16(0x3424, 0x0004);       // framecnt_frame_valid_start               0x0004
    write_inv_sensor_16(0x3426, 0x0004);       // framecnt_frame_valid_end                 0x0004
    write_inv_sensor_16(0x3428, 0xFFFF);       // lp_col_pulse_start                       0xFFFF
    write_inv_sensor_16(0x342A, 0xFFF0);       // lp_col_pulse_end                         0xFFF0
    write_inv_sensor_16(0x342C, 0xFFFF);       // lp_row_pulse_start                       0xFFFF
    write_inv_sensor_16(0x342E, 0xFFF0);       // lp_row_pulse_end                         0xFFF0
    write_inv_sensor(0x3441, 0x37);         // initphase_range0_end                     0x37       (55)
    write_inv_sensor(0x3442, 0xFF);         // initphase_latch_reset_at_startup_start   0x00
    write_inv_sensor(0x3443, 0xFE);         // initphase_latch_reset_at_startup_end     0xFF       (255)
    write_inv_sensor(0x3451, 0x78);         // readphase_range0_end                     0xD7       (215)
    write_inv_sensor(0x3452, 0x04);         // readphase_row_select_pulse_1_start       0x21       (33)
    write_inv_sensor(0x3453, 0x1F);         // readphase_row_select_pulse_1_end         0xAB       (171)
    write_inv_sensor(0x3454, 0x22);         // readphase_row_select_pulse_2_start       0xF0       (240)
    write_inv_sensor(0x3455, 0x77);         // readphase_row_select_pulse_2_end         0xFF       (255)
    write_inv_sensor(0x3456, 0x22);         // readphase_row_reset_pulse_1_start        0x58       (88)
    write_inv_sensor(0x3457, 0x41);         // readphase_row_reset_pulse_1_end          0x75       (117)
    write_inv_sensor(0x3458, 0x62);         // readphase_row_reset_pulse_2_start        0xA8       (168)
    write_inv_sensor(0x3459, 0x76);         // readphase_row_reset_pulse_2_end          0xD0       (208)
    write_inv_sensor(0x345A, 0xFF);         // readphase_en_global_reset_start          0xFF       (255)
    write_inv_sensor(0x345B, 0xFE);         // readphase_en_global_reset_end            0xFE       (254)
    write_inv_sensor(0x345C, 0xFF);         // readphase_latch_reset_pulse1_start       0xD1       (209)
    write_inv_sensor(0x345D, 0xFE);         // readphase_latch_reset_pulse1_end         0xD3       (211)
    write_inv_sensor(0x345E, 0xFF);         // readphase_latch_reset_pulse2_start       0xFF       (255)
    write_inv_sensor(0x345F, 0xFE);         // readphase_latch_reset_pulse2_end         0xFE       (254)
    write_inv_sensor(0x3460, 0xFF);         // readphase_gate_charge_pump_start         0xFF       (255)
    write_inv_sensor(0x3461, 0xFE);         // readphase_gate_charge_pump_end           0xFE       (254)
    write_inv_sensor(0x3462, 0x00);         // readphase_en_all_col_for_sampling_start  0x02
    write_inv_sensor(0x3463, 0x61);         // readphase_en_all_col_for_sampling_end    0xCF       (207)
    write_inv_sensor(0x3464, 0xFF);         // readphase_manual_path_a_enable_start     0x04
    write_inv_sensor(0x3465, 0xFE);         // readphase_manual_path_a_enable_end       0xCD       (205)
    write_inv_sensor(0x3466, 0xFF);         // readphase_manual_path_b_enable_start     0x04
    write_inv_sensor(0x3467, 0xFE);         // readphase_manual_path_b_enable_end       0xCD       (205)
    write_inv_sensor(0x3468, 0x0C);         // readphase_samp_sig_path_a_1_start        0x28       (40)
    write_inv_sensor(0x3469, 0x1E);         // readphase_samp_sig_path_a_1_end          0x4E       (78)
    write_inv_sensor(0x346A, 0xFF);         // readphase_samp_sig_path_a_2_start        0xFF       (255)
    write_inv_sensor(0x346B, 0xFE);         // readphase_samp_sig_path_a_2_end          0xFE       (254)
    write_inv_sensor(0x346C, 0x4E);         // readphase_samp_rst_path_a_start          0x7E       (126)
    write_inv_sensor(0x346D, 0x60);         // readphase_samp_rst_path_a_end            0xA4       (164)
    write_inv_sensor(0x346E, 0x0C);         // readphase_samp_sig_path_b_1_start        0x28       (40)
    write_inv_sensor(0x346F, 0x1E);         // readphase_samp_sig_path_b_1_end          0x4E       (78)
    write_inv_sensor(0x3470, 0xFF);         // readphase_samp_sig_path_b_2_start        0xFF       (255)
    write_inv_sensor(0x3471, 0xFE);         // readphase_samp_sig_path_b_2_end          0xFE       (254)
    write_inv_sensor(0x3472, 0x4E);         // readphase_samp_rst_path_b_start          0x7E       (126)
    write_inv_sensor(0x3473, 0x60);         // readphase_samp_rst_path_b_end            0xA4       (164)
    write_inv_sensor(0x3474, 0xFF);         // readphase_disable_col_bias_1_start       0xFF       (255)
    write_inv_sensor(0x3475, 0xFE);         // readphase_disable_col_bias_1_end         0xFE       (254)
    write_inv_sensor(0x3476, 0xFF);         // readphase_disable_col_bias_2_start       0xFF       (255)
    write_inv_sensor(0x3477, 0xFE);         // readphase_disable_col_bias_2_end         0xFE       (254)
    write_inv_sensor(0x3478, 0xFF);         // readphase_disable_col_bias_3_start       0xFF       (255)
    write_inv_sensor(0x3479, 0xFE);         // readphase_disable_col_bias_3_end         0xFE       (254)
    write_inv_sensor(0x347A, 0x20);         // readphase_en_hard_reset_start            0x50       (80)
    write_inv_sensor(0x347B, 0x34);         // readphase_en_hard_reset_end              0xCF       (207)
    write_inv_sensor(0x347C, 0xFF);         // readphase_en_low_noise_reset_start       0xFF       (255)
    write_inv_sensor(0x347D, 0xFE);         // readphase_en_low_noise_reset_end         0xFE       (254)
    write_inv_sensor(0x347E, 0xFF);         // readphase_set_low_noise_comps_start      0x00
    write_inv_sensor(0x347F, 0xFE);         // readphase_set_low_noise_comps_end        0xFF       (255)
    write_inv_sensor(0x3480, 0xFF);         // readphase_reset_low_noise_comps_start    0xFF       (255)
    write_inv_sensor(0x3481, 0xFE);         // readphase_reset_low_noise_comps_end      0xFE       (254)
    write_inv_sensor(0x3482, 0xFF);         // readphase_en_black_sun_clamp_top_start   0xFF       (255)
    write_inv_sensor(0x3483, 0xFE);         // readphase_en_black_sun_clamp_top_end     0xFE       (254)
    write_inv_sensor(0x3484, 0xFF);         // readphase_en_black_sun_clamp_bot_start   0xFF       (255)
    write_inv_sensor(0x3485, 0xFE);         // readphase_en_black_sun_clamp_bot_end     0xFE       (254)
    write_inv_sensor(0x3486, 0x02);         // readphase_pulse_sf_out_1_start           0x10       (16)
    write_inv_sensor(0x3487, 0x03);         // readphase_pulse_sf_out_1_end             0x1F       (31)
    write_inv_sensor(0x3488, 0x20);         // readphase_pulse_sf_out_2_start           0x8F       (143)
    write_inv_sensor(0x3489, 0x21);         // readphase_pulse_sf_out_2_end             0x9F       (159)
    write_inv_sensor(0x348A, 0xFF);         // readphase_pulse_sf_out_3_start           0xD0       (208)
    write_inv_sensor(0x348B, 0xFE);         // readphase_pulse_sf_out_3_end             0xDF       (223)
    write_inv_sensor(0x348C, 0xFF);         // readphase_inject_sig_at_sf_out_start     0xFF       (255)
    write_inv_sensor(0x348D, 0xFE);         // readphase_inject_sig_at_sf_out_end       0xFE       (254)
    write_inv_sensor(0x348E, 0xFF);         // readphase_inject_rst_at_sf_out_start     0xFF       (255)
    write_inv_sensor(0x348F, 0xFE);         // readphase_inject_rst_at_sf_out_end       0xFE       (254)
    write_inv_sensor_16(0x3490, 0x01E0);       // fine_itime_offset                        0x0050     (80)
    write_inv_sensor(0x3493, 0x78);         // resetphase_range0_end                    0x4F       (79)
    write_inv_sensor(0x3494, 0x04);         // resetphase_row_select_pulse_1_start      0x00
    write_inv_sensor(0x3495, 0x1F);         // resetphase_row_select_pulse_1_end        0xFF       (255)
    write_inv_sensor(0x3496, 0x22);         // resetphase_row_select_pulse_2_start      0xFF       (255)
    write_inv_sensor(0x3497, 0x77);         // resetphase_row_select_pulse_2_end        0xFE       (254)
    write_inv_sensor(0x3498, 0x22);         // resetphase_row_reset_pulse_start         0x25       (37)
    write_inv_sensor(0x3499, 0x41);         // resetphase_row_reset_pulse_end           0x42       (66)
    write_inv_sensor(0x349A, 0xFF);         // resetphase_en_global_reset_start         0xFF       (255)
    write_inv_sensor(0x349B, 0xFE);         // resetphase_en_global_reset_end           0xFE       (254)
    write_inv_sensor(0x349C, 0xFF);         // resetphase_latch_reset_pulse1_start      0x21       (33)
    write_inv_sensor(0x349D, 0xFE);         // resetphase_latch_reset_pulse1_end        0x23       (35)
    write_inv_sensor(0x349E, 0xFF);         // resetphase_latch_reset_pulse2_start      0xFF       (255)
    write_inv_sensor(0x349F, 0xFE);         // resetphase_latch_reset_pulse2_end        0xFE       (254)
    write_inv_sensor(0x34A0, 0xFF);         // resetphase_gate_charge_pump_start        0xFF       (255)
    write_inv_sensor(0x34A1, 0xFE);         // resetphase_gate_charge_pump_end          0xFE       (254)
    write_inv_sensor(0x34A2, 0xFF);         // resetphase_en_all_col_for_sampling_start 0xFF       (255)
    write_inv_sensor(0x34A3, 0xFE);         // resetphase_en_all_col_for_sampling_end   0xFE       (254)
    write_inv_sensor(0x34A4, 0xFF);         // resetphase_samp_sig_path_a_start         0xFF       (255)
    write_inv_sensor(0x34A5, 0xFE);         // resetphase_samp_sig_path_a_end           0xFE       (254)
    write_inv_sensor(0x34A6, 0xFF);         // resetphase_samp_rst_path_a_start         0xFF       (255)
    write_inv_sensor(0x34A7, 0xFE);         // resetphase_samp_rst_path_a_end           0xFE       (254)
    write_inv_sensor(0x34A8, 0xFF);         // resetphase_samp_sig_path_b_start         0xFF       (255)
    write_inv_sensor(0x34A9, 0xFE);         // resetphase_samp_sig_path_b_end           0xFE       (254)
    write_inv_sensor(0x34AA, 0xFF);         // resetphase_samp_rst_path_b_start         0xFF       (255)
    write_inv_sensor(0x34AB, 0xFE);         // resetphase_samp_rst_path_b_end           0xFE       (254)
    write_inv_sensor(0x34AC, 0xFF);         // resetphase_disable_col_bias_1_start      0xFF       (255)
    write_inv_sensor(0x34AD, 0xFE);         // resetphase_disable_col_bias_1_end        0xFE       (254)
    write_inv_sensor(0x34AE, 0xFF);         // resetphase_disable_col_bias_2_start      0xFF       (255)
    write_inv_sensor(0x34AF, 0xFE);         // resetphase_disable_col_bias_2_end        0xFE       (254)
    write_inv_sensor(0x34B0, 0xFF);         // resetphase_disable_col_bias_3_start      0xFF       (255)
    write_inv_sensor(0x34B1, 0xFE);         // resetphase_disable_col_bias_3_end        0xFE       (254)
    write_inv_sensor(0x34B2, 0x20);         // resetphase_en_hard_reset_start           0x50       (80)
    write_inv_sensor(0x34B3, 0x34);         // resetphase_en_hard_reset_end             0x6F       (111)
    write_inv_sensor(0x34B4, 0xFF);         // resetphase_en_low_noise_reset_start      0xFF       (255)
    write_inv_sensor(0x34B5, 0xFE);         // resetphase_en_low_noise_reset_end        0xFE       (254)
    write_inv_sensor(0x34B6, 0xFF);         // resetphase_set_low_noise_comps_start     0x00
    write_inv_sensor(0x34B7, 0xFE);         // resetphase_set_low_noise_comps_end       0xFF       (255)
    write_inv_sensor(0x34B8, 0xFF);         // resetphase_reset_low_noise_comps_start   0xFF       (255)
    write_inv_sensor(0x34B9, 0xFE);         // resetphase_reset_low_noise_comps_end     0xFE       (254)
    write_inv_sensor(0x34BA, 0xFF);         // resetphase_en_black_sun_clamp_top_start  0xFF       (255)
    write_inv_sensor(0x34BB, 0xFE);         // resetphase_en_black_sun_clamp_top_end    0xFE       (254)
    write_inv_sensor(0x34BC, 0xFF);         // resetphase_en_black_sun_clamp_bot_start  0xFF       (255)
    write_inv_sensor(0x34BD, 0xFE);         // resetphase_en_black_sun_clamp_bot_end    0xFE       (254)
    write_inv_sensor(0x34BE, 0x02);         // resetphase_pulse_sf_out_1_start          0xFF       (255)
    write_inv_sensor(0x34BF, 0x03);         // resetphase_pulse_sf_out_1_end            0xFE       (254)
    write_inv_sensor(0x34C0, 0x20);         // resetphase_pulse_sf_out_2_start          0xFF       (255)
    write_inv_sensor(0x34C1, 0x21);         // resetphase_pulse_sf_out_2_end            0xFE       (254)
    write_inv_sensor(0x34C2, 0xFF);         // resetphase_pulse_sf_out_3_start          0xFF       (255)
    write_inv_sensor(0x34C3, 0xFE);         // resetphase_pulse_sf_out_3_end            0xFE       (254)
    write_inv_sensor(0x34C4, 0xFF);         // resetphase_inject_sig_at_sf_out_start    0xFF       (255)
    write_inv_sensor(0x34C5, 0xFE);         // resetphase_inject_sig_at_sf_out_end      0xFE       (254)
    write_inv_sensor(0x34C6, 0xFF);         // resetphase_inject_rst_at_sf_out_start    0xFF       (255)
    write_inv_sensor(0x34C7, 0xFE);         // resetphase_inject_rst_at_sf_out_end      0xFE       (254)
    write_inv_sensor(0x34D0, 0x00);         // tgout_format_global_shutter              0x00
    write_inv_sensor(0x34D1, 0x00);         // tgout_format_global_reset                0x00
    write_inv_sensor(0x34D2, 0x00);         // tgout_format_row_select                  0x00
    write_inv_sensor(0x34D3, 0x00);         // tgout_format_row_reset                   0x00
    write_inv_sensor(0x34D4, 0x00);         // tgout_format_latch_reset_at_startup      0x00
    write_inv_sensor(0x34D5, 0x00);         // tgout_format_latch_reset                 0x00
    write_inv_sensor(0x34D6, 0x00);         // tgout_format_gate_charge_pump            0x00
    write_inv_sensor(0x34D8, 0x00);         // tgout_format_col_switch_left_top         0x00
    write_inv_sensor(0x34D9, 0x00);         // tgout_format_col_switch_left_bot         0x00
    write_inv_sensor(0x34DA, 0x00);         // tgout_format_col_switch_left_dig_top     0x00
    write_inv_sensor(0x34DB, 0x00);         // tgout_format_col_switch_left_dig_bot     0x00
    write_inv_sensor(0x34DC, 0x00);         // tgout_format_en_all_col_for_sampling_top 0x00
    write_inv_sensor(0x34DD, 0x00);         // tgout_format_en_all_col_for_sampling_bot 0x00
    write_inv_sensor(0x34DE, 0x00);         // tgout_format_samp_sig_path_a_top         0x00
    write_inv_sensor(0x34DF, 0x00);         // tgout_format_samp_sig_path_a_bot         0x00
    write_inv_sensor(0x34E0, 0x00);         // tgout_format_samp_sig_path_b_top         0x00
    write_inv_sensor(0x34E1, 0x00);         // tgout_format_samp_sig_path_b_bot         0x00
    write_inv_sensor(0x34E2, 0x00);         // tgout_format_samp_rst_path_a_top         0x00
    write_inv_sensor(0x34E3, 0x00);         // tgout_format_samp_rst_path_a_bot         0x00
    write_inv_sensor(0x34E4, 0x00);         // tgout_format_samp_rst_path_b_top         0x00
    write_inv_sensor(0x34E5, 0x00);         // tgout_format_samp_rst_path_b_bot         0x00
    write_inv_sensor(0x34E6, 0x00);         // tgout_format_disable_col_bias_top        0x00
    write_inv_sensor(0x34E7, 0x00);         // tgout_format_disable_col_bias_bot        0x00
    write_inv_sensor(0x34E8, 0x00);         // tgout_format_en_hard_reset_top           0x00
    write_inv_sensor(0x34E9, 0x00);         // tgout_format_en_hard_reset_bot           0x00
    write_inv_sensor(0x34EA, 0x00);         // tgout_format_en_low_noise_reset_top      0x00
    write_inv_sensor(0x34EB, 0x00);         // tgout_format_en_low_noise_reset_bot      0x00
    write_inv_sensor(0x34EC, 0x00);         // tgout_format_set_low_noise_comps_top     0x00
    write_inv_sensor(0x34ED, 0x00);         // tgout_format_set_low_noise_comps_bot     0x00
    write_inv_sensor(0x34EE, 0x00);         // tgout_format_reset_low_noise_comps_top   0x00
    write_inv_sensor(0x34EF, 0x00);         // tgout_format_reset_low_noise_comps_bot   0x00
    write_inv_sensor(0x34F0, 0x00);         // tgout_format_en_black_sun_clamp_top      0x00
    write_inv_sensor(0x34F1, 0x00);         // tgout_format_en_black_sun_clamp_bot      0x00
    write_inv_sensor(0x34F2, 0x00);         // tgout_format_pulse_sf_out_top            0x00
    write_inv_sensor(0x34F3, 0x00);         // tgout_format_pulse_sf_out_bot            0x00
    write_inv_sensor(0x34F4, 0x00);         // tgout_format_inject_sig_at_sf_out_top    0x00
    write_inv_sensor(0x34F5, 0x00);         // tgout_format_inject_sig_at_sf_out_bot    0x00
    write_inv_sensor(0x34F6, 0x00);         // tgout_format_inject_rst_at_sf_out_top    0x00
    write_inv_sensor(0x34F7, 0x00);         // tgout_format_inject_rst_at_sf_out_bot    0x00
    write_inv_sensor(0x3600, 0x00);         // pd_positive_filmbias_dac                 0x01
    write_inv_sensor(0x3601, 0x7F);         // pos_filmbias_threshold_adjust            0x00
    write_inv_sensor(0x3602, 0x00);         // en_float_filmbias                        0x00
    write_inv_sensor(0x3603, 0x00);         // en_internal_filmbias                     0x00
    write_inv_sensor(0x3604, 0x00);         // pd_rst_noise_dac                         0x01
    write_inv_sensor(0x3605, 0x00);         // rst_noise_threshold_adjust               0x00
    write_inv_sensor(0x3610, 0x00);         // pd_bg_inv_top                            0x01
    write_inv_sensor(0x3611, 0x00);         // pd_bg_inv_bot                            0x01
    write_inv_sensor(0x3612, 0x00);         // pd_iqa_iptat_gen                         0x03
    write_inv_sensor(0x3613, 0x00);         // pd_bias_gen                              0x03
    write_inv_sensor(0x3614, 0x01);         // sel_inv_bg_top                           0x00
    write_inv_sensor(0x3615, 0x01);         // sel_inv_bg_bot                           0x00
    write_inv_sensor(0x3616, 0x01);         // sel_inv_iptat_7p5uA_top                  0x00
    write_inv_sensor(0x3617, 0x01);         // sel_inv_iptat_7p5uA_bot                  0x00
    write_inv_sensor(0x3618, 0x00);         // ictrl_global_bias_top                    0x00
    write_inv_sensor(0x3619, 0x00);         // ictrl_global_bias_bot                    0x00
    write_inv_sensor(0x361A, 0x00);         // vbg_inv_trim_top                         0x00
    write_inv_sensor(0x361B, 0x00);         // vbg_inv_trim_bot                         0x00
    write_inv_sensor(0x3620, 0x00);         // pd_cpump                                 0x01
    write_inv_sensor(0x3621, 0x00);         // ictrl_cpump                              0x00
    write_inv_sensor(0x3622, 0x00);         // en_gnd_filmbias                          0x00
    write_inv_sensor(0x3623, 0xFA);         // prog_cpump                               0x00
    write_inv_sensor(0x3624, 0x00);         // cpump_hyst_ctrl                          0x00
    write_inv_sensor(0x3630, 0x00);         // pd_blacksun_dac                          0x01
    write_inv_sensor(0x3632, 0x00);         // blacksun_threshold_adjust_top            0x00
    write_inv_sensor(0x3633, 0x00);         // blacksun_threshold_adjust_bot            0x00
    write_inv_sensor(0x3640, 0x01);         // pd_adft_out_buf                          0x03
    write_inv_sensor(0x3642, 0x01);         // ictrl_col_bias_top                       0x01
    write_inv_sensor(0x3643, 0x01);         // ictrl_col_bias_bot                       0x01
    write_inv_sensor(0x3644, 0x00);         // ictrl_comp_top                           0x00
    write_inv_sensor(0x3645, 0x00);         // rst_noise_taper_n_adj                    0x00
    write_inv_sensor(0x3646, 0x00);         // ictrl_pullup_top                         0x00
    write_inv_sensor(0x3647, 0x00);         // rst_noise_taper_p_adj                    0x00
    write_inv_sensor(0x3648, 0x00);         // sel_adft_top                             0x00
    write_inv_sensor(0x3649, 0x00);         // sel_adft_bot                             0x00
    write_inv_sensor(0x364A, 0x00);         // adft_1_ch_sel_top                        0x00
    write_inv_sensor(0x364B, 0x00);         // adft_1_ch_sel_bot                        0x00
    write_inv_sensor(0x364E, 0x01);         // set_sf_out_to_gnd_top                    0x00
    write_inv_sensor(0x364F, 0x01);         // set_sf_out_to_gnd_bot                    0x00
    write_inv_sensor(0x3650, 0x00);         // set_sf_out_to_pixpwr_top                 0x00
    write_inv_sensor(0x3651, 0x00);         // set_sf_out_to_pixpwr_bot                 0x00
    write_inv_sensor(0x3652, 0x00);         // monitor_sf_out_top                       0x00
    write_inv_sensor(0x3653, 0x00);         // monitor_sf_out_bot                       0x00
    write_inv_sensor(0x3654, 0x00);         // en_temp_monitor_pad_top                  0x00
    write_inv_sensor(0x3655, 0x00);         // en_temp_monitor_pad_bot                  0x00
    write_inv_sensor(0x3656, 0x00);         // short_hard_reset_top                     0x00
    write_inv_sensor(0x3657, 0x00);         // short_hard_reset_bot                     0x00
    write_inv_sensor(0x3658, 0x02);         // analog_aux_top                           0x00
    write_inv_sensor(0x3659, 0x02);         // analog_aux_bot                           0x00
    write_inv_sensor_16(0x3660, 0xAAAA);       // digital_aux                              0xAAAA
    write_inv_sensor(0x3700, 0x00);         // pd_cds                                   0x03
    write_inv_sensor(0x3701, 0x00);         // pd_cds_vcm                               0x0F
    write_inv_sensor(0x3702, 0x00);         // pd_cds_ref_buf_ctrl                      0x0F
    write_inv_sensor(0x3704, 0x2A);         // ictrl_cds_top                            0x00
    write_inv_sensor(0x3705, 0x2A);         // ictrl_cds_bot                            0x00
    write_inv_sensor(0x3706, 0x02);         // vcmi_trim_top                            0x00
    write_inv_sensor(0x3707, 0x02);         // vcmi_trim_bot                            0x00
    write_inv_sensor(0x3708, 0x00);         // clk_delay_to_cds_top                     0x00
    write_inv_sensor(0x3709, 0x00);         // clk_delay_to_cds_bot                     0x00
    write_inv_sensor(0x370A, 0x03);         // cds_gain_top                             0x00
    write_inv_sensor(0x370B, 0x03);         // cds_gain_bot                             0x00
    write_inv_sensor(0x370C, 0x08);         // cds_ctrl_top                             0x00
    write_inv_sensor(0x370D, 0x08);         // cds_ctrl_bot                             0x00
    write_inv_sensor(0x370E, 0x91);         // cds_control                              0x11
    write_inv_sensor_16(0x3711, 0x0000);     // keep_on_cds_amp_top                      0x000000
    write_inv_sensor(0x3713, 0x00);     // keep_on_cds_amp_top                      0x000000
    write_inv_sensor_16(0x3715, 0x0000);     // keep_on_cds_amp_bot                      0x000000
    write_inv_sensor(0x3717, 0x00);     // keep_on_cds_amp_bot                      0x000000
    write_inv_sensor_16(0x3719, 0x0000);     // keep_off_cds_amp_top                     0x01FFFF
    write_inv_sensor(0x371B, 0x00);     // keep_off_cds_amp_top                     0x01FFFF
    write_inv_sensor_16(0x371D, 0x0000);     // keep_off_cds_amp_bot                     0x01FFFF
    write_inv_sensor(0x371F, 0x00);     // keep_off_cds_amp_bot                     0x01FFFF
    write_inv_sensor(0x3800, 0x88);         // afe_ctrl_0_top                           0x77
    write_inv_sensor(0x3801, 0x88);         // afe_ctrl_0_bot                           0x77
    write_inv_sensor(0x3802, 0x00);         // afe_ctrl_1_top                           0x00
    write_inv_sensor(0x3803, 0x00);         // afe_ctrl_1_bot                           0x00
    write_inv_sensor(0x3804, 0x00);         // afe_ctrl_2_top                           0x00
    write_inv_sensor(0x3805, 0x00);         // afe_ctrl_2_bot                           0x00
    write_inv_sensor(0x3806, 0x00);         // afe_ctrl_3_top                           0x00
    write_inv_sensor(0x3807, 0x00);         // afe_ctrl_3_bot                           0x00
    write_inv_sensor(0x3808, 0x00);         // afe_ctrl_4_top                           0x00
    write_inv_sensor(0x3809, 0x00);         // afe_ctrl_4_bot                           0x00
    write_inv_sensor(0x380A, 0x00);         // afe_ctrl_5_top                           0x00
    write_inv_sensor(0x380B, 0x00);         // afe_ctrl_5_bot                           0x00
    write_inv_sensor(0x380C, 0x00);         // afe_ctrl_6_top                           0x00
    write_inv_sensor(0x380D, 0x00);         // afe_ctrl_6_bot                           0x00
    write_inv_sensor(0x380E, 0x00);         // afe_ctrl_7_top                           0x00
    write_inv_sensor(0x380F, 0x00);         // afe_ctrl_7_bot                           0x00
    write_inv_sensor(0x3810, 0x02);         // afe_ctrl_8_top                           0x00
    write_inv_sensor(0x3811, 0x02);         // afe_ctrl_8_bot                           0x00
    write_inv_sensor(0x3812, 0x00);         // clk_delay_to_adc_top                     0x00
    write_inv_sensor(0x3813, 0x00);         // clk_delay_to_adc_bot                     0x00
    write_inv_sensor(0x3814, 0x0F);         // invert_adc_clk                           0x00
    write_inv_sensor(0x3815, 0x0F);         // analog_chain_latency                     0x11       (17)
    write_inv_sensor(0x3816, 0x00);         // adc_channel_mode                         0x00
    write_inv_sensor(0x3817, 0x00);         // adc_data_capture_control                 0x00
    write_inv_sensor(0x3900, 0x00);         // gain_mode                                0x00
    write_inv_sensor(0x3901, 0x00);         // analog_gain_global                       0x00
    write_inv_sensor(0x3902, 0x00);         // analog_gain_greenR                       0x00
    write_inv_sensor(0x3903, 0x00);         // analog_gain_red                          0x00
    write_inv_sensor(0x3904, 0x00);         // analog_gain_blue                         0x00
    write_inv_sensor(0x3905, 0x00);         // analog_gain_greenB                       0x00
    write_inv_sensor_16(0x3910, 0x0100);       // digital_gain_greenR                      0x0100     (256)
    write_inv_sensor_16(0x3912, 0x0100);       // digital_gain_red                         0x0100     (256)
    write_inv_sensor_16(0x3914, 0x0100);       // digital_gain_blue                        0x0100     (256)
    write_inv_sensor_16(0x3916, 0x0100);       // digital_gain_greenB                      0x0100     (256)
    write_inv_sensor(0x3A00, 0x03);       // blc_mode                                 0x00
    write_inv_sensor_16(0x3A02, 0x0240);       // blc_target                               0x0080     (128)
    write_inv_sensor_16(0x3A04, 0x0010);       // blc_window                               0x0040     (64)
    write_inv_sensor_16(0x3A06, 0x0FFF);       // blc_threshold                            0x0FFF
    write_inv_sensor(0x3A08, 0x1F);       // blc_update_ctrl                          0x01
    write_inv_sensor(0x3A09, 0x14);       // blc_settle_time                          0x14       (20)
    write_inv_sensor(0x3A0A, 0xFF);       // blc_adjust_rate                          0x01
    write_inv_sensor(0x3A0B, 0x04);       // blc_max_rows                             0x00
    write_inv_sensor_16(0x3A0C, 0x0000);       // blc_autostop                             0x0000
    write_inv_sensor_16(0x3A10, 0x01FF);       // blc_dac_ch0_top                          0x0100
    write_inv_sensor_16(0x3A12, 0x01FF);       // blc_dac_ch1_top                          0x0100
    write_inv_sensor_16(0x3A14, 0x01FF);       // blc_dac_ch0_bot                          0x0100
    write_inv_sensor_16(0x3A16, 0x01FF);       // blc_dac_ch1_bot                          0x0100
    write_inv_sensor(0x3A1A, 0x03);       // blc_event_mask                           0x03
    write_inv_sensor(0x3A40, 0x0F);       // rtn_mode                                 0x0E       (14)
    write_inv_sensor_16(0x3A42, 0x0000);       // rtn_target                               0x0080     (128)
    write_inv_sensor_16(0x3A44, 0x0000);       // rtn_min                                  0x0020     (32)
    write_inv_sensor_16(0x3A46, 0x0600);       // rtn_max                                  0x0200     (512)
    write_inv_sensor_16(0x3A48, 0x0000);       // rtn_invalid_cols                         0x0000
    write_inv_sensor_16(0x3A80, 0x602D);       // fpn_mode                                 0x6021
    write_inv_sensor_16(0x3A84, 0x0000);       // fpn_target                               0x0040     (64)
    write_inv_sensor_16(0x3A86, 0x3A00);       // fpn_min                                  0x0010     (16)
    write_inv_sensor_16(0x3A88, 0x0600);       // fpn_max                                  0x0200     (512)
    write_inv_sensor_16(0x3A8A, 0x01FF);       // fpn_initial_step_size                    0x0020     (32)
    write_inv_sensor_16(0x3A8C, 0x0004);       // fpn_num_ref_row_step                     0x0080     (128)
    write_inv_sensor(0x3A92, 0x00);       // fpn_event_clear                          0x00
    write_inv_sensor(0x3A94, 0x03);       // fpn_event_mask                           0x03
    write_inv_sensor_16(0x3A96, 0x0000);       // fpn_mem_status                           0x0000
    write_inv_sensor_16(0x3B00, 0x0000);       // test_pattern_mode                        0x0000
    write_inv_sensor_16(0x3B02, 0x0000);       // test_data_red                            0x0000
    write_inv_sensor_16(0x3B04, 0x0000);       // test_data_greenR                         0x0000
    write_inv_sensor_16(0x3B06, 0x0000);       // test_data_blue                           0x0000
    write_inv_sensor_16(0x3B08, 0x0000);       // test_data_greenB                         0x0000
    write_inv_sensor_16(0x3B0A, 0x0000);       // horizontal_cursor_width                  0x0000
    write_inv_sensor_16(0x3B0C, 0x0000);       // horizontal_cursor_position               0x0000
    write_inv_sensor_16(0x3B0E, 0x0000);       // vertical_cursor_width                    0x0000
    write_inv_sensor_16(0x3B10, 0x0000);       // vertical_cursor_position                 0x0000
    write_inv_sensor_16(0x3D00, 0x0500);       // fifo_water_mark_pixels                   0x0400     (1024)
    write_inv_sensor(0x3D04, 0x00);       // bypass_output_fifo                       0x00
    write_inv_sensor(0x3D07, 0x00);       // output_fifo_event1_clear                 0x00
    write_inv_sensor(0x3D08, 0x07);       // output_fifo_event1_mask                  0x07
    write_inv_sensor(0x3D0B, 0x00);       // output_fifo_event2_clear                 0x00
    write_inv_sensor(0x3D0C, 0x3F);       // output_fifo_event2_mask                  0x3F
    write_inv_sensor_16(0x3E00, 0x0000);       // otp_addr                                 0x0000
    write_inv_sensor(0x3E02, 0x00);       // otp_wr_data                              0x00
    write_inv_sensor(0x3E04, 0x00);       // otp_config                               0x00
    write_inv_sensor(0x3E05, 0x00);       // otp_cmd                                  0x00
    write_inv_sensor_16(0x3E08, 0x0000);       // otp_mr_wr_data                           0x0000
    write_inv_sensor_16(0x3E0A, 0x0000);       // otp_mra_wr_data                          0x0000
    write_inv_sensor_16(0x3E0C, 0x0000);       // otp_mrb_wr_data                          0x0000
    write_inv_sensor_16(0x3E14, 0x0000);       // otp_mr_wr_pgm_rd_data                    0x0000
    write_inv_sensor_16(0x3E16, 0x0000);       // otp_mra_wr_pgm_rd_data                   0x0000
    write_inv_sensor_16(0x3E18, 0x0000);       // otp_mrb_wr_pgm_rd_data                   0x0000
    write_inv_sensor_16(0x3E1A, 0x0000);       // otp_ctl                                  0x0000
    write_inv_sensor_16(0x3E1C, 0x0000);       // otp_prog_pulse_width_cnt                 0x0000
    write_inv_sensor_16(0x3E1E, 0x0000);       // otp_prog_soak_pulse_width_cnt            0x0000
    write_inv_sensor_16(0x3E20, 0x0000);       // otp_prog_recovery_width_cnt              0x0000
    write_inv_sensor_16(0x3E22, 0x0000);       // otp_read_recovery_width_cnt              0x0000
    write_inv_sensor_16(0x3E24, 0x0000);       // otp_other_recovery_width_cnt             0x0000
    write_inv_sensor_16(0x3E26, 0x0000);       // otp_chg_pump_rdy_cnt                     0x0000
    write_inv_sensor_16(0x3E28, 0x0000);       // otp_transaction_cntr_max                 0x0000
    write_inv_sensor(0x3F00, 0x88);       // mbist_rm_fpn_top                         0x88
    write_inv_sensor(0x3F01, 0x88);       // mbist_rm_fpn_bot                         0x88
    write_inv_sensor(0x3F02, 0xDD);       // mbist_rm_oi                              0xDD
    write_inv_sensor_16(0x3F72, 0x0005);       // mbist_seed_fpn_top                       0x0005
    write_inv_sensor_16(0x3F76, 0x0000);       // mbist_wr_data_fpn_top                    0x0000
    write_inv_sensor_16(0x3F7A, 0x0000);       // mbist_address_fpn_top                    0x0000
    write_inv_sensor_16(0x3F7C, 0x137F);       // mbist_depth_fpn_top                      0x137F     (4991)
    write_inv_sensor_16(0x3F7E, 0x4060);       // mbist_ctrl_fpn_top                       0x4060
    write_inv_sensor_16(0x3F82, 0x0005);       // mbist_seed_fpn_bot                       0x0005
    write_inv_sensor_16(0x3F86, 0x0000);       // mbist_wr_data_fpn_bot                    0x0000
    write_inv_sensor_16(0x3F8A, 0x0000);       // mbist_address_fpn_bot                    0x0000
    write_inv_sensor_16(0x3F8C, 0x137F);       // mbist_depth_fpn_bot                      0x137F     (4991)
    write_inv_sensor_16(0x3F8E, 0x4060);       // mbist_ctrl_fpn_bot                       0x4060
    write_inv_sensor_16(0x3F94, 0x0000);   // mbist_seed_oi                            0x00000005
    write_inv_sensor_16(0x3F96, 0x0005);   // mbist_seed_oi                            0x00000005
    write_inv_sensor_16(0x3F98, 0x0000);   // mbist_wr_data_oi                         0x00000000
    write_inv_sensor_16(0x3F9A, 0x0000);   // mbist_wr_data_oi                         0x00000000
    write_inv_sensor_16(0x3FA0, 0x0000);       // mbist_address_oi                         0x0000
    write_inv_sensor_16(0x3FA2, 0x03FE);       // mbist_depth_oi                           0x03FE     (1022)
    write_inv_sensor_16(0x3FA4, 0xC060);       // mbist_ctrl_oi                            0xC060
    write_inv_sensor(0x3FA8, 0x07);         // mbist_ctrl                               0x07
    // Turn on the pll and enable its clock as the system clock
    write_inv_sensor(0x3110, 0x00);         // pll_ctrl                                 0x0F
    msleep(10);               // 50 ms delay to allow pll to settle after powerup and clock enabling
    // Start streaming
    write_inv_sensor(0x3000, 0x01);        // mode_select                                0x00
    // restart CFPN
    write_inv_sensor(0x3A01, 0x01);         // restart BLC
    write_inv_sensor(0x3A82, 0x01);		   // restart CFPN
    ////msleep(250);
}   /*  preview_setting  */

static void capture_setting_15fps(kal_uint16 currefps)  // INV1310MIPI_set_13M
{

}

//hzmadd
static void capture_setting(kal_uint16 currefps)  // INV1310MIPI_set_13M
{
    // Program the sensor registers
    write_inv_sensor(0x3000, 0x00);        // mode_select - STOP STREAMING 0x00
    msleep(250);                // 50 ms delay to allow pll to settle after powerup and clock enabling
    write_inv_sensor(0x3002, 0x00);         // grouped_parameter_hold                   0x00
    write_inv_sensor(0x3003, 0x01);         // unlock                                   0x00
    write_inv_sensor(0x3004, 0x00);         // corrupted_frame_control                  0x00
    write_inv_sensor(0x3007, 0x4E);         // cfa_config                               0x4E
    write_inv_sensor_16(0x3008, 0x0040);       // pedestal								  0x10
    write_inv_sensor_16(0x300E, 0x0000);       // isp_bypass_ctrl                          0x0000
    write_inv_sensor(0x3014, 0x10);         // cci_slave_addr_0                         0x10
    write_inv_sensor(0x3016, 0x36);         // cci_slave_addr_1                         0x36
    write_inv_sensor(0x3020, 0x01);         // enable_auto_trigger                      0x01
    write_inv_sensor(0x3021, 0x00);         // enable_cci_trigger                       0x00
    write_inv_sensor(0x3023, 0x00);         // enable_external_trigger                  0x00
    write_inv_sensor(0x3024, 0x00);         // external_trigger_type                    0x00
    write_inv_sensor_16(0x3026, 0x0000);       // trigger_delay                            0x0000
    write_inv_sensor(0x3030, 0x00);         // io_ctrl                                  0x00
    write_inv_sensor(0x3042, 0x7F);         // global_interrupt_mask                    0x7F
    write_inv_sensor_16(0x3080, 0x0A0A);       // csi2_data_format                         0x0C0C
    write_inv_sensor(0x3082, 0x03);         // csi2_lane_mode                           0x00
    write_inv_sensor(0x3083, 0x00);         // csi2_vc_id                               0x00
    write_inv_sensor(0x3084, 0x30);         // csi2_10_to_8_dt                          0x30
    write_inv_sensor(0x3085, 0x00);         // csi2_ctrl                                0x00
    write_inv_sensor(0x3086, 0x02);         // csi2_t_hs_prepare                        0x0A
    write_inv_sensor(0x3087, 0x60);         // csi2_t_clk_pre                           0x01
    write_inv_sensor(0x3088, 0x01);         // csi2_t_clk_post                          0x01
    write_inv_sensor(0x3089, 0x01);         // csi2_t_hs_exit                           0x01
    write_inv_sensor_16(0x308D, 0x0000);     // csi2_t_wakeup                            0x000000
    write_inv_sensor(0x308F, 0x00);         // csi2_edl_dt                              0x00
    write_inv_sensor(0x3090, 0x00);         // csi2_edl_dt                              0x00
    write_inv_sensor(0x3091, 0x89);         // csi2_trigger_dt                          0x89
    write_inv_sensor(0x3100, 0x03);         // pre_pll_clk_div                          0x01
    write_inv_sensor_16(0x3102, 0x0043);       // pll_multiplier                           0x0001
    write_inv_sensor(0x3104, 0x10);         // vt_sys_clk_div                           0x01
    write_inv_sensor(0x3105, 0x01);         // vt_pix_clk_div                           0x01
    write_inv_sensor(0x3106, 0x02);         // op_sys_clk_div                           0x01
    write_inv_sensor(0x3107, 0x05);         // op_pix_clk_div                           0x04
    write_inv_sensor(0x3110, 0x01);         // pll_ctrl                                 0x0F
    write_inv_sensor(0x3111, 0x01);         // clk_ctrl1                                0x01
    write_inv_sensor(0x3112, 0x20);         // clk_ctrl0                                0x20
    write_inv_sensor(0x3115, 0x00);         // pll_event_clear                          0x00
    write_inv_sensor(0x3116, 0x03);         // pll_event_mask                           0x03
    write_inv_sensor(0x3117, 0x00);         // clk_ana_delay                            0x00
    write_inv_sensor(0x3118, 0x00);         // clk_cp_delay                             0x00
    write_inv_sensor(0x3119, 0x01);         // cp_divider                               0x01
    write_inv_sensor(0x311A, 0x0F);         // clk_standby_delay                        0x0F       (15)
    write_inv_sensor(0x311B, 0x02);         // csi2_esc_clk_ctrl                        0x02
    write_inv_sensor_16(0x3200, 0x0BFD);       // frame_length_lines                       0x0BFE     (3070)
    write_inv_sensor_16(0x3202, 0x12F0);       // line_length_pck                          0x11BC     (4540)
    write_inv_sensor_16(0x3204, 0x0BFB);       // coarse_integration_time                  0x0001
    write_inv_sensor_16(0x3206, 0x0000);       // fine_integration_time                    0x0000
    write_inv_sensor(0x3208, 0x00);         // image_orientation                        0x00
    write_inv_sensor_16(0x320A, 0x0070);       // x_addr_start                             0x0070     (112)
    write_inv_sensor_16(0x320C, 0x0004);       // y_addr_start                             0x0004
    write_inv_sensor_16(0x320E, 0x100F);       // x_addr_end                               0x100F     (4111)
    write_inv_sensor_16(0x3210, 0x0BBB);       // y_addr_end                               0x0BBB     (3003)
    write_inv_sensor_16(0x3212, 0x0FA0);       // x_output_size                            0x0FA0     (4000)
    write_inv_sensor_16(0x3214, 0x0BB8);       // y_output_size                            0x0BB8     (3000)
    write_inv_sensor_16(0x3216, 0x0001);       // x_even_inc                               0x0001
    write_inv_sensor_16(0x3218, 0x0001);       // x_odd_inc                                0x0001
    write_inv_sensor_16(0x321A, 0x0001);       // y_even_inc                               0x0001
    write_inv_sensor_16(0x321C, 0x0001);       // y_odd_inc                                0x0001
    write_inv_sensor_16(0x321E, 0x0064);       // x_addr_offset                            0x0064     (100)
    write_inv_sensor_16(0x3220, 0x0064);       // y_addr_offset                            0x0064     (100)
    write_inv_sensor(0x3222, 0x00);        // bin_control                              0x00
    write_inv_sensor_16(0x3226, 0x0002);       // y_even_inc_bin                           0x0002
    write_inv_sensor_16(0x3228, 0x0002);       // y_odd_inc_bin                            0x0002
    write_inv_sensor_16(0x322A, 0x0001);       // x_even_inc_ref                           0x0001
    write_inv_sensor_16(0x322C, 0x0001);       // x_odd_inc_ref                            0x0001
    write_inv_sensor_16(0x322E, 0x0001);       // y_even_inc_ref                           0x0001
    write_inv_sensor_16(0x3230, 0x0001);       // y_odd_inc_ref                            0x0001
    write_inv_sensor_16(0x3232, 0x0004);       // first_ref_row                            0x0004
    write_inv_sensor_16(0x3234, 0x0040);       // num_ref_rows                             0x0044     (68)
    write_inv_sensor_16(0x3236, 0x0004);       // first_ref_col                            0x0004
    write_inv_sensor_16(0x3238, 0x0040);       // num_ref_cols                             0x0044     (68)
    write_inv_sensor(0x323A, 0x00);         // show_ref_rows                            0x00
    write_inv_sensor(0x323C, 0x00);         // show_ref_cols                            0x00
    write_inv_sensor_16(0x323E, 0x0000);       // num_lead_edl_rows                        0x0001
    write_inv_sensor_16(0x3240, 0x0000);       // num_trail_edl_rows                       0x0000
    write_inv_sensor(0x3242, 0x00);         // lead_edl_mode                            0x01
    write_inv_sensor(0x3244, 0x00);         // trail_edl_mode                           0x00
    write_inv_sensor_16(0x3246, 0x0080);       // lead_edl_limit                           0x0080     (128)
    write_inv_sensor_16(0x3248, 0x0080);       // trail_edl_limit                          0x0080     (128)
    write_inv_sensor(0x324A, 0x00);         // show_edl_rows                            0x01
    write_inv_sensor(0x3300, 0xF0);         // gpio_config                              0xFF
    write_inv_sensor(0x3301, 0x00);         // gpio_in_sel                              0x00
    write_inv_sensor(0x3302, 0x0F);         // gpio_out_sel                             0x00
    write_inv_sensor(0x3303, 0x3C);         // gpio_ctrl                                0x00
    write_inv_sensor_16(0x3304, 0x0000);       // gpio1_clk_select                         0x0000
    write_inv_sensor_16(0x3306, 0x0000);       // gpio3_clk_select                         0x0000
    write_inv_sensor_16(0x3308, 0xFFFF);       // gpio1_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x330A, 0xFFF0);       // gpio1_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x330C, 0xFFFF);       // gpio1_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x330E, 0xFFF0);       // gpio1_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3310, 0x012C);       // gpio2_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x3312, 0x016E);       // gpio2_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x3314, 0xFFFF);       // gpio2_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x3316, 0xFFF0);       // gpio2_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3318, 0xFFFF);       // gpio3_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x331A, 0xFFF0);       // gpio3_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x331C, 0xFFFF);       // gpio3_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x331E, 0xFFF0);       // gpio3_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3320, 0xFFFF);       // gpio4_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x3322, 0xFFF0);       // gpio4_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x3324, 0xFFFF);       // gpio4_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x3326, 0xFFF0);       // gpio4_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3328, 0xFFFF);       // gpio1_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x332A, 0xFFF0);       // gpio1_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x332C, 0xFFFF);       // gpio2_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x332E, 0xFFF0);       // gpio2_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x3330, 0xFFFF);       // gpio3_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x3332, 0xFFF0);       // gpio3_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x3334, 0xFFFF);       // gpio4_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x3336, 0xFFF0);       // gpio4_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x3400, 0x0005);       // tg_control                               0x0000
    write_inv_sensor_16(0x3402, 0x0064);       // col_park_addr                            0x1555     (5461)
    write_inv_sensor_16(0x3404, 0x0064);       // row_park_addr                            0x0FFF     (4095)
    write_inv_sensor(0x3406, 0x00);         // gs_ctrl                                  0x00
    write_inv_sensor_16(0x3408, 0xFFFF);       // gs_open_row                              0xFFFF
    write_inv_sensor_16(0x340A, 0xFFFF);       // gs_open_col                              0xFFFF
    write_inv_sensor_16(0x340C, 0xFFFF);       // gs_close_row                             0xFFFF
    write_inv_sensor_16(0x340E, 0xFFFF);       // gs_close_col                             0xFFFF
    write_inv_sensor_16(0x3422, 0x0007);       // framecnt_range0_end                      0x0007
    write_inv_sensor_16(0x3424, 0x0004);       // framecnt_frame_valid_start               0x0004
    write_inv_sensor_16(0x3426, 0x0004);       // framecnt_frame_valid_end                 0x0004
    write_inv_sensor_16(0x3428, 0xFFFF);       // lp_col_pulse_start                       0xFFFF
    write_inv_sensor_16(0x342A, 0xFFF0);       // lp_col_pulse_end                         0xFFF0
    write_inv_sensor_16(0x342C, 0xFFFF);       // lp_row_pulse_start                       0xFFFF
    write_inv_sensor_16(0x342E, 0xFFF0);       // lp_row_pulse_end                         0xFFF0
    write_inv_sensor(0x3441, 0x37);         // initphase_range0_end                     0x37       (55)
    write_inv_sensor(0x3442, 0xFF);         // initphase_latch_reset_at_startup_start   0x00
    write_inv_sensor(0x3443, 0xFE);         // initphase_latch_reset_at_startup_end     0xFF       (255)
    write_inv_sensor(0x3451, 0xB7);         // readphase_range0_end                     0xD7       (215)
    write_inv_sensor(0x3452, 0x05);         // readphase_row_select_pulse_1_start       0x21       (33)
    write_inv_sensor(0x3453, 0x2D);         // readphase_row_select_pulse_1_end         0xAB       (171)
    write_inv_sensor(0x3454, 0x33);         // readphase_row_select_pulse_2_start       0xF0       (240)
    write_inv_sensor(0x3455, 0xB7);         // readphase_row_select_pulse_2_end         0xFF       (255)
    write_inv_sensor(0x3456, 0x33);         // readphase_row_reset_pulse_1_start        0x58       (88)
    write_inv_sensor(0x3457, 0x63);         // readphase_row_reset_pulse_1_end          0x75       (117)
    write_inv_sensor(0x3458, 0x95);         // readphase_row_reset_pulse_2_start        0xA8       (168)
    write_inv_sensor(0x3459, 0xB5);         // readphase_row_reset_pulse_2_end          0xD0       (208)
    write_inv_sensor(0x345A, 0xFF);         // readphase_en_global_reset_start          0xFF       (255)
    write_inv_sensor(0x345B, 0xFE);         // readphase_en_global_reset_end            0xFE       (254)
    write_inv_sensor(0x345C, 0xFF);         // readphase_latch_reset_pulse1_start       0xD1       (209)
    write_inv_sensor(0x345D, 0xFE);         // readphase_latch_reset_pulse1_end         0xD3       (211)
    write_inv_sensor(0x345E, 0xFF);         // readphase_latch_reset_pulse2_start       0xFF       (255)
    write_inv_sensor(0x345F, 0xFE);         // readphase_latch_reset_pulse2_end         0xFE       (254)
    write_inv_sensor(0x3460, 0xFF);         // readphase_gate_charge_pump_start         0xFF       (255)
    write_inv_sensor(0x3461, 0xFE);         // readphase_gate_charge_pump_end           0xFE       (254)
    write_inv_sensor(0x3462, 0x00);         // readphase_en_all_col_for_sampling_start  0x02
    write_inv_sensor(0x3463, 0x92);         // readphase_en_all_col_for_sampling_end    0xCF       (207)
    write_inv_sensor(0x3464, 0xFF);         // readphase_manual_path_a_enable_start     0x04
    write_inv_sensor(0x3465, 0xFE);         // readphase_manual_path_a_enable_end       0xCD       (205)
    write_inv_sensor(0x3466, 0xFF);         // readphase_manual_path_b_enable_start     0x04
    write_inv_sensor(0x3467, 0xFE);         // readphase_manual_path_b_enable_end       0xCD       (205)
    write_inv_sensor(0x3468, 0x11);         // readphase_samp_sig_path_a_1_start        0x28       (40)
    write_inv_sensor(0x3469, 0x2B);         // readphase_samp_sig_path_a_1_end          0x4E       (78)
    write_inv_sensor(0x346A, 0xFF);         // readphase_samp_sig_path_a_2_start        0xFF       (255)
    write_inv_sensor(0x346B, 0xFE);         // readphase_samp_sig_path_a_2_end          0xFE       (254)
    write_inv_sensor(0x346C, 0x75);         // readphase_samp_rst_path_a_start          0x7E       (126)
    write_inv_sensor(0x346D, 0x91);         // readphase_samp_rst_path_a_end            0xA4       (164)
    write_inv_sensor(0x346E, 0x11);         // readphase_samp_sig_path_b_1_start        0x28       (40)
    write_inv_sensor(0x346F, 0x2B);         // readphase_samp_sig_path_b_1_end          0x4E       (78)
    write_inv_sensor(0x3470, 0xFF);         // readphase_samp_sig_path_b_2_start        0xFF       (255)
    write_inv_sensor(0x3471, 0xFE);         // readphase_samp_sig_path_b_2_end          0xFE       (254)
    write_inv_sensor(0x3472, 0x75);         // readphase_samp_rst_path_b_start          0x7E       (126)
    write_inv_sensor(0x3473, 0x91);         // readphase_samp_rst_path_b_end            0xA4       (164)
    write_inv_sensor(0x3474, 0xFF);         // readphase_disable_col_bias_1_start       0xFF       (255)
    write_inv_sensor(0x3475, 0xFE);         // readphase_disable_col_bias_1_end         0xFE       (254)
    write_inv_sensor(0x3476, 0xFF);         // readphase_disable_col_bias_2_start       0xFF       (255)
    write_inv_sensor(0x3477, 0xFE);         // readphase_disable_col_bias_2_end         0xFE       (254)
    write_inv_sensor(0x3478, 0xFF);         // readphase_disable_col_bias_3_start       0xFF       (255)
    write_inv_sensor(0x3479, 0xFE);         // readphase_disable_col_bias_3_end         0xFE       (254)
    write_inv_sensor(0x347A, 0x2F);         // readphase_en_hard_reset_start            0x50       (80)
    write_inv_sensor(0x347B, 0x4F);         // readphase_en_hard_reset_end              0xCF       (207)
    write_inv_sensor(0x347C, 0xFF);         // readphase_en_low_noise_reset_start       0xFF       (255)
    write_inv_sensor(0x347D, 0xFE);         // readphase_en_low_noise_reset_end         0xFE       (254)
    write_inv_sensor(0x347E, 0xFF);         // readphase_set_low_noise_comps_start      0x00
    write_inv_sensor(0x347F, 0xFE);         // readphase_set_low_noise_comps_end        0xFF       (255)
    write_inv_sensor(0x3480, 0xFF);         // readphase_reset_low_noise_comps_start    0xFF       (255)
    write_inv_sensor(0x3481, 0xFE);         // readphase_reset_low_noise_comps_end      0xFE       (254)
    write_inv_sensor(0x3482, 0xFF);         // readphase_en_black_sun_clamp_top_start   0xFF       (255)
    write_inv_sensor(0x3483, 0xFE);         // readphase_en_black_sun_clamp_top_end     0xFE       (254)
    write_inv_sensor(0x3484, 0xFF);         // readphase_en_black_sun_clamp_bot_start   0xFF       (255)
    write_inv_sensor(0x3485, 0xFE);         // readphase_en_black_sun_clamp_bot_end     0xFE       (254)
    write_inv_sensor(0x3486, 0x02);         // readphase_pulse_sf_out_1_start           0x10       (16)
    write_inv_sensor(0x3487, 0x04);         // readphase_pulse_sf_out_1_end             0x1F       (31)
    write_inv_sensor(0x3488, 0x2F);         // readphase_pulse_sf_out_2_start           0x8F       (143)
    write_inv_sensor(0x3489, 0x31);         // readphase_pulse_sf_out_2_end             0x9F       (159)
    write_inv_sensor(0x348A, 0xFF);         // readphase_pulse_sf_out_3_start           0xD0       (208)
    write_inv_sensor(0x348B, 0xFE);         // readphase_pulse_sf_out_3_end             0xDF       (223)
    write_inv_sensor(0x348C, 0xFF);         // readphase_inject_sig_at_sf_out_start     0xFF       (255)
    write_inv_sensor(0x348D, 0xFE);         // readphase_inject_sig_at_sf_out_end       0xFE       (254)
    write_inv_sensor(0x348E, 0xFF);         // readphase_inject_rst_at_sf_out_start     0xFF       (255)
    write_inv_sensor(0x348F, 0xFE);         // readphase_inject_rst_at_sf_out_end       0xFE       (254)
    write_inv_sensor_16(0x3490, 0x016E);       // fine_itime_offset                        0x0050     (80)
    write_inv_sensor(0x3493, 0xB7);         // resetphase_range0_end                    0x4F       (79)
    write_inv_sensor(0x3494, 0x05);         // resetphase_row_select_pulse_1_start      0x00
    write_inv_sensor(0x3495, 0x2D);         // resetphase_row_select_pulse_1_end        0xFF       (255)
    write_inv_sensor(0x3496, 0x33);         // resetphase_row_select_pulse_2_start      0xFF       (255)
    write_inv_sensor(0x3497, 0xB7);         // resetphase_row_select_pulse_2_end        0xFE       (254)
    write_inv_sensor(0x3498, 0x33);         // resetphase_row_reset_pulse_start         0x25       (37)
    write_inv_sensor(0x3499, 0x63);         // resetphase_row_reset_pulse_end           0x42       (66)
    write_inv_sensor(0x349A, 0xFF);         // resetphase_en_global_reset_start         0xFF       (255)
    write_inv_sensor(0x349B, 0xFE);         // resetphase_en_global_reset_end           0xFE       (254)
    write_inv_sensor(0x349C, 0xFF);         // resetphase_latch_reset_pulse1_start      0x21       (33)
    write_inv_sensor(0x349D, 0xFE);         // resetphase_latch_reset_pulse1_end        0x23       (35)
    write_inv_sensor(0x349E, 0xFF);         // resetphase_latch_reset_pulse2_start      0xFF       (255)
    write_inv_sensor(0x349F, 0xFE);         // resetphase_latch_reset_pulse2_end        0xFE       (254)
    write_inv_sensor(0x34A0, 0xFF);         // resetphase_gate_charge_pump_start        0xFF       (255)
    write_inv_sensor(0x34A1, 0xFE);         // resetphase_gate_charge_pump_end          0xFE       (254)
    write_inv_sensor(0x34A2, 0xFF);         // resetphase_en_all_col_for_sampling_start 0xFF       (255)
    write_inv_sensor(0x34A3, 0xFE);         // resetphase_en_all_col_for_sampling_end   0xFE       (254)
    write_inv_sensor(0x34A4, 0xFF);         // resetphase_samp_sig_path_a_start         0xFF       (255)
    write_inv_sensor(0x34A5, 0xFE);         // resetphase_samp_sig_path_a_end           0xFE       (254)
    write_inv_sensor(0x34A6, 0xFF);         // resetphase_samp_rst_path_a_start         0xFF       (255)
    write_inv_sensor(0x34A7, 0xFE);         // resetphase_samp_rst_path_a_end           0xFE       (254)
    write_inv_sensor(0x34A8, 0xFF);         // resetphase_samp_sig_path_b_start         0xFF       (255)
    write_inv_sensor(0x34A9, 0xFE);         // resetphase_samp_sig_path_b_end           0xFE       (254)
    write_inv_sensor(0x34AA, 0xFF);         // resetphase_samp_rst_path_b_start         0xFF       (255)
    write_inv_sensor(0x34AB, 0xFE);         // resetphase_samp_rst_path_b_end           0xFE       (254)
    write_inv_sensor(0x34AC, 0xFF);         // resetphase_disable_col_bias_1_start      0xFF       (255)
    write_inv_sensor(0x34AD, 0xFE);         // resetphase_disable_col_bias_1_end        0xFE       (254)
    write_inv_sensor(0x34AE, 0xFF);         // resetphase_disable_col_bias_2_start      0xFF       (255)
    write_inv_sensor(0x34AF, 0xFE);         // resetphase_disable_col_bias_2_end        0xFE       (254)
    write_inv_sensor(0x34B0, 0xFF);         // resetphase_disable_col_bias_3_start      0xFF       (255)
    write_inv_sensor(0x34B1, 0xFE);         // resetphase_disable_col_bias_3_end        0xFE       (254)
    write_inv_sensor(0x34B2, 0x2F);         // resetphase_en_hard_reset_start           0x50       (80)
    write_inv_sensor(0x34B3, 0x4F);         // resetphase_en_hard_reset_end             0x6F       (111)
    write_inv_sensor(0x34B4, 0xFF);         // resetphase_en_low_noise_reset_start      0xFF       (255)
    write_inv_sensor(0x34B5, 0xFE);         // resetphase_en_low_noise_reset_end        0xFE       (254)
    write_inv_sensor(0x34B6, 0xFF);         // resetphase_set_low_noise_comps_start     0x00
    write_inv_sensor(0x34B7, 0xFE);         // resetphase_set_low_noise_comps_end       0xFF       (255)
    write_inv_sensor(0x34B8, 0xFF);         // resetphase_reset_low_noise_comps_start   0xFF       (255)
    write_inv_sensor(0x34B9, 0xFE);         // resetphase_reset_low_noise_comps_end     0xFE       (254)
    write_inv_sensor(0x34BA, 0xFF);         // resetphase_en_black_sun_clamp_top_start  0xFF       (255)
    write_inv_sensor(0x34BB, 0xFE);         // resetphase_en_black_sun_clamp_top_end    0xFE       (254)
    write_inv_sensor(0x34BC, 0xFF);         // resetphase_en_black_sun_clamp_bot_start  0xFF       (255)
    write_inv_sensor(0x34BD, 0xFE);         // resetphase_en_black_sun_clamp_bot_end    0xFE       (254)
    write_inv_sensor(0x34BE, 0x02);         // resetphase_pulse_sf_out_1_start          0xFF       (255)
    write_inv_sensor(0x34BF, 0x04);         // resetphase_pulse_sf_out_1_end            0xFE       (254)
    write_inv_sensor(0x34C0, 0x2F);         // resetphase_pulse_sf_out_2_start          0xFF       (255)
    write_inv_sensor(0x34C1, 0x31);         // resetphase_pulse_sf_out_2_end            0xFE       (254)
    write_inv_sensor(0x34C2, 0xFF);         // resetphase_pulse_sf_out_3_start          0xFF       (255)
    write_inv_sensor(0x34C3, 0xFE);         // resetphase_pulse_sf_out_3_end            0xFE       (254)
    write_inv_sensor(0x34C4, 0xFF);         // resetphase_inject_sig_at_sf_out_start    0xFF       (255)
    write_inv_sensor(0x34C5, 0xFE);         // resetphase_inject_sig_at_sf_out_end      0xFE       (254)
    write_inv_sensor(0x34C6, 0xFF);         // resetphase_inject_rst_at_sf_out_start    0xFF       (255)
    write_inv_sensor(0x34C7, 0xFE);         // resetphase_inject_rst_at_sf_out_end      0xFE       (254)
    write_inv_sensor(0x34D0, 0x00);         // tgout_format_global_shutter              0x00
    write_inv_sensor(0x34D1, 0x00);         // tgout_format_global_reset                0x00
    write_inv_sensor(0x34D2, 0x00);         // tgout_format_row_select                  0x00
    write_inv_sensor(0x34D3, 0x00);         // tgout_format_row_reset                   0x00
    write_inv_sensor(0x34D4, 0x00);         // tgout_format_latch_reset_at_startup      0x00
    write_inv_sensor(0x34D5, 0x00);         // tgout_format_latch_reset                 0x00
    write_inv_sensor(0x34D6, 0x00);         // tgout_format_gate_charge_pump            0x00
    write_inv_sensor(0x34D8, 0x00);         // tgout_format_col_switch_left_top         0x00
    write_inv_sensor(0x34D9, 0x00);         // tgout_format_col_switch_left_bot         0x00
    write_inv_sensor(0x34DA, 0x00);         // tgout_format_col_switch_left_dig_top     0x00
    write_inv_sensor(0x34DB, 0x00);         // tgout_format_col_switch_left_dig_bot     0x00
    write_inv_sensor(0x34DC, 0x00);         // tgout_format_en_all_col_for_sampling_top 0x00
    write_inv_sensor(0x34DD, 0x00);         // tgout_format_en_all_col_for_sampling_bot 0x00
    write_inv_sensor(0x34DE, 0x00);         // tgout_format_samp_sig_path_a_top         0x00
    write_inv_sensor(0x34DF, 0x00);         // tgout_format_samp_sig_path_a_bot         0x00
    write_inv_sensor(0x34E0, 0x00);         // tgout_format_samp_sig_path_b_top         0x00
    write_inv_sensor(0x34E1, 0x00);         // tgout_format_samp_sig_path_b_bot         0x00
    write_inv_sensor(0x34E2, 0x00);         // tgout_format_samp_rst_path_a_top         0x00
    write_inv_sensor(0x34E3, 0x00);         // tgout_format_samp_rst_path_a_bot         0x00
    write_inv_sensor(0x34E4, 0x00);         // tgout_format_samp_rst_path_b_top         0x00
    write_inv_sensor(0x34E5, 0x00);         // tgout_format_samp_rst_path_b_bot         0x00
    write_inv_sensor(0x34E6, 0x00);         // tgout_format_disable_col_bias_top        0x00
    write_inv_sensor(0x34E7, 0x00);         // tgout_format_disable_col_bias_bot        0x00
    write_inv_sensor(0x34E8, 0x00);         // tgout_format_en_hard_reset_top           0x00
    write_inv_sensor(0x34E9, 0x00);         // tgout_format_en_hard_reset_bot           0x00
    write_inv_sensor(0x34EA, 0x00);         // tgout_format_en_low_noise_reset_top      0x00
    write_inv_sensor(0x34EB, 0x00);         // tgout_format_en_low_noise_reset_bot      0x00
    write_inv_sensor(0x34EC, 0x00);         // tgout_format_set_low_noise_comps_top     0x00
    write_inv_sensor(0x34ED, 0x00);         // tgout_format_set_low_noise_comps_bot     0x00
    write_inv_sensor(0x34EE, 0x00);         // tgout_format_reset_low_noise_comps_top   0x00
    write_inv_sensor(0x34EF, 0x00);         // tgout_format_reset_low_noise_comps_bot   0x00
    write_inv_sensor(0x34F0, 0x00);         // tgout_format_en_black_sun_clamp_top      0x00
    write_inv_sensor(0x34F1, 0x00);         // tgout_format_en_black_sun_clamp_bot      0x00
    write_inv_sensor(0x34F2, 0x00);         // tgout_format_pulse_sf_out_top            0x00
    write_inv_sensor(0x34F3, 0x00);         // tgout_format_pulse_sf_out_bot            0x00
    write_inv_sensor(0x34F4, 0x00);         // tgout_format_inject_sig_at_sf_out_top    0x00
    write_inv_sensor(0x34F5, 0x00);         // tgout_format_inject_sig_at_sf_out_bot    0x00
    write_inv_sensor(0x34F6, 0x00);         // tgout_format_inject_rst_at_sf_out_top    0x00
    write_inv_sensor(0x34F7, 0x00);         // tgout_format_inject_rst_at_sf_out_bot    0x00
    write_inv_sensor(0x3600, 0x00);         // pd_positive_filmbias_dac                 0x01
    write_inv_sensor(0x3601, 0x7F);         // pos_filmbias_threshold_adjust            0x00
    write_inv_sensor(0x3602, 0x00);         // en_float_filmbias                        0x00
    write_inv_sensor(0x3603, 0x00);         // en_internal_filmbias                     0x00
    write_inv_sensor(0x3604, 0x00);         // pd_rst_noise_dac                         0x01
    write_inv_sensor(0x3605, 0x00);         // rst_noise_threshold_adjust               0x00
    write_inv_sensor(0x3610, 0x00);         // pd_bg_inv_top                            0x01
    write_inv_sensor(0x3611, 0x00);         // pd_bg_inv_bot                            0x01
    write_inv_sensor(0x3612, 0x00);         // pd_iqa_iptat_gen                         0x03
    write_inv_sensor(0x3613, 0x00);         // pd_bias_gen                              0x03
    write_inv_sensor(0x3614, 0x01);         // sel_inv_bg_top                           0x00
    write_inv_sensor(0x3615, 0x01);         // sel_inv_bg_bot                           0x00
    write_inv_sensor(0x3616, 0x01);         // sel_inv_iptat_7p5uA_top                  0x00
    write_inv_sensor(0x3617, 0x01);         // sel_inv_iptat_7p5uA_bot                  0x00
    write_inv_sensor(0x3618, 0x00);         // ictrl_global_bias_top                    0x00
    write_inv_sensor(0x3619, 0x00);         // ictrl_global_bias_bot                    0x00
    write_inv_sensor(0x361A, 0x00);         // vbg_inv_trim_top                         0x00
    write_inv_sensor(0x361B, 0x00);         // vbg_inv_trim_bot                         0x00
    write_inv_sensor(0x3620, 0x00);         // pd_cpump                                 0x01
    write_inv_sensor(0x3621, 0x00);         // ictrl_cpump                              0x00
    write_inv_sensor(0x3622, 0x00);         // en_gnd_filmbias                          0x00
    write_inv_sensor(0x3623, 0xFA);         // prog_cpump                               0x00
    write_inv_sensor(0x3624, 0x00);         // cpump_hyst_ctrl                          0x00
    write_inv_sensor(0x3630, 0x00);         // pd_blacksun_dac                          0x01
    write_inv_sensor(0x3632, 0x00);         // blacksun_threshold_adjust_top            0x00
    write_inv_sensor(0x3633, 0x00);         // blacksun_threshold_adjust_bot            0x00
    write_inv_sensor(0x3640, 0x01);         // pd_adft_out_buf                          0x03
    write_inv_sensor(0x3642, 0x01);         // ictrl_col_bias_top                       0x01
    write_inv_sensor(0x3643, 0x01);         // ictrl_col_bias_bot                       0x01
    write_inv_sensor(0x3644, 0x00);         // ictrl_comp_top                           0x00
    write_inv_sensor(0x3645, 0x00);         // rst_noise_taper_n_adj                    0x00
    write_inv_sensor(0x3646, 0x00);         // ictrl_pullup_top                         0x00
    write_inv_sensor(0x3647, 0x00);         // rst_noise_taper_p_adj                    0x00
    write_inv_sensor(0x3648, 0x00);         // sel_adft_top                             0x00
    write_inv_sensor(0x3649, 0x00);         // sel_adft_bot                             0x00
    write_inv_sensor(0x364A, 0x00);         // adft_1_ch_sel_top                        0x00
    write_inv_sensor(0x364B, 0x00);         // adft_1_ch_sel_bot                        0x00
    write_inv_sensor(0x364E, 0x01);         // set_sf_out_to_gnd_top                    0x00
    write_inv_sensor(0x364F, 0x01);         // set_sf_out_to_gnd_bot                    0x00
    write_inv_sensor(0x3650, 0x00);         // set_sf_out_to_pixpwr_top                 0x00
    write_inv_sensor(0x3651, 0x00);         // set_sf_out_to_pixpwr_bot                 0x00
    write_inv_sensor(0x3652, 0x00);         // monitor_sf_out_top                       0x00
    write_inv_sensor(0x3653, 0x00);         // monitor_sf_out_bot                       0x00
    write_inv_sensor(0x3654, 0x00);         // en_temp_monitor_pad_top                  0x00
    write_inv_sensor(0x3655, 0x00);         // en_temp_monitor_pad_bot                  0x00
    write_inv_sensor(0x3656, 0x00);         // short_hard_reset_top                     0x00
    write_inv_sensor(0x3657, 0x00);         // short_hard_reset_bot                     0x00
    write_inv_sensor(0x3658, 0x02);         // analog_aux_top                           0x00
    write_inv_sensor(0x3659, 0x02);         // analog_aux_bot                           0x00
    write_inv_sensor_16(0x3660, 0xAAAA);       // digital_aux                              0xAAAA
    write_inv_sensor(0x3700, 0x00);         // pd_cds                                   0x03
    write_inv_sensor(0x3701, 0x00);         // pd_cds_vcm                               0x0F
    write_inv_sensor(0x3702, 0x00);         // pd_cds_ref_buf_ctrl                      0x0F
    write_inv_sensor(0x3704, 0x2A);         // ictrl_cds_top                            0x00
    write_inv_sensor(0x3705, 0x2A);         // ictrl_cds_bot                            0x00
    write_inv_sensor(0x3706, 0x02);         // vcmi_trim_top                            0x00
    write_inv_sensor(0x3707, 0x02);         // vcmi_trim_bot                            0x00
    write_inv_sensor(0x3708, 0x00);         // clk_delay_to_cds_top                     0x00
    write_inv_sensor(0x3709, 0x00);         // clk_delay_to_cds_bot                     0x00
    write_inv_sensor(0x370A, 0x03);         // cds_gain_top                             0x00
    write_inv_sensor(0x370B, 0x03);         // cds_gain_bot                             0x00
    write_inv_sensor(0x370C, 0x08);         // cds_ctrl_top                             0x00
    write_inv_sensor(0x370D, 0x08);         // cds_ctrl_bot                             0x00
    write_inv_sensor(0x370E, 0x91);         // cds_control                              0x11
    write_inv_sensor_16(0x3711, 0x0000);     // keep_on_cds_amp_top                      0x000000
    write_inv_sensor(0x3713, 0x00);
    write_inv_sensor_16(0x3715, 0x0000);     // keep_on_cds_amp_bot                      0x000000
    write_inv_sensor(0x3717, 0x00);
    write_inv_sensor_16(0x3719, 0x0000);     // keep_off_cds_amp_top                     0x01FFFF
    write_inv_sensor(0x371B, 0x00);
    write_inv_sensor_16(0x371D, 0x0000);     // keep_off_cds_amp_bot                     0x01FFFF
    write_inv_sensor(0x371F, 0x00);
    write_inv_sensor(0x3800, 0x88);         // afe_ctrl_0_top                           0x77
    write_inv_sensor(0x3801, 0x88);         // afe_ctrl_0_bot                           0x77
    write_inv_sensor(0x3802, 0x00);         // afe_ctrl_1_top                           0x00
    write_inv_sensor(0x3803, 0x00);         // afe_ctrl_1_bot                           0x00
    write_inv_sensor(0x3804, 0x00);         // afe_ctrl_2_top                           0x00
    write_inv_sensor(0x3805, 0x00);         // afe_ctrl_2_bot                           0x00
    write_inv_sensor(0x3806, 0x00);         // afe_ctrl_3_top                           0x00
    write_inv_sensor(0x3807, 0x00);         // afe_ctrl_3_bot                           0x00
    write_inv_sensor(0x3808, 0x00);         // afe_ctrl_4_top                           0x00
    write_inv_sensor(0x3809, 0x00);         // afe_ctrl_4_bot                           0x00
    write_inv_sensor(0x380A, 0x00);         // afe_ctrl_5_top                           0x00
    write_inv_sensor(0x380B, 0x00);         // afe_ctrl_5_bot                           0x00
    write_inv_sensor(0x380C, 0x00);         // afe_ctrl_6_top                           0x00
    write_inv_sensor(0x380D, 0x00);         // afe_ctrl_6_bot                           0x00
    write_inv_sensor(0x380E, 0x00);         // afe_ctrl_7_top                           0x00
    write_inv_sensor(0x380F, 0x00);         // afe_ctrl_7_bot                           0x00
    write_inv_sensor(0x3810, 0x02);         // afe_ctrl_8_top                           0x00
    write_inv_sensor(0x3811, 0x02);         // afe_ctrl_8_bot                           0x00
    write_inv_sensor(0x3812, 0x00);         // clk_delay_to_adc_top                     0x00
    write_inv_sensor(0x3813, 0x00);         // clk_delay_to_adc_bot                     0x00
    write_inv_sensor(0x3814, 0x0F);         // invert_adc_clk                           0x00
    write_inv_sensor(0x3815, 0x0F);         // analog_chain_latency                     0x11       (17)
    write_inv_sensor(0x3816, 0x00);         // adc_channel_mode                         0x00
    write_inv_sensor(0x3817, 0x00);         // adc_data_capture_control                 0x00
    write_inv_sensor(0x3900, 0x00);         // gain_mode                                0x00
    write_inv_sensor(0x3901, 0x00);         // analog_gain_global                       0x00
    write_inv_sensor(0x3902, 0x00);         // analog_gain_greenR                       0x00
    write_inv_sensor(0x3903, 0x00);         // analog_gain_red                          0x00
    write_inv_sensor(0x3904, 0x00);         // analog_gain_blue                         0x00
    write_inv_sensor(0x3905, 0x00);         // analog_gain_greenB                       0x00
    write_inv_sensor_16(0x3910, 0x0100);       // digital_gain_greenR                      0x0100     (256)
    write_inv_sensor_16(0x3912, 0x0100);       // digital_gain_red                         0x0100     (256)
    write_inv_sensor_16(0x3914, 0x0100);       // digital_gain_blue                        0x0100     (256)
    write_inv_sensor_16(0x3916, 0x0100);       // digital_gain_greenB                      0x0100     (256)
    write_inv_sensor(0x3A00, 0x03);            // blc_mode                                 0x00
    write_inv_sensor_16(0x3A02, 0x0240);       // blc_target                               0x0080     (128)
    write_inv_sensor_16(0x3A04, 0x0010);       // blc_window                               0x0040     (64)
    write_inv_sensor_16(0x3A06, 0x0FFF);       // blc_threshold                            0x0FFF
    write_inv_sensor(0x3A08, 0x1F);            // blc_update_ctrl                          0x01
    write_inv_sensor(0x3A09, 0x14);            // blc_settle_time                          0x14       (20)
    write_inv_sensor(0x3A0A, 0xFF);            // blc_adjust_rate                          0x01
    write_inv_sensor(0x3A0B, 0x04);            // blc_max_rows                             0x00
    write_inv_sensor_16(0x3A0C, 0x0000);       // blc_autostop                             0x0000
    write_inv_sensor_16(0x3A10, 0x01FF);       // blc_dac_ch0_top                          0x0100
    write_inv_sensor_16(0x3A12, 0x01FF);       // blc_dac_ch1_top                          0x0100
    write_inv_sensor_16(0x3A14, 0x01FF);       // blc_dac_ch0_bot                          0x0100
    write_inv_sensor_16(0x3A16, 0x01FF);       // blc_dac_ch1_bot                          0x0100
    write_inv_sensor(0x3A1A, 0x03);            // blc_event_mask                           0x03
    write_inv_sensor(0x3A40, 0x0F);            // rtn_mode                                 0x0E       (14)
    write_inv_sensor_16(0x3A42, 0x0000);       // rtn_target                               0x0080     (128)
    write_inv_sensor_16(0x3A44, 0x0000);       // rtn_min                                  0x0020     (32)
    write_inv_sensor_16(0x3A46, 0x0600);       // rtn_max                                  0x0200     (512)
    write_inv_sensor_16(0x3A48, 0x0000);       // rtn_invalid_cols                         0x0000
    write_inv_sensor_16(0x3A80, 0x602D);       // fpn_mode                                 0x6021
    write_inv_sensor_16(0x3A84, 0x0000);       // fpn_target                               0x0040     (64)
    write_inv_sensor_16(0x3A86, 0x3A00);       // fpn_min                                  0x0010     (16)
    write_inv_sensor_16(0x3A88, 0x0600);       // fpn_max                                  0x0200     (512)
    write_inv_sensor_16(0x3A8A, 0x01FF);       // fpn_initial_step_size                    0x0020     (32)
    write_inv_sensor_16(0x3A8C, 0x0004);       // fpn_num_ref_row_step                     0x0080     (128)
    write_inv_sensor(0x3A92, 0x00);            // fpn_event_clear                          0x00
    write_inv_sensor(0x3A94, 0x03);            // fpn_event_mask                           0x03
    write_inv_sensor_16(0x3A96, 0x0000);       // fpn_mem_status                           0x0000
    write_inv_sensor_16(0x3B00, 0x0000);       // test_pattern_mode                        0x0000
    write_inv_sensor_16(0x3B02, 0x0000);       // test_data_red                            0x0000
    write_inv_sensor_16(0x3B04, 0x0000);       // test_data_greenR                         0x0000
    write_inv_sensor_16(0x3B06, 0x0000);       // test_data_blue                           0x0000
    write_inv_sensor_16(0x3B08, 0x0000);       // test_data_greenB                         0x0000
    write_inv_sensor_16(0x3B0A, 0x0000);       // horizontal_cursor_width                  0x0000
    write_inv_sensor_16(0x3B0C, 0x0000);       // horizontal_cursor_position               0x0000
    write_inv_sensor_16(0x3B0E, 0x0000);       // vertical_cursor_width                    0x0000
    write_inv_sensor_16(0x3B10, 0x0000);       // vertical_cursor_position                 0x0000
    write_inv_sensor_16(0x3D00, 0x0500);       // fifo_water_mark_pixels                   0x0400     (1024)
    write_inv_sensor(0x3D04, 0x00);            // bypass_output_fifo                       0x00
    write_inv_sensor(0x3D07, 0x00);            // output_fifo_event1_clear                 0x00
    write_inv_sensor(0x3D08, 0x07);            // output_fifo_event1_mask                  0x07
    write_inv_sensor(0x3D0B, 0x00);            // output_fifo_event2_clear                 0x00
    write_inv_sensor(0x3D0C, 0x3F);            // output_fifo_event2_mask                  0x3F
    write_inv_sensor_16(0x3E00, 0x0000);       // otp_addr                                 0x0000
    write_inv_sensor(0x3E02, 0x00);            // otp_wr_data                              0x00
    write_inv_sensor(0x3E04, 0x00);            // otp_config                               0x00
    write_inv_sensor(0x3E05, 0x00);            // otp_cmd                                  0x00
    write_inv_sensor_16(0x3E08, 0x0000);       // otp_mr_wr_data                           0x0000
    write_inv_sensor_16(0x3E0A, 0x0000);       // otp_mra_wr_data                          0x0000
    write_inv_sensor_16(0x3E0C, 0x0000);       // otp_mrb_wr_data                          0x0000
    write_inv_sensor_16(0x3E14, 0x0000);       // otp_mr_wr_pgm_rd_data                    0x0000
    write_inv_sensor_16(0x3E16, 0x0000);       // otp_mra_wr_pgm_rd_data                   0x0000
    write_inv_sensor_16(0x3E18, 0x0000);       // otp_mrb_wr_pgm_rd_data                   0x0000
    write_inv_sensor_16(0x3E1A, 0x0000);       // otp_ctl                                  0x0000
    write_inv_sensor_16(0x3E1C, 0x0000);       // otp_prog_pulse_width_cnt                 0x0000
    write_inv_sensor_16(0x3E1E, 0x0000);       // otp_prog_soak_pulse_width_cnt            0x0000
    write_inv_sensor_16(0x3E20, 0x0000);       // otp_prog_recovery_width_cnt              0x0000
    write_inv_sensor_16(0x3E22, 0x0000);       // otp_read_recovery_width_cnt              0x0000
    write_inv_sensor_16(0x3E24, 0x0000);       // otp_other_recovery_width_cnt             0x0000
    write_inv_sensor_16(0x3E26, 0x0000);       // otp_chg_pump_rdy_cnt                     0x0000
    write_inv_sensor_16(0x3E28, 0x0000);       // otp_transaction_cntr_max                 0x0000
    write_inv_sensor(0x3F00, 0x88);            // mbist_rm_fpn_top                         0x88
    write_inv_sensor(0x3F01, 0x88);            // mbist_rm_fpn_bot                         0x88
    write_inv_sensor(0x3F02, 0xDD);            // mbist_rm_oi                              0xDD
    write_inv_sensor_16(0x3F72, 0x0005);       // mbist_seed_fpn_top                       0x0005
    write_inv_sensor_16(0x3F76, 0x0000);       // mbist_wr_data_fpn_top                    0x0000
    write_inv_sensor_16(0x3F7A, 0x0000);       // mbist_address_fpn_top                    0x0000
    write_inv_sensor_16(0x3F7C, 0x137F);       // mbist_depth_fpn_top                      0x137F     (4991)
    write_inv_sensor_16(0x3F7E, 0x4060);       // mbist_ctrl_fpn_top                       0x4060
    write_inv_sensor_16(0x3F82, 0x0005);       // mbist_seed_fpn_bot                       0x0005
    write_inv_sensor_16(0x3F86, 0x0000);       // mbist_wr_data_fpn_bot                    0x0000
    write_inv_sensor_16(0x3F8A, 0x0000);       // mbist_address_fpn_bot                    0x0000     
    write_inv_sensor_16(0x3F8C, 0x137F);       // mbist_depth_fpn_bot                      0x137F     (4991)
    write_inv_sensor_16(0x3F8E, 0x4060);       // mbist_ctrl_fpn_bot                       0x4060     
    write_inv_sensor_16(0x3F94, 0x0000);       // mbist_seed_oi                            0x00000005 
    write_inv_sensor_16(0x3F96, 0x0005);
    write_inv_sensor_16(0x3F98, 0x0000);       // mbist_wr_data_oi                         0x00000000 
    write_inv_sensor_16(0x3F9A, 0x0000);
    write_inv_sensor_16(0x3FA0, 0x0000);       // mbist_address_oi                         0x0000     
    write_inv_sensor_16(0x3FA2, 0x03FE);       // mbist_depth_oi                           0x03FE     (1022)
    write_inv_sensor_16(0x3FA4, 0xC060);       // mbist_ctrl_oi                            0xC060     
    write_inv_sensor(0x3FA8, 0x07);         // mbist_ctrl                               0x07       
    // Turn on the pll and enable its clock as the system clock
    write_inv_sensor(0x3110, 0x00);         // pll_ctrl                                 0x0F       
    msleep(10);                // 50 ms delay to allow pll to settle after powerup and clock enabling
    // Start streaming
    write_inv_sensor(0x3000, 0x01);        // mode_select                                0x00
    // restart CFPN
    write_inv_sensor(0x3A01, 0x01);         // restart BLC
    write_inv_sensor(0x3A82, 0x01);		   // restart CFPN
    ////msleep(250);
} //end of cap

static void normal_video_setting(kal_uint16 currefps)    // VideoFullSizeSetting
{
    // Program the write_inv_sensor(0xregisters
    write_inv_sensor(0x3000, 0x00);        // mode_select - STOP STREAMING 0x00
    msleep(250);                // 50 ms delay to allow pll to settle after powerup and clock enabling
    write_inv_sensor(0x3002, 0x00);         // grouped_parameter_hold                   0x00
    write_inv_sensor(0x3003, 0x01);         // unlock                                   0x00
    write_inv_sensor(0x3004, 0x00);         // corrupted_frame_control                  0x00
    write_inv_sensor(0x3007, 0x4E);         // cfa_config                               0x4E
    write_inv_sensor_16(0x3008, 0x0040);       // pedestal								  0x10
    write_inv_sensor_16(0x300E, 0x0000);       // isp_bypass_ctrl                          0x0000
    write_inv_sensor(0x3014, 0x10);       // cci_slave_addr_0                         0x10
    write_inv_sensor(0x3016, 0x36);       // cci_slave_addr_1                         0x36
    write_inv_sensor(0x3020, 0x01);       // enable_auto_trigger                      0x01
    write_inv_sensor(0x3021, 0x00);       // enable_cci_trigger                       0x00
    write_inv_sensor(0x3023, 0x00);       // enable_external_trigger                  0x00
    write_inv_sensor(0x3024, 0x00);       // external_trigger_type                    0x00
    write_inv_sensor_16(0x3026, 0x0000);       // trigger_delay                            0x0000
    write_inv_sensor(0x3030, 0x00);       // io_ctrl                                  0x00
    write_inv_sensor(0x3042, 0x7F);       // global_interrupt_mask                    0x7F
    write_inv_sensor_16(0x3080, 0x0A0A);       // csi2_data_format                         0x0C0C
    write_inv_sensor(0x3082, 0x03);         // csi2_lane_mode                           0x00
    write_inv_sensor(0x3083, 0x00);         // csi2_vc_id                               0x00
    write_inv_sensor(0x3084, 0x30);         // csi2_10_to_8_dt                          0x30
    write_inv_sensor(0x3085, 0x00);         // csi2_ctrl                                0x00
    write_inv_sensor(0x3086, 0x02);         // csi2_t_hs_prepare                        0x0A
    write_inv_sensor(0x3087, 0x60);         // csi2_t_clk_pre                           0x01
    write_inv_sensor(0x3088, 0x01);         // csi2_t_clk_post                          0x01
    write_inv_sensor(0x3089, 0x01);         // csi2_t_hs_exit                           0x01
    write_inv_sensor_16(0x308D, 0x0000);     // csi2_t_wakeup                            0x000000
    write_inv_sensor(0x308F, 0x00);     // csi2_t_wakeup                            0x000000
    write_inv_sensor(0x3090, 0x00);         // csi2_edl_dt                              0x00
    write_inv_sensor(0x3091, 0x89);         // csi2_trigger_dt                          0x89
    write_inv_sensor(0x3100, 0x03);         // pre_pll_clk_div                          0x01
    write_inv_sensor_16(0x3102, 0x0042);       // pll_multiplier                           0x0001
    write_inv_sensor(0x3104, 0x0C);         // vt_sys_clk_div                           0x01
    write_inv_sensor(0x3105, 0x01);         // vt_pix_clk_div                           0x01
    write_inv_sensor(0x3106, 0x02);         // op_sys_clk_div                           0x01
    write_inv_sensor(0x3107, 0x05);         // op_pix_clk_div                           0x04
    write_inv_sensor(0x3110, 0x01);         // pll_ctrl                                 0x0F
    write_inv_sensor(0x3111, 0x01);         // clk_ctrl1                                0x01
    write_inv_sensor(0x3112, 0x20);         // clk_ctrl0                                0x20
    write_inv_sensor(0x3115, 0x00);         // pll_event_clear                          0x00
    write_inv_sensor(0x3116, 0x03);         // pll_event_mask                           0x03
    write_inv_sensor(0x3117, 0x00);         // clk_ana_delay                            0x00
    write_inv_sensor(0x3118, 0x00);         // clk_cp_delay                             0x00
    write_inv_sensor(0x3119, 0x01);         // cp_divider                               0x01
    write_inv_sensor(0x311A, 0x0F);         // clk_standby_delay                        0x0F       (15)
    write_inv_sensor(0x311B, 0x02);         // csi2_esc_clk_ctrl                        0x02
    write_inv_sensor_16(0x3200, 0x0495);       // frame_length_lines                       0x0BFE     (3070)
    write_inv_sensor_16(0x3202, 0x0BB8);       // line_length_pck                          0x11BC     (4540)
    write_inv_sensor_16(0x3204, 0x0493);       // coarse_integration_time                  0x0001
    write_inv_sensor_16(0x3206, 0x0000);       // fine_integration_time                    0x0000
    write_inv_sensor(0x3208, 0x00);       // image_orientation                        0x00
    write_inv_sensor_16(0x320A, 0x0480);       // x_addr_start                             0x0070     (112)
    write_inv_sensor_16(0x320C, 0x03C4);       // y_addr_start                             0x0004
    write_inv_sensor_16(0x320E, 0x0BFF);       // x_addr_end                               0x100F     (4111)
    write_inv_sensor_16(0x3210, 0x07FB);       // y_addr_end                               0x0BBB     (3003)
    write_inv_sensor_16(0x3212, 0x0780);       // x_output_size                            0x0FA0     (4000)
    write_inv_sensor_16(0x3214, 0x0438);       // y_output_size                            0x0BB8     (3000)
    write_inv_sensor_16(0x3216, 0x0001);       // x_even_inc                               0x0001
    write_inv_sensor_16(0x3218, 0x0001);       // x_odd_inc                                0x0001
    write_inv_sensor_16(0x321A, 0x0001);       // y_even_inc                               0x0001
    write_inv_sensor_16(0x321C, 0x0001);       // y_odd_inc                                0x0001
    write_inv_sensor_16(0x321E, 0x0064);       // x_addr_offset                            0x0064     (100)
    write_inv_sensor_16(0x3220, 0x0064);       // y_addr_offset                            0x0064     (100)
    write_inv_sensor(0x3222, 0x00);       // bin_control                              0x00
    write_inv_sensor_16(0x3226, 0x0002);       // y_even_inc_bin                           0x0002
    write_inv_sensor_16(0x3228, 0x0002);       // y_odd_inc_bin                            0x0002
    write_inv_sensor_16(0x322A, 0x0001);       // x_even_inc_ref                           0x0001
    write_inv_sensor_16(0x322C, 0x0001);       // x_odd_inc_ref                            0x0001
    write_inv_sensor_16(0x322E, 0x0001);       // y_even_inc_ref                           0x0001
    write_inv_sensor_16(0x3230, 0x0001);       // y_odd_inc_ref                            0x0001
    write_inv_sensor_16(0x3232, 0x0004);       // first_ref_row                            0x0004
    write_inv_sensor_16(0x3234, 0x0040);       // num_ref_rows                             0x0044     (68)
    write_inv_sensor_16(0x3236, 0x0004);       // first_ref_col                            0x0004
    write_inv_sensor_16(0x3238, 0x0040);       // num_ref_cols                             0x0044     (68)
    write_inv_sensor(0x323A, 0x00);       // show_ref_rows                            0x00
    write_inv_sensor(0x323C, 0x00);       // show_ref_cols                            0x00
    write_inv_sensor_16(0x323E, 0x0000);       // num_lead_edl_rows                        0x0001
    write_inv_sensor_16(0x3240, 0x0000);       // num_trail_edl_rows                       0x0000
    write_inv_sensor(0x3242, 0x00);       // lead_edl_mode                            0x01
    write_inv_sensor(0x3244, 0x00);       // trail_edl_mode                           0x00
    write_inv_sensor_16(0x3246, 0x0080);       // lead_edl_limit                           0x0080     (128)
    write_inv_sensor_16(0x3248, 0x0080);       // trail_edl_limit                          0x0080     (128)
    write_inv_sensor(0x324A, 0x00);       // show_edl_rows                            0x01
    write_inv_sensor(0x3300, 0xF0);       // gpio_config                              0xFF
    write_inv_sensor(0x3301, 0x00);       // gpio_in_sel                              0x00
    write_inv_sensor(0x3302, 0x0F);       // gpio_out_sel                             0x00
    write_inv_sensor(0x3303, 0x3C);       // gpio_ctrl                                0x00
    write_inv_sensor_16(0x3304, 0x0000);       // gpio1_clk_select                         0x0000
    write_inv_sensor_16(0x3306, 0x0000);       // gpio3_clk_select                         0x0000
    write_inv_sensor_16(0x3308, 0xFFFF);       // gpio1_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x330A, 0xFFF0);       // gpio1_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x330C, 0xFFFF);       // gpio1_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x330E, 0xFFF0);       // gpio1_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3310, 0x0184);       // gpio2_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x3312, 0x01DC);       // gpio2_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x3314, 0xFFFF);       // gpio2_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x3316, 0xFFF0);       // gpio2_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3318, 0xFFFF);       // gpio3_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x331A, 0xFFF0);       // gpio3_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x331C, 0xFFFF);       // gpio3_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x331E, 0xFFF0);       // gpio3_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3320, 0xFFFF);       // gpio4_col_pulse1_start                   0xFFFF
    write_inv_sensor_16(0x3322, 0xFFF0);       // gpio4_col_pulse1_end                     0xFFF0
    write_inv_sensor_16(0x3324, 0xFFFF);       // gpio4_col_pulse2_start                   0xFFFF
    write_inv_sensor_16(0x3326, 0xFFF0);       // gpio4_col_pulse2_end                     0xFFF0
    write_inv_sensor_16(0x3328, 0xFFFF);       // gpio1_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x332A, 0xFFF0);       // gpio1_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x332C, 0xFFFF);       // gpio2_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x332E, 0xFFF0);       // gpio2_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x3330, 0xFFFF);       // gpio3_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x3332, 0xFFF0);       // gpio3_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x3334, 0xFFFF);       // gpio4_row_pulse_start                    0xFFFF
    write_inv_sensor_16(0x3336, 0xFFF0);       // gpio4_row_pulse_end                      0xFFF0
    write_inv_sensor_16(0x3400, 0x000A);       // tg_control                               0x0000
    write_inv_sensor_16(0x3402, 0x0064);       // col_park_addr                            0x1555     (5461)
    write_inv_sensor_16(0x3404, 0x0064);       // row_park_addr                            0x0FFF     (4095)
    write_inv_sensor(0x3406, 0x00);       // gs_ctrl                                  0x00
    write_inv_sensor_16(0x3408, 0xFFFF);       // gs_open_row                              0xFFFF
    write_inv_sensor_16(0x340A, 0xFFFF);       // gs_open_col                              0xFFFF
    write_inv_sensor_16(0x340C, 0xFFFF);       // gs_close_row                             0xFFFF
    write_inv_sensor_16(0x340E, 0xFFFF);       // gs_close_col                             0xFFFF
    write_inv_sensor_16(0x3422, 0x0007);       // framecnt_range0_end                      0x0007
    write_inv_sensor_16(0x3424, 0x0004);       // framecnt_frame_valid_start               0x0004
    write_inv_sensor_16(0x3426, 0x0004);       // framecnt_frame_valid_end                 0x0004
    write_inv_sensor_16(0x3428, 0xFFFF);       // lp_col_pulse_start                       0xFFFF
    write_inv_sensor_16(0x342A, 0xFFF0);       // lp_col_pulse_end                         0xFFF0
    write_inv_sensor_16(0x342C, 0xFFFF);       // lp_row_pulse_start                       0xFFFF
    write_inv_sensor_16(0x342E, 0xFFF0);       // lp_row_pulse_end                         0xFFF0
    write_inv_sensor(0x3441, 0x37);         // initphase_range0_end                     0x37       (55)
    write_inv_sensor(0x3442, 0xFF);         // initphase_latch_reset_at_startup_start   0x00
    write_inv_sensor(0x3443, 0xFE);         // initphase_latch_reset_at_startup_end     0xFF       (255)
    write_inv_sensor(0x3451, 0x78);         // readphase_range0_end                     0xD7       (215)
    write_inv_sensor(0x3452, 0x04);         // readphase_row_select_pulse_1_start       0x21       (33)
    write_inv_sensor(0x3453, 0x1F);         // readphase_row_select_pulse_1_end         0xAB       (171)
    write_inv_sensor(0x3454, 0x22);         // readphase_row_select_pulse_2_start       0xF0       (240)
    write_inv_sensor(0x3455, 0x77);         // readphase_row_select_pulse_2_end         0xFF       (255)
    write_inv_sensor(0x3456, 0x22);         // readphase_row_reset_pulse_1_start        0x58       (88)
    write_inv_sensor(0x3457, 0x41);         // readphase_row_reset_pulse_1_end          0x75       (117)
    write_inv_sensor(0x3458, 0x62);         // readphase_row_reset_pulse_2_start        0xA8       (168)
    write_inv_sensor(0x3459, 0x76);         // readphase_row_reset_pulse_2_end          0xD0       (208)
    write_inv_sensor(0x345A, 0xFF);         // readphase_en_global_reset_start          0xFF       (255)
    write_inv_sensor(0x345B, 0xFE);         // readphase_en_global_reset_end            0xFE       (254)
    write_inv_sensor(0x345C, 0xFF);         // readphase_latch_reset_pulse1_start       0xD1       (209)
    write_inv_sensor(0x345D, 0xFE);         // readphase_latch_reset_pulse1_end         0xD3       (211)
    write_inv_sensor(0x345E, 0xFF);         // readphase_latch_reset_pulse2_start       0xFF       (255)
    write_inv_sensor(0x345F, 0xFE);         // readphase_latch_reset_pulse2_end         0xFE       (254)
    write_inv_sensor(0x3460, 0xFF);         // readphase_gate_charge_pump_start         0xFF       (255)
    write_inv_sensor(0x3461, 0xFE);         // readphase_gate_charge_pump_end           0xFE       (254)
    write_inv_sensor(0x3462, 0x00);         // readphase_en_all_col_for_sampling_start  0x02
    write_inv_sensor(0x3463, 0x61);         // readphase_en_all_col_for_sampling_end    0xCF       (207)
    write_inv_sensor(0x3464, 0xFF);         // readphase_manual_path_a_enable_start     0x04
    write_inv_sensor(0x3465, 0xFE);         // readphase_manual_path_a_enable_end       0xCD       (205)
    write_inv_sensor(0x3466, 0xFF);         // readphase_manual_path_b_enable_start     0x04
    write_inv_sensor(0x3467, 0xFE);         // readphase_manual_path_b_enable_end       0xCD       (205)
    write_inv_sensor(0x3468, 0x0C);         // readphase_samp_sig_path_a_1_start        0x28       (40)
    write_inv_sensor(0x3469, 0x1E);         // readphase_samp_sig_path_a_1_end          0x4E       (78)
    write_inv_sensor(0x346A, 0xFF);         // readphase_samp_sig_path_a_2_start        0xFF       (255)
    write_inv_sensor(0x346B, 0xFE);         // readphase_samp_sig_path_a_2_end          0xFE       (254)
    write_inv_sensor(0x346C, 0x4E);         // readphase_samp_rst_path_a_start          0x7E       (126)
    write_inv_sensor(0x346D, 0x60);         // readphase_samp_rst_path_a_end            0xA4       (164)
    write_inv_sensor(0x346E, 0x0C);         // readphase_samp_sig_path_b_1_start        0x28       (40)
    write_inv_sensor(0x346F, 0x1E);         // readphase_samp_sig_path_b_1_end          0x4E       (78)
    write_inv_sensor(0x3470, 0xFF);         // readphase_samp_sig_path_b_2_start        0xFF       (255)
    write_inv_sensor(0x3471, 0xFE);         // readphase_samp_sig_path_b_2_end          0xFE       (254)
    write_inv_sensor(0x3472, 0x4E);         // readphase_samp_rst_path_b_start          0x7E       (126)
    write_inv_sensor(0x3473, 0x60);         // readphase_samp_rst_path_b_end            0xA4       (164)
    write_inv_sensor(0x3474, 0xFF);         // readphase_disable_col_bias_1_start       0xFF       (255)
    write_inv_sensor(0x3475, 0xFE);         // readphase_disable_col_bias_1_end         0xFE       (254)
    write_inv_sensor(0x3476, 0xFF);         // readphase_disable_col_bias_2_start       0xFF       (255)
    write_inv_sensor(0x3477, 0xFE);         // readphase_disable_col_bias_2_end         0xFE       (254)
    write_inv_sensor(0x3478, 0xFF);         // readphase_disable_col_bias_3_start       0xFF       (255)
    write_inv_sensor(0x3479, 0xFE);         // readphase_disable_col_bias_3_end         0xFE       (254)
    write_inv_sensor(0x347A, 0x20);         // readphase_en_hard_reset_start            0x50       (80)
    write_inv_sensor(0x347B, 0x34);         // readphase_en_hard_reset_end              0xCF       (207)
    write_inv_sensor(0x347C, 0xFF);         // readphase_en_low_noise_reset_start       0xFF       (255)
    write_inv_sensor(0x347D, 0xFE);         // readphase_en_low_noise_reset_end         0xFE       (254)
    write_inv_sensor(0x347E, 0xFF);         // readphase_set_low_noise_comps_start      0x00
    write_inv_sensor(0x347F, 0xFE);         // readphase_set_low_noise_comps_end        0xFF       (255)
    write_inv_sensor(0x3480, 0xFF);         // readphase_reset_low_noise_comps_start    0xFF       (255)
    write_inv_sensor(0x3481, 0xFE);         // readphase_reset_low_noise_comps_end      0xFE       (254)
    write_inv_sensor(0x3482, 0xFF);         // readphase_en_black_sun_clamp_top_start   0xFF       (255)
    write_inv_sensor(0x3483, 0xFE);         // readphase_en_black_sun_clamp_top_end     0xFE       (254)
    write_inv_sensor(0x3484, 0xFF);         // readphase_en_black_sun_clamp_bot_start   0xFF       (255)
    write_inv_sensor(0x3485, 0xFE);         // readphase_en_black_sun_clamp_bot_end     0xFE       (254)
    write_inv_sensor(0x3486, 0x02);         // readphase_pulse_sf_out_1_start           0x10       (16)
    write_inv_sensor(0x3487, 0x03);         // readphase_pulse_sf_out_1_end             0x1F       (31)
    write_inv_sensor(0x3488, 0x20);         // readphase_pulse_sf_out_2_start           0x8F       (143)
    write_inv_sensor(0x3489, 0x21);         // readphase_pulse_sf_out_2_end             0x9F       (159)
    write_inv_sensor(0x348A, 0xFF);         // readphase_pulse_sf_out_3_start           0xD0       (208)
    write_inv_sensor(0x348B, 0xFE);         // readphase_pulse_sf_out_3_end             0xDF       (223)
    write_inv_sensor(0x348C, 0xFF);         // readphase_inject_sig_at_sf_out_start     0xFF       (255)
    write_inv_sensor(0x348D, 0xFE);         // readphase_inject_sig_at_sf_out_end       0xFE       (254)
    write_inv_sensor(0x348E, 0xFF);         // readphase_inject_rst_at_sf_out_start     0xFF       (255)
    write_inv_sensor(0x348F, 0xFE);         // readphase_inject_rst_at_sf_out_end       0xFE       (254)
    write_inv_sensor_16(0x3490, 0x01E0);       // fine_itime_offset                        0x0050     (80)
    write_inv_sensor(0x3493, 0x78);         // resetphase_range0_end                    0x4F       (79)
    write_inv_sensor(0x3494, 0x04);         // resetphase_row_select_pulse_1_start      0x00
    write_inv_sensor(0x3495, 0x1F);         // resetphase_row_select_pulse_1_end        0xFF       (255)
    write_inv_sensor(0x3496, 0x22);         // resetphase_row_select_pulse_2_start      0xFF       (255)
    write_inv_sensor(0x3497, 0x77);         // resetphase_row_select_pulse_2_end        0xFE       (254)
    write_inv_sensor(0x3498, 0x22);         // resetphase_row_reset_pulse_start         0x25       (37)
    write_inv_sensor(0x3499, 0x41);         // resetphase_row_reset_pulse_end           0x42       (66)
    write_inv_sensor(0x349A, 0xFF);         // resetphase_en_global_reset_start         0xFF       (255)
    write_inv_sensor(0x349B, 0xFE);         // resetphase_en_global_reset_end           0xFE       (254)
    write_inv_sensor(0x349C, 0xFF);         // resetphase_latch_reset_pulse1_start      0x21       (33)
    write_inv_sensor(0x349D, 0xFE);         // resetphase_latch_reset_pulse1_end        0x23       (35)
    write_inv_sensor(0x349E, 0xFF);         // resetphase_latch_reset_pulse2_start      0xFF       (255)
    write_inv_sensor(0x349F, 0xFE);         // resetphase_latch_reset_pulse2_end        0xFE       (254)
    write_inv_sensor(0x34A0, 0xFF);         // resetphase_gate_charge_pump_start        0xFF       (255)
    write_inv_sensor(0x34A1, 0xFE);         // resetphase_gate_charge_pump_end          0xFE       (254)
    write_inv_sensor(0x34A2, 0xFF);         // resetphase_en_all_col_for_sampling_start 0xFF       (255)
    write_inv_sensor(0x34A3, 0xFE);         // resetphase_en_all_col_for_sampling_end   0xFE       (254)
    write_inv_sensor(0x34A4, 0xFF);         // resetphase_samp_sig_path_a_start         0xFF       (255)
    write_inv_sensor(0x34A5, 0xFE);         // resetphase_samp_sig_path_a_end           0xFE       (254)
    write_inv_sensor(0x34A6, 0xFF);         // resetphase_samp_rst_path_a_start         0xFF       (255)
    write_inv_sensor(0x34A7, 0xFE);         // resetphase_samp_rst_path_a_end           0xFE       (254)
    write_inv_sensor(0x34A8, 0xFF);         // resetphase_samp_sig_path_b_start         0xFF       (255)
    write_inv_sensor(0x34A9, 0xFE);         // resetphase_samp_sig_path_b_end           0xFE       (254)
    write_inv_sensor(0x34AA, 0xFF);         // resetphase_samp_rst_path_b_start         0xFF       (255)
    write_inv_sensor(0x34AB, 0xFE);         // resetphase_samp_rst_path_b_end           0xFE       (254)
    write_inv_sensor(0x34AC, 0xFF);         // resetphase_disable_col_bias_1_start      0xFF       (255)
    write_inv_sensor(0x34AD, 0xFE);         // resetphase_disable_col_bias_1_end        0xFE       (254)
    write_inv_sensor(0x34AE, 0xFF);         // resetphase_disable_col_bias_2_start      0xFF       (255)
    write_inv_sensor(0x34AF, 0xFE);         // resetphase_disable_col_bias_2_end        0xFE       (254)
    write_inv_sensor(0x34B0, 0xFF);         // resetphase_disable_col_bias_3_start      0xFF       (255)
    write_inv_sensor(0x34B1, 0xFE);         // resetphase_disable_col_bias_3_end        0xFE       (254)
    write_inv_sensor(0x34B2, 0x20);         // resetphase_en_hard_reset_start           0x50       (80)
    write_inv_sensor(0x34B3, 0x34);         // resetphase_en_hard_reset_end             0x6F       (111)
    write_inv_sensor(0x34B4, 0xFF);         // resetphase_en_low_noise_reset_start      0xFF       (255)
    write_inv_sensor(0x34B5, 0xFE);         // resetphase_en_low_noise_reset_end        0xFE       (254)
    write_inv_sensor(0x34B6, 0xFF);         // resetphase_set_low_noise_comps_start     0x00
    write_inv_sensor(0x34B7, 0xFE);         // resetphase_set_low_noise_comps_end       0xFF       (255)
    write_inv_sensor(0x34B8, 0xFF);         // resetphase_reset_low_noise_comps_start   0xFF       (255)
    write_inv_sensor(0x34B9, 0xFE);         // resetphase_reset_low_noise_comps_end     0xFE       (254)
    write_inv_sensor(0x34BA, 0xFF);         // resetphase_en_black_sun_clamp_top_start  0xFF       (255)
    write_inv_sensor(0x34BB, 0xFE);         // resetphase_en_black_sun_clamp_top_end    0xFE       (254)
    write_inv_sensor(0x34BC, 0xFF);         // resetphase_en_black_sun_clamp_bot_start  0xFF       (255)
    write_inv_sensor(0x34BD, 0xFE);         // resetphase_en_black_sun_clamp_bot_end    0xFE       (254)
    write_inv_sensor(0x34BE, 0x02);         // resetphase_pulse_sf_out_1_start          0xFF       (255)
    write_inv_sensor(0x34BF, 0x03);         // resetphase_pulse_sf_out_1_end            0xFE       (254)
    write_inv_sensor(0x34C0, 0x20);         // resetphase_pulse_sf_out_2_start          0xFF       (255)
    write_inv_sensor(0x34C1, 0x21);         // resetphase_pulse_sf_out_2_end            0xFE       (254)
    write_inv_sensor(0x34C2, 0xFF);         // resetphase_pulse_sf_out_3_start          0xFF       (255)
    write_inv_sensor(0x34C3, 0xFE);         // resetphase_pulse_sf_out_3_end            0xFE       (254)
    write_inv_sensor(0x34C4, 0xFF);         // resetphase_inject_sig_at_sf_out_start    0xFF       (255)
    write_inv_sensor(0x34C5, 0xFE);         // resetphase_inject_sig_at_sf_out_end      0xFE       (254)
    write_inv_sensor(0x34C6, 0xFF);         // resetphase_inject_rst_at_sf_out_start    0xFF       (255)
    write_inv_sensor(0x34C7, 0xFE);         // resetphase_inject_rst_at_sf_out_end      0xFE       (254)
    write_inv_sensor(0x34D0, 0x00);         // tgout_format_global_shutter              0x00
    write_inv_sensor(0x34D1, 0x00);         // tgout_format_global_reset                0x00
    write_inv_sensor(0x34D2, 0x00);         // tgout_format_row_select                  0x00
    write_inv_sensor(0x34D3, 0x00);         // tgout_format_row_reset                   0x00
    write_inv_sensor(0x34D4, 0x00);         // tgout_format_latch_reset_at_startup      0x00
    write_inv_sensor(0x34D5, 0x00);         // tgout_format_latch_reset                 0x00
    write_inv_sensor(0x34D6, 0x00);         // tgout_format_gate_charge_pump            0x00
    write_inv_sensor(0x34D8, 0x00);         // tgout_format_col_switch_left_top         0x00
    write_inv_sensor(0x34D9, 0x00);         // tgout_format_col_switch_left_bot         0x00
    write_inv_sensor(0x34DA, 0x00);         // tgout_format_col_switch_left_dig_top     0x00
    write_inv_sensor(0x34DB, 0x00);         // tgout_format_col_switch_left_dig_bot     0x00
    write_inv_sensor(0x34DC, 0x00);         // tgout_format_en_all_col_for_sampling_top 0x00
    write_inv_sensor(0x34DD, 0x00);         // tgout_format_en_all_col_for_sampling_bot 0x00
    write_inv_sensor(0x34DE, 0x00);         // tgout_format_samp_sig_path_a_top         0x00
    write_inv_sensor(0x34DF, 0x00);         // tgout_format_samp_sig_path_a_bot         0x00
    write_inv_sensor(0x34E0, 0x00);         // tgout_format_samp_sig_path_b_top         0x00
    write_inv_sensor(0x34E1, 0x00);         // tgout_format_samp_sig_path_b_bot         0x00
    write_inv_sensor(0x34E2, 0x00);         // tgout_format_samp_rst_path_a_top         0x00
    write_inv_sensor(0x34E3, 0x00);         // tgout_format_samp_rst_path_a_bot         0x00
    write_inv_sensor(0x34E4, 0x00);         // tgout_format_samp_rst_path_b_top         0x00
    write_inv_sensor(0x34E5, 0x00);         // tgout_format_samp_rst_path_b_bot         0x00
    write_inv_sensor(0x34E6, 0x00);         // tgout_format_disable_col_bias_top        0x00
    write_inv_sensor(0x34E7, 0x00);         // tgout_format_disable_col_bias_bot        0x00
    write_inv_sensor(0x34E8, 0x00);         // tgout_format_en_hard_reset_top           0x00
    write_inv_sensor(0x34E9, 0x00);         // tgout_format_en_hard_reset_bot           0x00
    write_inv_sensor(0x34EA, 0x00);         // tgout_format_en_low_noise_reset_top      0x00
    write_inv_sensor(0x34EB, 0x00);         // tgout_format_en_low_noise_reset_bot      0x00
    write_inv_sensor(0x34EC, 0x00);         // tgout_format_set_low_noise_comps_top     0x00
    write_inv_sensor(0x34ED, 0x00);         // tgout_format_set_low_noise_comps_bot     0x00
    write_inv_sensor(0x34EE, 0x00);         // tgout_format_reset_low_noise_comps_top   0x00
    write_inv_sensor(0x34EF, 0x00);         // tgout_format_reset_low_noise_comps_bot   0x00
    write_inv_sensor(0x34F0, 0x00);         // tgout_format_en_black_sun_clamp_top      0x00
    write_inv_sensor(0x34F1, 0x00);         // tgout_format_en_black_sun_clamp_bot      0x00
    write_inv_sensor(0x34F2, 0x00);         // tgout_format_pulse_sf_out_top            0x00
    write_inv_sensor(0x34F3, 0x00);         // tgout_format_pulse_sf_out_bot            0x00
    write_inv_sensor(0x34F4, 0x00);         // tgout_format_inject_sig_at_sf_out_top    0x00
    write_inv_sensor(0x34F5, 0x00);         // tgout_format_inject_sig_at_sf_out_bot    0x00
    write_inv_sensor(0x34F6, 0x00);         // tgout_format_inject_rst_at_sf_out_top    0x00
    write_inv_sensor(0x34F7, 0x00);         // tgout_format_inject_rst_at_sf_out_bot    0x00
    write_inv_sensor(0x3600, 0x00);         // pd_positive_filmbias_dac                 0x01
    write_inv_sensor(0x3601, 0x7F);         // pos_filmbias_threshold_adjust            0x00
    write_inv_sensor(0x3602, 0x00);         // en_float_filmbias                        0x00
    write_inv_sensor(0x3603, 0x00);         // en_internal_filmbias                     0x00
    write_inv_sensor(0x3604, 0x00);         // pd_rst_noise_dac                         0x01
    write_inv_sensor(0x3605, 0x00);         // rst_noise_threshold_adjust               0x00
    write_inv_sensor(0x3610, 0x00);         // pd_bg_inv_top                            0x01
    write_inv_sensor(0x3611, 0x00);         // pd_bg_inv_bot                            0x01
    write_inv_sensor(0x3612, 0x00);         // pd_iqa_iptat_gen                         0x03
    write_inv_sensor(0x3613, 0x00);         // pd_bias_gen                              0x03
    write_inv_sensor(0x3614, 0x01);         // sel_inv_bg_top                           0x00
    write_inv_sensor(0x3615, 0x01);         // sel_inv_bg_bot                           0x00
    write_inv_sensor(0x3616, 0x01);         // sel_inv_iptat_7p5uA_top                  0x00
    write_inv_sensor(0x3617, 0x01);         // sel_inv_iptat_7p5uA_bot                  0x00
    write_inv_sensor(0x3618, 0x00);         // ictrl_global_bias_top                    0x00
    write_inv_sensor(0x3619, 0x00);         // ictrl_global_bias_bot                    0x00
    write_inv_sensor(0x361A, 0x00);         // vbg_inv_trim_top                         0x00
    write_inv_sensor(0x361B, 0x00);         // vbg_inv_trim_bot                         0x00
    write_inv_sensor(0x3620, 0x00);         // pd_cpump                                 0x01
    write_inv_sensor(0x3621, 0x00);         // ictrl_cpump                              0x00
    write_inv_sensor(0x3622, 0x00);         // en_gnd_filmbias                          0x00
    write_inv_sensor(0x3623, 0xFA);         // prog_cpump                               0x00
    write_inv_sensor(0x3624, 0x00);         // cpump_hyst_ctrl                          0x00
    write_inv_sensor(0x3630, 0x00);         // pd_blacksun_dac                          0x01
    write_inv_sensor(0x3632, 0x00);         // blacksun_threshold_adjust_top            0x00
    write_inv_sensor(0x3633, 0x00);         // blacksun_threshold_adjust_bot            0x00
    write_inv_sensor(0x3640, 0x01);         // pd_adft_out_buf                          0x03
    write_inv_sensor(0x3642, 0x01);         // ictrl_col_bias_top                       0x01
    write_inv_sensor(0x3643, 0x01);         // ictrl_col_bias_bot                       0x01
    write_inv_sensor(0x3644, 0x00);         // ictrl_comp_top                           0x00
    write_inv_sensor(0x3645, 0x00);         // rst_noise_taper_n_adj                    0x00
    write_inv_sensor(0x3646, 0x00);         // ictrl_pullup_top                         0x00
    write_inv_sensor(0x3647, 0x00);         // rst_noise_taper_p_adj                    0x00
    write_inv_sensor(0x3648, 0x00);         // sel_adft_top                             0x00
    write_inv_sensor(0x3649, 0x00);         // sel_adft_bot                             0x00
    write_inv_sensor(0x364A, 0x00);         // adft_1_ch_sel_top                        0x00
    write_inv_sensor(0x364B, 0x00);         // adft_1_ch_sel_bot                        0x00
    write_inv_sensor(0x364E, 0x01);         // set_sf_out_to_gnd_top                    0x00
    write_inv_sensor(0x364F, 0x01);         // set_sf_out_to_gnd_bot                    0x00
    write_inv_sensor(0x3650, 0x00);         // set_sf_out_to_pixpwr_top                 0x00
    write_inv_sensor(0x3651, 0x00);         // set_sf_out_to_pixpwr_bot                 0x00
    write_inv_sensor(0x3652, 0x00);         // monitor_sf_out_top                       0x00
    write_inv_sensor(0x3653, 0x00);         // monitor_sf_out_bot                       0x00
    write_inv_sensor(0x3654, 0x00);         // en_temp_monitor_pad_top                  0x00
    write_inv_sensor(0x3655, 0x00);         // en_temp_monitor_pad_bot                  0x00
    write_inv_sensor(0x3656, 0x00);         // short_hard_reset_top                     0x00
    write_inv_sensor(0x3657, 0x00);         // short_hard_reset_bot                     0x00
    write_inv_sensor(0x3658, 0x02);         // analog_aux_top                           0x00
    write_inv_sensor(0x3659, 0x02);         // analog_aux_bot                           0x00
    write_inv_sensor_16(0x3660, 0xAAAA);       // digital_aux                              0xAAAA
    write_inv_sensor(0x3700, 0x00);         // pd_cds                                   0x03
    write_inv_sensor(0x3701, 0x00);         // pd_cds_vcm                               0x0F
    write_inv_sensor(0x3702, 0x00);         // pd_cds_ref_buf_ctrl                      0x0F
    write_inv_sensor(0x3704, 0x2A);         // ictrl_cds_top                            0x00
    write_inv_sensor(0x3705, 0x2A);         // ictrl_cds_bot                            0x00
    write_inv_sensor(0x3706, 0x02);         // vcmi_trim_top                            0x00
    write_inv_sensor(0x3707, 0x02);         // vcmi_trim_bot                            0x00
    write_inv_sensor(0x3708, 0x00);         // clk_delay_to_cds_top                     0x00
    write_inv_sensor(0x3709, 0x00);         // clk_delay_to_cds_bot                     0x00
    write_inv_sensor(0x370A, 0x03);         // cds_gain_top                             0x00
    write_inv_sensor(0x370B, 0x03);         // cds_gain_bot                             0x00
    write_inv_sensor(0x370C, 0x08);         // cds_ctrl_top                             0x00
    write_inv_sensor(0x370D, 0x08);         // cds_ctrl_bot                             0x00
    write_inv_sensor(0x370E, 0x91);         // cds_control                              0x11
    write_inv_sensor_16(0x3711, 0x0000);     // keep_on_cds_amp_top                      0x000000
    write_inv_sensor(0x3713, 0x00);     // keep_on_cds_amp_top                      0x000000
    write_inv_sensor_16(0x3715, 0x0000);     // keep_on_cds_amp_bot                      0x000000
    write_inv_sensor(0x3717, 0x00);     // keep_on_cds_amp_bot                      0x000000
    write_inv_sensor_16(0x3719, 0x0000);     // keep_off_cds_amp_top                     0x01FFFF
    write_inv_sensor(0x3721, 0x00);     // keep_off_cds_amp_top                     0x01FFFF
    write_inv_sensor_16(0x371D, 0x0000);     // keep_off_cds_amp_bot                     0x01FFFF
    write_inv_sensor(0x371F, 0x00);     // keep_off_cds_amp_bot                     0x01FFFF
    write_inv_sensor(0x3800, 0x88);         // afe_ctrl_0_top                           0x77
    write_inv_sensor(0x3801, 0x88);         // afe_ctrl_0_bot                           0x77
    write_inv_sensor(0x3802, 0x00);         // afe_ctrl_1_top                           0x00
    write_inv_sensor(0x3803, 0x00);         // afe_ctrl_1_bot                           0x00
    write_inv_sensor(0x3804, 0x00);         // afe_ctrl_2_top                           0x00
    write_inv_sensor(0x3805, 0x00);         // afe_ctrl_2_bot                           0x00
    write_inv_sensor(0x3806, 0x00);         // afe_ctrl_3_top                           0x00
    write_inv_sensor(0x3807, 0x00);         // afe_ctrl_3_bot                           0x00
    write_inv_sensor(0x3808, 0x00);         // afe_ctrl_4_top                           0x00
    write_inv_sensor(0x3809, 0x00);         // afe_ctrl_4_bot                           0x00
    write_inv_sensor(0x380A, 0x00);         // afe_ctrl_5_top                           0x00
    write_inv_sensor(0x380B, 0x00);         // afe_ctrl_5_bot                           0x00
    write_inv_sensor(0x380C, 0x00);         // afe_ctrl_6_top                           0x00
    write_inv_sensor(0x380D, 0x00);         // afe_ctrl_6_bot                           0x00
    write_inv_sensor(0x380E, 0x00);         // afe_ctrl_7_top                           0x00
    write_inv_sensor(0x380F, 0x00);         // afe_ctrl_7_bot                           0x00
    write_inv_sensor(0x3810, 0x02);         // afe_ctrl_8_top                           0x00
    write_inv_sensor(0x3811, 0x02);         // afe_ctrl_8_bot                           0x00
    write_inv_sensor(0x3812, 0x00);         // clk_delay_to_adc_top                     0x00
    write_inv_sensor(0x3813, 0x00);         // clk_delay_to_adc_bot                     0x00
    write_inv_sensor(0x3814, 0x0F);         // invert_adc_clk                           0x00
    write_inv_sensor(0x3815, 0x0F);         // analog_chain_latency                     0x11       (17)
    write_inv_sensor(0x3816, 0x00);         // adc_channel_mode                         0x00
    write_inv_sensor(0x3817, 0x00);         // adc_data_capture_control                 0x00
    write_inv_sensor(0x3900, 0x00);         // gain_mode                                0x00
    write_inv_sensor(0x3901, 0x00);         // analog_gain_global                       0x00
    write_inv_sensor(0x3902, 0x00);         // analog_gain_greenR                       0x00
    write_inv_sensor(0x3903, 0x00);         // analog_gain_red                          0x00
    write_inv_sensor(0x3904, 0x00);         // analog_gain_blue                         0x00
    write_inv_sensor(0x3905, 0x00);         // analog_gain_greenB                       0x00
    write_inv_sensor_16(0x3910, 0x0100);       // digital_gain_greenR                      0x0100     (256)
    write_inv_sensor_16(0x3912, 0x0100);       // digital_gain_red                         0x0100     (256)
    write_inv_sensor_16(0x3914, 0x0100);       // digital_gain_blue                        0x0100     (256)
    write_inv_sensor_16(0x3916, 0x0100);       // digital_gain_greenB                      0x0100     (256)
    write_inv_sensor(0x3A00, 0x03);       // blc_mode                                 0x00
    write_inv_sensor_16(0x3A02, 0x0240);       // blc_target                               0x0080     (128)
    write_inv_sensor_16(0x3A04, 0x0010);       // blc_window                               0x0040     (64)
    write_inv_sensor_16(0x3A06, 0x0FFF);       // blc_threshold                            0x0FFF
    write_inv_sensor(0x3A08, 0x1F);       // blc_update_ctrl                          0x01
    write_inv_sensor(0x3A09, 0x14);       // blc_settle_time                          0x14       (20)
    write_inv_sensor(0x3A0A, 0xFF);       // blc_adjust_rate                          0x01
    write_inv_sensor(0x3A0B, 0x04);       // blc_max_rows                             0x00
    write_inv_sensor_16(0x3A0C, 0x0000);       // blc_autostop                             0x0000
    write_inv_sensor_16(0x3A10, 0x01FF);       // blc_dac_ch0_top                          0x0100
    write_inv_sensor_16(0x3A12, 0x01FF);       // blc_dac_ch1_top                          0x0100
    write_inv_sensor_16(0x3A14, 0x01FF);       // blc_dac_ch0_bot                          0x0100
    write_inv_sensor_16(0x3A16, 0x01FF);       // blc_dac_ch1_bot                          0x0100
    write_inv_sensor(0x3A1A, 0x03);       // blc_event_mask                           0x03
    write_inv_sensor(0x3A40, 0x0F);       // rtn_mode                                 0x0E       (14)
    write_inv_sensor_16(0x3A42, 0x0000);       // rtn_target                               0x0080     (128)
    write_inv_sensor_16(0x3A44, 0x0000);       // rtn_min                                  0x0020     (32)
    write_inv_sensor_16(0x3A46, 0x0600);       // rtn_max                                  0x0200     (512)
    write_inv_sensor_16(0x3A48, 0x0000);       // rtn_invalid_cols                         0x0000
    write_inv_sensor_16(0x3A80, 0x602D);       // fpn_mode                                 0x6021
    write_inv_sensor_16(0x3A84, 0x0000);       // fpn_target                               0x0040     (64)
    write_inv_sensor_16(0x3A86, 0x3A00);       // fpn_min                                  0x0010     (16)
    write_inv_sensor_16(0x3A88, 0x0600);       // fpn_max                                  0x0200     (512)
    write_inv_sensor_16(0x3A8A, 0x01FF);       // fpn_initial_step_size                    0x0020     (32)
    write_inv_sensor_16(0x3A8C, 0x0004);       // fpn_num_ref_row_step                     0x0080     (128)
    write_inv_sensor(0x3A92, 0x00);       // fpn_event_clear                          0x00
    write_inv_sensor(0x3A94, 0x03);       // fpn_event_mask                           0x03
    write_inv_sensor_16(0x3A96, 0x0000);       // fpn_mem_status                           0x0000
    write_inv_sensor_16(0x3B00, 0x0000);       // test_pattern_mode                        0x0000
    write_inv_sensor_16(0x3B02, 0x0000);       // test_data_red                            0x0000
    write_inv_sensor_16(0x3B04, 0x0000);       // test_data_greenR                         0x0000
    write_inv_sensor_16(0x3B06, 0x0000);       // test_data_blue                           0x0000
    write_inv_sensor_16(0x3B08, 0x0000);       // test_data_greenB                         0x0000
    write_inv_sensor_16(0x3B0A, 0x0000);       // horizontal_cursor_width                  0x0000
    write_inv_sensor_16(0x3B0C, 0x0000);       // horizontal_cursor_position               0x0000
    write_inv_sensor_16(0x3B0E, 0x0000);       // vertical_cursor_width                    0x0000
    write_inv_sensor_16(0x3B10, 0x0000);       // vertical_cursor_position                 0x0000
    write_inv_sensor_16(0x3D00, 0x0500);       // fifo_water_mark_pixels                   0x0400     (1024)
    write_inv_sensor(0x3D04, 0x00);       // bypass_output_fifo                       0x00
    write_inv_sensor(0x3D07, 0x00);       // output_fifo_event1_clear                 0x00
    write_inv_sensor(0x3D08, 0x07);       // output_fifo_event1_mask                  0x07
    write_inv_sensor(0x3D0B, 0x00);       // output_fifo_event2_clear                 0x00
    write_inv_sensor(0x3D0C, 0x3F);       // output_fifo_event2_mask                  0x3F
    write_inv_sensor_16(0x3E00, 0x0000);       // otp_addr                                 0x0000
    write_inv_sensor(0x3E02, 0x00);       // otp_wr_data                              0x00
    write_inv_sensor(0x3E04, 0x00);       // otp_config                               0x00
    write_inv_sensor(0x3E05, 0x00);       // otp_cmd                                  0x00
    write_inv_sensor_16(0x3E08, 0x0000);       // otp_mr_wr_data                           0x0000
    write_inv_sensor_16(0x3E0A, 0x0000);       // otp_mra_wr_data                          0x0000
    write_inv_sensor_16(0x3E0C, 0x0000);       // otp_mrb_wr_data                          0x0000
    write_inv_sensor_16(0x3E14, 0x0000);       // otp_mr_wr_pgm_rd_data                    0x0000
    write_inv_sensor_16(0x3E16, 0x0000);       // otp_mra_wr_pgm_rd_data                   0x0000
    write_inv_sensor_16(0x3E18, 0x0000);       // otp_mrb_wr_pgm_rd_data                   0x0000
    write_inv_sensor_16(0x3E1A, 0x0000);       // otp_ctl                                  0x0000
    write_inv_sensor_16(0x3E1C, 0x0000);       // otp_prog_pulse_width_cnt                 0x0000
    write_inv_sensor_16(0x3E1E, 0x0000);       // otp_prog_soak_pulse_width_cnt            0x0000
    write_inv_sensor_16(0x3E20, 0x0000);       // otp_prog_recovery_width_cnt              0x0000
    write_inv_sensor_16(0x3E22, 0x0000);       // otp_read_recovery_width_cnt              0x0000
    write_inv_sensor_16(0x3E24, 0x0000);       // otp_other_recovery_width_cnt             0x0000
    write_inv_sensor_16(0x3E26, 0x0000);       // otp_chg_pump_rdy_cnt                     0x0000
    write_inv_sensor_16(0x3E28, 0x0000);       // otp_transaction_cntr_max                 0x0000
    write_inv_sensor(0x3F00, 0x88);       // mbist_rm_fpn_top                         0x88
    write_inv_sensor(0x3F01, 0x88);       // mbist_rm_fpn_bot                         0x88
    write_inv_sensor(0x3F02, 0xDD);       // mbist_rm_oi                              0xDD
    write_inv_sensor_16(0x3F72, 0x0005);       // mbist_seed_fpn_top                       0x0005
    write_inv_sensor_16(0x3F76, 0x0000);       // mbist_wr_data_fpn_top                    0x0000
    write_inv_sensor_16(0x3F7A, 0x0000);       // mbist_address_fpn_top                    0x0000
    write_inv_sensor_16(0x3F7C, 0x137F);       // mbist_depth_fpn_top                      0x137F     (4991)
    write_inv_sensor_16(0x3F7E, 0x4060);       // mbist_ctrl_fpn_top                       0x4060
    write_inv_sensor_16(0x3F82, 0x0005);       // mbist_seed_fpn_bot                       0x0005
    write_inv_sensor_16(0x3F86, 0x0000);       // mbist_wr_data_fpn_bot                    0x0000
    write_inv_sensor_16(0x3F8A, 0x0000);       // mbist_address_fpn_bot                    0x0000
    write_inv_sensor_16(0x3F8C, 0x137F);       // mbist_depth_fpn_bot                      0x137F     (4991)
    write_inv_sensor_16(0x3F8E, 0x4060);       // mbist_ctrl_fpn_bot                       0x4060
    write_inv_sensor_16(0x3F94, 0x0000);   // mbist_seed_oi                            0x00000005
    write_inv_sensor_16(0x3F94, 0x0005);   // mbist_seed_oi                            0x00000005
    write_inv_sensor_16(0x3F98, 0x0000);   // mbist_wr_data_oi                         0x00000000
    write_inv_sensor_16(0x3F98, 0x0000);   // mbist_wr_data_oi                         0x00000000
    write_inv_sensor_16(0x3FA0, 0x0000);       // mbist_address_oi                         0x0000
    write_inv_sensor_16(0x3FA2, 0x03FE);       // mbist_depth_oi                           0x03FE     (1022)
    write_inv_sensor_16(0x3FA4, 0xC060);       // mbist_ctrl_oi                            0xC060
    write_inv_sensor(0x3FA8, 0x07);         // mbist_ctrl                               0x07
    // Turn on the pll and enable its clock as the system clock
    write_inv_sensor(0x3110, 0x00);         // pll_ctrl                                 0x0F
    msleep(10);               // 50 ms delay to allow pll to settle after powerup and clock enabling
    // Start streaming
    write_inv_sensor(0x3000, 0x01);        // mode_select                                0x00
    // restart CFPN
    write_inv_sensor(0x3A01, 0x01);         // restart BLC
    write_inv_sensor(0x3A82, 0x01);		   // restart CFPN
    //msleep(250);
} //endof video

static void hs_video_setting()  // VideoHDSetting_120fps
{
    LOG_INF("E\n  Video  120fps ");

    
}

static void slim_video_setting()  // VideoHDSetting
{
    LOG_INF("E\n  Video  120fps ");

    
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);
    
    if (enable) {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_inv_sensor(0x30D8, 0x10);
        write_inv_sensor(0x0600, 0x00);
        write_inv_sensor(0x0601, 0x02);
    } else {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_inv_sensor(0x30D8, 0x00);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *   get_imgsensor_id
 *
 * DESCRIPTION
 *   This function get the sensor ID
 *
 * PARAMETERS
 *   *sensorID : return the sensor ID
 *
 * RETURNS
 *   None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *   open
 *
 * DESCRIPTION
 *   This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *   None
 *
 * RETURNS
 *   None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
    //const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
    LOG_1;
    LOG_2;
    
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;
    
    /* initail sequence write in  */
    sensor_init();
    
    spin_lock(&imgsensor_drv_lock);
    
    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = KAL_FALSE;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);
    
    return ERROR_NONE;
}   /*  open  */



/*************************************************************************
 * FUNCTION
 *   close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *   None
 *
 * RETURNS
 *   None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E reset otp flag\n");
    //otp_clear_flag();//clear otp flag
    
    /*No Need to implement this function*/
    
    return ERROR_NONE;
}   /*  close  */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *   This function start the sensor preview.
 *
 * PARAMETERS
 *   *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *   None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    kal_uint32 sensor_id = 0;
    
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    sensor_id = return_sensor_id();
    LOG_INF("L, sensor_id[%d]\n",sensor_id);
    return ERROR_NONE;
}   /*  preview   */

/*************************************************************************
 * FUNCTION
 *   capture
 *
 * DESCRIPTION
 *   This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *   None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps < 50) {
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }else if (imgsensor.current_fps == 150) {  //317Mhz PIP capture: 15fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap2.pclk;
        imgsensor.line_length = imgsensor_info.cap2.linelength;
        imgsensor.frame_length = imgsensor_info.cap2.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap2.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }else  {  //400Mhz PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("Caputre fps:%d\n",imgsensor.current_fps);
    if(imgsensor.current_fps == 150)
        capture_setting_15fps(imgsensor.current_fps);
    else
        capture_setting(imgsensor.current_fps);
    
    LOG_INF("L\n");
    return ERROR_NONE;
}   /* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                               MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
#ifdef IHDR_USED
    if(imgsensor.ihdr_en)
        INV1310MIPI_set_Video_IHDR(1);
    else
        INV1310MIPI_set_Video_IHDR(0);
#else
    normal_video_setting(imgsensor.current_fps);
#endif
    
    LOG_INF("L\n");
    return ERROR_NONE;
}   /*  normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                           MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();
    LOG_INF("L\n");
    return ERROR_NONE;
}   /*  hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                             MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
    LOG_INF("L\n");
    return ERROR_NONE;
}   /*  slim_video   */

/*************************************************************************
 * FUNCTION
 * Custom1
 *
 * DESCRIPTION
 *   This function start the sensor Custom1.
 *
 * PARAMETERS
 *   *image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *   None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
    imgsensor.pclk = imgsensor_info.custom1.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom1.linelength;
    imgsensor.frame_length = imgsensor_info.custom1.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
    imgsensor.pclk = imgsensor_info.custom3.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom3.linelength;
    imgsensor.frame_length = imgsensor_info.custom3.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom3   */

static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
    imgsensor.pclk = imgsensor_info.custom4.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom4.linelength;
    imgsensor.frame_length = imgsensor_info.custom4.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom4   */


static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
    
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
    imgsensor.pclk = imgsensor_info.custom5.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom5.linelength;
    imgsensor.frame_length = imgsensor_info.custom5.framelength;
    imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}   /*  Custom5   */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    
    LOG_INF("imgsensor_info.cap.grabwindow_width: %d\n", imgsensor_info.cap.grabwindow_width);
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
    
    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;
    
    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;
    
    
    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight    = imgsensor_info.hs_video.grabwindow_height;
    
    sensor_resolution->SensorSlimVideoWidth  = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;
    
    sensor_resolution->SensorCustom1Width  = imgsensor_info.custom1.grabwindow_width;
    sensor_resolution->SensorCustom1Height     = imgsensor_info.custom1.grabwindow_height;
    
    sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
    sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;
    
    sensor_resolution->SensorCustom3Width  = imgsensor_info.custom3.grabwindow_width;
    sensor_resolution->SensorCustom3Height     = imgsensor_info.custom3.grabwindow_height;
    
    sensor_resolution->SensorCustom4Width  = imgsensor_info.custom4.grabwindow_width;
    sensor_resolution->SensorCustom4Height     = imgsensor_info.custom4.grabwindow_height;
    
    sensor_resolution->SensorCustom5Width  = imgsensor_info.custom5.grabwindow_width;
    sensor_resolution->SensorCustom5Height     = imgsensor_info.custom5.grabwindow_height;
    
    LOG_INF("L\n");
    return ERROR_NONE;
}   /*  get_resolution  */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
                           MSDK_SENSOR_INFO_STRUCT *sensor_info,
                           MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
    
    
    //sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
    //sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */
    
    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 1; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */
    
    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;
    
    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
    sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
    sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
    sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
    sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
    sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;
    
    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
    
    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
    
    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 5; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */
    
    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;   // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;
    
    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
            
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
            
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
            
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            
            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
            
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
            
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
            
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
            
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
            
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
            
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
            
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
            
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
            
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
            
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
            
            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
            
            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }
    
    return ERROR_NONE;
}   /*  get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            preview(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            capture(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            normal_video(image_window, sensor_config_data);  // VideoFullSizeSetting
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            hs_video(image_window, sensor_config_data);  // VideoHDSetting_120fps
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            slim_video(image_window, sensor_config_data); // VideoHDSetting
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            Custom1(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            Custom2(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            Custom3(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            Custom4(image_window, sensor_config_data); // Custom1
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            Custom5(image_window, sensor_config_data); // Custom1
            break;
        default:
            LOG_INF("Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}   /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
    LOG_INF("framerate = %d\n ", framerate);
    // SetVideoMode Function should fix framerate
    if (framerate == 0)
        // Dynamic frame rate
        return ERROR_NONE;
    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = framerate;
    spin_unlock(&imgsensor_drv_lock);
    set_max_framerate(imgsensor.current_fps,1);
    set_dummy();
    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
    //dingej1 2015.05.09 begin disable auto flicker at the beginning of debug.
    /*
     spin_lock(&imgsensor_drv_lock);
     if (enable) //enable auto flicker
     imgsensor.autoflicker_en = KAL_TRUE;
     else //Cancel Auto flick
     imgsensor.autoflicker_en = KAL_FALSE;
     spin_unlock(&imgsensor_drv_lock);
     */
    //dingej1 end.
    return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;
    
    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);
    
    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            if (imgsensor.current_fps == 300){
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
                imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
                imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
                imgsensor.min_frame_length = imgsensor.frame_length;
                spin_unlock(&imgsensor_drv_lock);
            }
            else if (imgsensor.current_fps == 150) {  //317Mhz, PIP capture: 15fps for less than 13M, 20fps for 16M,15fps for 20M
                frame_length = imgsensor_info.cap2.pclk / framerate * 10 / imgsensor_info.cap2.linelength;
                spin_lock(&imgsensor_drv_lock);
                imgsensor.dummy_line = (frame_length > imgsensor_info.cap2.framelength) ? (frame_length - imgsensor_info.cap2.framelength) : 0;
                imgsensor.frame_length = imgsensor_info.cap2.framelength + imgsensor.dummy_line;
                spin_unlock(&imgsensor_drv_lock);
            }
            else{
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
                imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
                imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
                spin_unlock(&imgsensor_drv_lock);
            }
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
        case MSDK_SCENARIO_ID_CUSTOM1:
            frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            frame_length = imgsensor_info.custom5.pclk / framerate * 10 / imgsensor_info.custom5.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength) ? (frame_length - imgsensor_info.custom5.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
    
    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *framerate = imgsensor_info.hs_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *framerate = imgsensor_info.slim_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM1:
            *framerate = imgsensor_info.custom1.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *framerate = imgsensor_info.custom2.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM3:
            *framerate = imgsensor_info.custom3.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM4:
            *framerate = imgsensor_info.custom4.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CUSTOM5:
            *framerate = imgsensor_info.custom5.max_framerate;
            break;
        default:
            break;
    }
    
    return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                                  UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    unsigned long long *feature_return_para=(unsigned long long *) feature_para;
    
    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
    
    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_inv_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_inv_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_SENSOR_OTP_AWB_CMD:
            LOG_INF("Update sensor awb from otp :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.update_sensor_otp_awb = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            if(0 != imgsensor.update_sensor_otp_awb || 0 != imgsensor.update_sensor_otp_lsc) {
                //inv_opt_update(imgsensor.update_sensor_otp_awb, imgsensor.update_sensor_otp_lsc);
            }
            break;
        case SENSOR_FEATURE_SET_SENSOR_OTP_LSC_CMD:
            LOG_INF("Update sensor lsc from otp :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.update_sensor_otp_lsc = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            if(0 != imgsensor.update_sensor_otp_awb || 0 != imgsensor.update_sensor_otp_lsc) {
                //inv_opt_update(imgsensor.update_sensor_otp_awb, imgsensor.update_sensor_otp_lsc);
            }
            break;
        case SENSOR_FEATURE_SET_HDR:
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        default:
            break;
    }
    
    return ERROR_NONE;
}   /*  feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 INV1310_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}   /*  INV1310_MIPI_RAW_SensorInit  */
