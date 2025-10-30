#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "imu_orientation.h"

#define I2C_PORT i2c0
#define I2C_SDA_PIN 16
#define I2C_SCL_PIN 17

static const uint8_t ACC_ADDRS[] = {0x18, 0x19};
static const uint8_t MAG_ADDRS[] = {0x1E, 0x1C};

#define A_WHO_AM_I 0x0F
#define A_CTRL1    0x20
#define A_CTRL4    0x23
#define A_OUT_X_L  0x28
#define A_AUTOINC  0x80
#define A_CTRL1_100HZ_XYZ 0x57
#define A_CTRL4_HR2G_BDU  0x88

#define M_CRA      0x00
#define M_CRB      0x01
#define M_MR       0x02
#define M_OUT_X_H  0x03
#define M_ID_A     0x0A
#define M_ID_B     0x0B
#define M_ID_C     0x0C

#define SAMPLE_HZ 100
#define PRINT_INTERVAL_MS 500

#define MAP_AX 0
#define MAP_AY 1
#define MAP_AZ 2
#define SIGN_AX (+1)
#define SIGN_AY (+1)
#define SIGN_AZ (+1)

#define MAP_MX 0
#define MAP_MY 2
#define MAP_MZ 1
#define SIGN_MX (+1)
#define SIGN_MY (+1)
#define SIGN_MZ (+1)

static uint8_t g_acc=0, g_mag=0;

static inline void i2c_wr(uint8_t addr, uint8_t reg, uint8_t val){uint8_t b[2]={reg,val};i2c_write_blocking(I2C_PORT,addr,b,2,false);}
static inline void i2c_rd(uint8_t addr, uint8_t reg, uint8_t*dst,size_t n){i2c_write_blocking(I2C_PORT,addr,&reg,1,true);i2c_read_blocking(I2C_PORT,addr,dst,n,false);}

static bool acc_init(void){
    for(size_t i=0;i<sizeof(ACC_ADDRS);++i){
        uint8_t a=ACC_ADDRS[i], who=0;
        i2c_rd(a,A_WHO_AM_I,&who,1);
        if(who==0x33){g_acc=a;i2c_wr(g_acc,A_CTRL1,A_CTRL1_100HZ_XYZ);i2c_wr(g_acc,A_CTRL4,A_CTRL4_HR2G_BDU);return true;}
    }
    return false;
}

static bool mag_init(void){
    for(size_t i=0;i<sizeof(MAG_ADDRS);++i){
        uint8_t a=MAG_ADDRS[i], ida=0,idb=0,idc=0;
        i2c_rd(a,M_ID_A,&ida,1); i2c_rd(a,M_ID_B,&idb,1); i2c_rd(a,M_ID_C,&idc,1);
        if(ida=='H' && idb=='4' && idc=='3'){g_mag=a; i2c_wr(g_mag,M_CRA,0x14); i2c_wr(g_mag,M_CRB,0x20); i2c_wr(g_mag,M_MR,0x00); return true;}
    }
    return false;
}

static void acc_read_mg(float*ax,float*ay,float*az){
    uint8_t raw[6]; i2c_rd(g_acc,A_OUT_X_L|A_AUTOINC,raw,6);
    int16_t rx=(int16_t)(raw[1]<<8|raw[0]);
    int16_t ry=(int16_t)(raw[3]<<8|raw[2]);
    int16_t rz=(int16_t)(raw[5]<<8|raw[4]);
    rx>>=4; ry>>=4; rz>>=4;
    float v[3]={(float)rx,(float)ry,(float)rz};
    *ax=v[MAP_AX]*SIGN_AX; *ay=v[MAP_AY]*SIGN_AY; *az=v[MAP_AZ]*SIGN_AZ;
}

static void mag_read_raw(int16_t*mx,int16_t*my,int16_t*mz){
    uint8_t r[6]; i2c_rd(g_mag,M_OUT_X_H,r,6);
    int16_t x=(int16_t)(r[0]<<8|r[1]);
    int16_t z=(int16_t)(r[2]<<8|r[3]);
    int16_t y=(int16_t)(r[4]<<8|r[5]);
    int16_t vv[3]={x,y,z};
    *mx=vv[MAP_MX]*SIGN_MX; *my=vv[MAP_MY]*SIGN_MY; *mz=vv[MAP_MZ]*SIGN_MZ;
}

void run_imu_orientation(void){
    i2c_init(I2C_PORT,400*1000);
    gpio_set_function(I2C_SDA_PIN,GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN,GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN); gpio_pull_up(I2C_SCL_PIN);

    if(!acc_init()){printf("[ACC] not found\n"); while(1)tight_loop_contents();}
    if(!mag_init()){printf("[MAG] not found\n"); while(1)tight_loop_contents();}

    absolute_time_t next_sample=make_timeout_time_ms(0);
    static absolute_time_t next_print;
    const int dt=1000/SAMPLE_HZ;

    while(true){
        sleep_until(next_sample); next_sample=delayed_by_ms(next_sample,dt);

        float ax,ay,az; acc_read_mg(&ax,&ay,&az);
        int16_t mx,my,mz; mag_read_raw(&mx,&my,&mz);

        uint32_t now=to_ms_since_boot(get_absolute_time());
        uint32_t gate=to_ms_since_boot(next_print);
        if(now>=gate){
            next_print=delayed_by_ms(get_absolute_time(),PRINT_INTERVAL_MS);
            printf("ax=%+7.1f ay=%+7.1f az=%+7.1f mg | mx=%+6d my=%+6d mz=%+6d\n", ax,ay,az, mx,my,mz);
        }
    }
}
