/**************************************************************
 * 文件名:  battery_detect.c
 * 创建时间: 2026年3月7日 下午4:02:39
 * 描述:
 **************************************************************/

#define VOLTAGE_PORT    ADC0_CH11_A11 // 定义检测端口

#include "LED.h"
#include "battery_detect.h"

float valtage_now = 0;

void voltage_detect_init(){
    adc_init(VOLTAGE_PORT, ADC_12BIT);
    pit_ms_init(CCU61_CH0,500);
}

void detect_main(){
    int valtage_adc = 0;


    valtage_adc = adc_convert(VOLTAGE_PORT);
    valtage_now = 36.3 * valtage_adc / 4096-0.3;

}
