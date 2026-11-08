/**************************************************************
 * 锟侥硷拷锟斤拷:  initialize.c
 * 锟斤拷锟斤拷时锟斤拷: 2026锟斤拷3锟斤拷6锟斤拷 锟斤拷锟斤拷6:13:05
 * 锟斤拷锟斤拷:
 **************************************************************/
#include "zf_common_headfile.h"
volatile int start_flag = 0;

void init_mian(void){
    ips114_init();

    while(1)
    {
        if(mt9v03x_init())
            ips114_show_string(0, 80, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(500);                                                   // 锟斤拷锟斤拷时锟斤拷锟斤拷锟斤拷锟狡憋拷示锟届常
    }
    ips114_show_string(0, 16, "init success.");
    Motor_Init();
    Encoder_Init();
    voltage_detect_init();
    LED_init();
    imu660rc_init_main();

    menu_init();
    menu_display();
    start_flag = 1;
    negative_pressure_start();
    //system_delay_ms(1000);
    // ================= 锟斤拷始锟斤拷1000ms 锟斤拷锟斤拷锟斤拷烁 LED 锟斤拷效 =================
        int total_time = 0;          // 锟斤拷录锟斤拷锟斤拷锟侥碉拷锟斤拷时锟斤拷
        int current_delay = 150;     // 锟斤拷始锟斤拷烁锟斤拷锟轿� 150ms

        while(total_time < 2000)
        {
            // 状态 1锟斤拷LED1锟斤拷锟斤拷LED2锟斤拷
            LED1_on();
            LED2_on();
            system_delay_ms(current_delay);
            total_time += current_delay;

            if (total_time >= 2000) break; // 锟斤拷锟斤拷锟绞憋拷锟斤拷汛锟疥，锟斤拷前锟斤拷锟斤拷

            // 状态 2锟斤拷LED1锟斤拷LED2锟斤拷
            LED1_off();
            LED2_off();
            system_delay_ms(current_delay);
            total_time += current_delay;

            // 每锟轿斤拷锟斤拷锟疥，锟斤拷锟斤拷锟斤拷时时锟戒（每锟轿硷拷小锟斤拷原锟斤拷锟斤拷 75%锟斤拷锟斤拷实锟斤拷越锟斤拷越锟斤拷
            current_delay = (current_delay * 3) / 4;

            // 锟斤拷锟斤拷锟斤拷小锟斤拷时为 20ms锟斤拷锟斤拷止锟斤拷锟届到锟斤拷锟桔匡拷锟斤拷锟斤拷锟斤拷烁
            if(current_delay < 10) {
                current_delay = 10;
            }
        }

        // 锟斤拷锟斤拷前锟斤拷锟斤拷 LED1 锟斤拷 LED2 锟斤拷锟截闭ｏ拷锟斤拷锟街凤拷锟斤拷状态锟斤拷锟洁，锟斤拷锟斤拷锟揭拷锟斤拷锟斤拷筛锟轿� _on() 锟斤拷
    LED1_off();
    LED2_off();
    INS_Init();
    Motor_Reset_State();
    pit_ms_init(CCU60_CH0,2);
}
