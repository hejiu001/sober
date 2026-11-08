/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

/*
 *  350 13 0.1
 *  300 11 0.1
 */

// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等

    static uint16 time_used=0;
    wireless_uart_init();                                                    // 判断是否通过初始化


    init_mian();
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    system_start();
    while (TRUE)
    {
        time_used = system_getval()/100000000.0f;//秒
                // 此处编写需要循环执行的代码
//        sent_two_int(0,INS_Get_Target_Speed());
//        send_speed_to_pc();
//        ips114_show_int(0,0,return_error(),2);
//        float yaw = eulerAngle.yaw;
//        ips114_show_float(0,16,yaw,3,3);
        if(time_used>5)
        {
            car_stop();

             // ================= 跑完停车后的交互小菜单 =================
             if (run_mode == 0) {
                 // 1. 立刻计算速度表
                 INS_Generate_Velocity_Table(600, 1200);

                 int save_choice = 1; // 1: 保存, 0: 取消
                 ips114_clear();
                 ips114_show_string(10, 16*1, "Done!");

                 // 交互循环
                 while(1) {
                     // 显示选项
                     ips114_show_string(30, 16*3, " Save");
                     ips114_show_string(30, 16*4, " Discard Map");

                     // 显示光标
                     if(save_choice == 1) ips114_show_string(10, 16*3, "->");
                     else                 ips114_show_string(10, 16*4, "->");

                     // 扫描按键 (由于此时菜单系统set_f=1，我们需要手动调用gpio判断)
                     if(!gpio_get_level(KEY_UP) || !gpio_get_level(KEY_DOWN)) {
                         save_choice = !save_choice; // 切换选择
                         ips114_show_string(10, 16*3, "  "); // 清除旧光标
                         ips114_show_string(10, 16*4, "  ");
                         system_delay_ms(200); // 简单消抖加延时
                     }

                     if(!gpio_get_level(KEY_PRESS)) { // 按下确定
                         system_delay_ms(20); while(!gpio_get_level(KEY_PRESS)); // 等待释放

                         if(save_choice == 1) {
                             ips114_show_string(20, 16*6, "SAVING...");
                             Save_Trajectory_To_Flash();
                             ips114_show_string(20, 16*6, "OK! ");
                         } else {
                             ips114_show_string(20, 16*6, "ABORTED");
                         }
                         system_delay_ms(800);
                         break; // 退出交互循环
                     }
                 }
             }

             // ================= 重置并返回主菜单界面 =================
             start_flag = 0;
             set_f = 0;       // 回到菜单模式
             time_used = 0;   // 计时归零

             ips114_clear();
          //  time_used=0;
        }
        // 此处编写需要循环执行的代码
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
