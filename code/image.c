/**************************************************************
 * �ļ���:  image.c
 * ����ʱ��: 2026��1��30�� ����8:26:19
 * ����: ����ͼ����
 **************************************************************/
#include "zf_device_mt9v03x.h"
#include "stdio.h"
#include "stdint.h"
#include <math.h>
#pragma section all "cpu1_dsram"  // ��ʼ�����±���ȫ�����䵽 Core 1 �ڴ�
uint8      mt9v03x_image_bin[MT9V03X_H][MT9V03X_W];
#pragma section all restore       // �������ָ�Ĭ�Ϸ���

uint8 safe_access_binimg(int x, int y) {
    if (x >= 0 && x < MT9V03X_W && y >= 0 && y < MT9V03X_H) {
        return mt9v03x_image_bin[y][x];
    } else {
        return 0;
    }
}

uint8 safe_access_img(int x, int y) {
    if (x >= 0 && x < MT9V03X_W - 1 && y >= 0 && y < MT9V03X_H) {
        return mt9v03x_image[y][x];
    } else {
        return 0;
    }
}

void binarization(int thres)
{
    uint8 *src = (uint8*)mt9v03x_image;       // ֱ��ʹ��ȫ���������ʼָ��
    uint8 *dst = (uint8*)mt9v03x_image_bin;
    int total = MT9V03X_H * MT9V03X_W;

    for (int i = 0; i < total; i++) {
        *dst++ = (*src++ > thres) ? 255 : 0;
    }
}


/**
 * @brief ʹ�ð�ȫ���ʺ������Ż���Otsu��ֵ����
 * @param step ����������Ĭ��2�����������������������ٶȣ����ή�;���
 * @return uint8_t ����õ��������ֵ
 */
uint8 otsuThreshold_fast(void)
{
    #define GrayScale 256

    int width = MT9V03X_W;
    int height = MT9V03X_H;
    int bottom_start = height * 2 / 3;
    int start_x = 20;
    int end_x = width - 20;

    uint16 pixelCount[GrayScale] = {0};
    uint16 bottomPixelCount[GrayScale] = {0};
    uint32 pixelSum = 0;
    uint32 gray_sum = 0;
    uint32 bottomGraySum = 0;
    uint32 bottomGraySumSq = 0;
    uint32 bottomPixelSum = 0;
    uint8 Pixel_Max = 0;
    uint8 Pixel_Min = 255;

    // ֱ������������ safe_access_img��ѭ���߽��ѱ�֤��ȫ
    for (int i = 0; i < height; i += 2)
    {
        uint8 *row = mt9v03x_image[i];
        int is_bottom = (i >= bottom_start);
        for (int j = start_x; j < end_x; j += 2)
        {
            uint8 val = row[j];
            pixelCount[val]++;
            gray_sum += val;
            pixelSum++;

            if (is_bottom)
            {
                bottomPixelCount[val]++;
                bottomPixelSum++;
                bottomGraySum += val;
                bottomGraySumSq += (uint32)val * val;
            }

            if (val > Pixel_Max) Pixel_Max = val;
            if (val < Pixel_Min) Pixel_Min = val;
        }
    }

    if (pixelSum == 0) return 100;

    // ---------- ǰ׺�ͣ�uint32 �㹻�����Ҷ��ܺ� ~1.13M��----------
    uint16 prefixCount[GrayScale];
    uint32 prefixGraySum[GrayScale];
    uint32 cumCount = 0;
    uint32 cumGray = 0;
    for (int g = Pixel_Min; g <= Pixel_Max; g++)
    {
        cumCount += pixelCount[g];
        cumGray += (uint32)g * pixelCount[g];
        prefixCount[g] = (uint16)cumCount;
        prefixGraySum[g] = cumGray;
    }
    uint32 totalCount = prefixCount[Pixel_Max];
    uint32 totalGray = prefixGraySum[Pixel_Max];
    float invN = 1.0f / totalCount;

    // ---------- Otsu��float��TC1.6 Ӳ�� FPU��----------
    uint8 bestTh = 0;
    float maxVar = 0.0f;

    for (int th = Pixel_Min; th < Pixel_Max; th++)
    {
        uint32 cnt0 = prefixCount[th];
        if (cnt0 == 0 || cnt0 == totalCount) continue;

        uint32 cnt1 = totalCount - cnt0;
        uint32 sum0 = prefixGraySum[th];
        uint32 sum1 = totalGray - sum0;
        float diff = (float)sum0 / cnt0 - (float)sum1 / cnt1;
        float var = (float)cnt0 * invN * (float)cnt1 * invN * diff * diff;

        if (var > maxVar)
        {
            maxVar = var;
            bestTh = th;
        }
    }

    // ---------- Otsu ���У�� & �ײ�����ͳ�� ----------
    uint32 cnt0 = prefixCount[bestTh];
    uint32 sum0 = prefixGraySum[bestTh];
    float mean0 = (float)sum0 / cnt0;
    float mean1 = (float)(totalGray - sum0) / (totalCount - cnt0);
    float class_gap = mean1 - mean0;

    uint32 white_cnt = totalCount - cnt0;
    float white_ratio = (float)white_cnt * invN;

    float bg_std = 0.0f;
    uint8 bg_ref = 100;
    if (bottomPixelSum > 0)
    {
        uint32 cum = 0;
        for (int g = 0; g < GrayScale; g++)
        {
            cum += bottomPixelCount[g];
            if (cum >= bottomPixelSum * 0.95f)
            {
                bg_ref = g;
                break;
            }
        }
        float bg_mean = (float)bottomGraySum / bottomPixelSum;
        float bg_var = (float)bottomGraySumSq / bottomPixelSum - bg_mean * bg_mean;
        if (bg_var > 0) bg_std = sqrtf(bg_var);
    }

    uint8 final_th = bestTh;

    // Otsu �������������׵�̫�� OR ����ֵ��̫С���ֵ��ǡ����� vs ���⡱��
    int otsu_unreliable = (white_ratio < 0.04f) || (class_gap < 30.0f);

    if (otsu_unreliable && bottomPixelSum > 0)
    {
        // ����Ӧƫ�ƣ�����Խǿ��bg_std Խ�󣩣�ƫ��Խ��
        int offset = 20 + (int)(bg_std * 0.5f);
        if (offset > 40) offset = 40;
        int new_th = bg_ref + offset;
        final_th = (new_th > 70) ? new_th : 70;
    }

    // �޷�
    if (final_th < 70) final_th = 70;
    if (final_th > 140) final_th = 140;

    // ֡��ƽ��
    static uint8 prev_th = 100;
    final_th = (prev_th * 3 + final_th) / 4;
    prev_th = final_th;

    return final_th;
}
