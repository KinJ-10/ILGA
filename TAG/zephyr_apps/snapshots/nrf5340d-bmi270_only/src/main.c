#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>

/* sensor_value を「ミリ単位の整数」に変換
*
* Zephyr の sensor_value は:
* val1: 整数部
* val2: 小数部（1e-6 単位）
*
* として扱われることが多いので、
* 実数 ≒ val1 + val2 / 1e6
* ミリ単位にすると:
* milli ≒ (val1 * 1000) + (val2 / 1000)
*/

//SensorValue o milli
static inline int32_t sv_to_milli(const struct sensor_value *v)
{
    return (int32_t)(v->val1 * 1000 + v->val2 / 1000);
}

int main(void)
{
    const struct device *bmi = DEVICE_DT_GET_ONE(bosch_bmi270);

    struct sensor_value acc[3];
    struct sensor_value gyr[3];

    struct sensor_value full_scale;
    struct sensor_value sampling_freq;
    struct sensor_value oversampling;

    uint32_t seq = 0;
    int ret;

    /* デバイスの存在確認 */
    printk("BMI270 dev ptr = %p, name = %s\n", bmi, bmi->name);

    if (!device_is_ready(bmi)) {
    printk("BMI270 device not ready: %s\n", bmi->name);
    return 0;
    }

    printk("BMI270 is ready\n");

    /* ----------------------------------------------------- */
    /* 加速度センサの設定（±2G, 100Hz, Normal mode） */
    /* ----------------------------------------------------- */
    full_scale.val1 = 2; /* ±2G */
    full_scale.val2 = 0;
    oversampling.val1 = 1; /* Normal mode 相当 */
    oversampling.val2 = 0;
    sampling_freq.val1 = 100; /* 100Hz */
    sampling_freq.val2 = 0;

    ret = sensor_attr_set(bmi, SENSOR_CHAN_ACCEL_XYZ,
    SENSOR_ATTR_FULL_SCALE, &full_scale);
    printk("attr accel FULL_SCALE ret=%d\n", ret);

    ret = sensor_attr_set(bmi, SENSOR_CHAN_ACCEL_XYZ,
    SENSOR_ATTR_OVERSAMPLING, &oversampling);
    printk("attr accel OVERSAMPLING ret=%d\n", ret);

    ret = sensor_attr_set(bmi, SENSOR_CHAN_ACCEL_XYZ,
    SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);
    printk("attr accel SAMPLING_FREQ ret=%d\n", ret);

    /* ----------------------------------------------------- */
    /* ジャイロセンサの設定（±500 dps, 100Hz, Normal mode） */
    /* ----------------------------------------------------- */
    full_scale.val1 = 500; /* ±500 dps */
    full_scale.val2 = 0;
    oversampling.val1 = 1;
    oversampling.val2 = 0;
    sampling_freq.val1 = 100;
    sampling_freq.val2 = 0;

    ret = sensor_attr_set(bmi, SENSOR_CHAN_GYRO_XYZ,
    SENSOR_ATTR_FULL_SCALE, &full_scale);
    printk("attr gyro FULL_SCALE ret=%d\n", ret);

    ret = sensor_attr_set(bmi, SENSOR_CHAN_GYRO_XYZ,
    SENSOR_ATTR_OVERSAMPLING, &oversampling);
    printk("attr gyro OVERSAMPLING ret=%d\n", ret);

    ret = sensor_attr_set(bmi, SENSOR_CHAN_GYRO_XYZ,
    SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);
    printk("attr gyro SAMPLING_FREQ ret=%d\n", ret);

    /* ----------------------------------------------------- */
    /* メインループ：データ取得 → 整数スケーリング → UART出力 */
    /* ----------------------------------------------------- */
    while (1) {
        int err = sensor_sample_fetch(bmi);
        if (err) {
            printk("sensor_sample_fetch err=%d\n", err);
            k_msleep(10);
            continue;
        }

        /* センサ値を取得 */
        sensor_channel_get(bmi, SENSOR_CHAN_ACCEL_XYZ, acc);
        sensor_channel_get(bmi, SENSOR_CHAN_GYRO_XYZ, gyr);

        /* mg / mdps に変換 */
        int32_t ax_mg = sv_to_milli(&acc[0]);
        int32_t ay_mg = sv_to_milli(&acc[1]);
        int32_t az_mg = sv_to_milli(&acc[2]);

        int32_t gx_mdps = sv_to_milli(&gyr[0]);
        int32_t gy_mdps = sv_to_milli(&gyr[1]);
        int32_t gz_mdps = sv_to_milli(&gyr[2]);

        /* CSV形式で1行出力:
        * seq,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps
        */
        printk("%u,%d,%d,%d,%d,%d,%d\n",
        seq++,
        ax_mg, ay_mg, az_mg,
        gx_mdps, gy_mdps, gz_mdps);

        /* 表示間引き用（サンプリング設定は100HzのままでもOK） */
        k_msleep(10); /* 100Hz相当なら 10ms */
    }

    return 0;
}
