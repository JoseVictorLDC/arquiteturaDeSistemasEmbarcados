#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

#define ADC_NODE DT_NODELABEL(adc0)
#define UART_NODE DT_CHOSEN(zephyr_console)

#if !DT_NODE_HAS_STATUS(ADC_NODE, okay)
#error "ADC0 nao esta habilitado no device tree."
#endif

#if !DT_NODE_HAS_STATUS(UART_NODE, okay)
#error "UART console nao esta habilitada no device tree."
#endif

#define ADC_CHANNEL_ID 0
#define ADC_RESOLUTION 12
#define ADC_REF_MV 3300

#define DEFAULT_ACQ_RATE_HZ 100
#define DEFAULT_CUTOFF_HZ 5

#define MIN_RATE_HZ 1
#define MAX_RATE_HZ 2000

#define MAX_FIR_TAPS 128

#define ACQ_STACK_SIZE 2048
#define CMD_STACK_SIZE 2048

#define ACQ_PRIORITY 5
#define CMD_PRIORITY 6

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

static int16_t adc_sample_buffer;

static struct adc_channel_cfg adc_cfg = {
    .gain = ADC_GAIN_1,
    .reference = ADC_REF_VDD_1,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = ADC_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
    .input_positive = ADC_CHANNEL_ID,
#endif
};

static struct adc_sequence adc_seq = {
    .channels = BIT(ADC_CHANNEL_ID),
    .buffer = &adc_sample_buffer,
    .buffer_size = sizeof(adc_sample_buffer),
    .resolution = ADC_RESOLUTION,
};

static struct k_mutex cfg_mutex;
static struct k_mutex print_mutex;

static uint32_t acq_rate_hz = DEFAULT_ACQ_RATE_HZ;
static uint32_t acq_period_us = 1000000U / DEFAULT_ACQ_RATE_HZ;

static bool filter_enabled = false;
static uint32_t cutoff_hz = DEFAULT_CUTOFF_HZ;
static uint16_t fir_taps = 0;

static int32_t fir_buffer[MAX_FIR_TAPS];
static int64_t fir_sum = 0;
static uint16_t fir_index = 0;
static uint16_t fir_count = 0;

static atomic_t produced_samples;
static atomic_t transmitted_samples;
static atomic_t adc_errors;

static uint64_t now_us(void)
{
    return k_uptime_get() * 1000ULL;
}

static uint16_t compute_fir_taps(uint32_t rate_hz, uint32_t cutoff)
{
    if (cutoff == 0) {
        return 1;
    }

    uint32_t taps = rate_hz / cutoff;

    if (taps < 1) {
        taps = 1;
    }

    if (taps > MAX_FIR_TAPS) {
        taps = MAX_FIR_TAPS;
    }

    return (uint16_t)taps;
}

static void clear_filter_state_locked(void)
{
    memset(fir_buffer, 0, sizeof(fir_buffer));
    fir_sum = 0;
    fir_index = 0;
    fir_count = 0;
}

static void print_cfg(void)
{
    k_mutex_lock(&cfg_mutex, K_FOREVER);

    uint32_t rate = acq_rate_hz;
    uint32_t period = acq_period_us;
    bool enabled = filter_enabled;
    uint32_t cutoff = cutoff_hz;
    uint16_t taps = fir_taps;

    k_mutex_unlock(&cfg_mutex);

    k_mutex_lock(&print_mutex, K_FOREVER);

    printk(
        "CFG,%u,%u,%s,%u,%u\n",
        rate,
        period,
        enabled ? "LOWPASS_FIR" : "NONE",
        cutoff,
        enabled ? taps : 0
    );

    k_mutex_unlock(&print_mutex);
}

static int set_rate(uint32_t rate)
{
    if (rate < MIN_RATE_HZ || rate > MAX_RATE_HZ) {
        return -EINVAL;
    }

    k_mutex_lock(&cfg_mutex, K_FOREVER);

    acq_rate_hz = rate;
    acq_period_us = 1000000U / rate;

    if (filter_enabled) {
        fir_taps = compute_fir_taps(acq_rate_hz, cutoff_hz);
        clear_filter_state_locked();
    }

    k_mutex_unlock(&cfg_mutex);

    LOG_INF("Frequencia alterada para %u Hz", rate);

    return 0;
}

static int set_lowpass(uint32_t cutoff)
{
    if (cutoff == 0) {
        return -EINVAL;
    }

    k_mutex_lock(&cfg_mutex, K_FOREVER);

    filter_enabled = true;
    cutoff_hz = cutoff;
    fir_taps = compute_fir_taps(acq_rate_hz, cutoff_hz);

    clear_filter_state_locked();

    k_mutex_unlock(&cfg_mutex);

    LOG_INF("Filtro passa-baixa ativado com corte em %u Hz", cutoff);

    return 0;
}

static void set_filter_none(void)
{
    k_mutex_lock(&cfg_mutex, K_FOREVER);

    filter_enabled = false;
    fir_taps = 0;
    clear_filter_state_locked();

    k_mutex_unlock(&cfg_mutex);

    LOG_INF("Filtro desativado");
}

static void reset_system(void)
{
    atomic_clear(&produced_samples);
    atomic_clear(&transmitted_samples);
    atomic_clear(&adc_errors);

    k_mutex_lock(&cfg_mutex, K_FOREVER);

    acq_rate_hz = DEFAULT_ACQ_RATE_HZ;
    acq_period_us = 1000000U / DEFAULT_ACQ_RATE_HZ;

    filter_enabled = false;
    cutoff_hz = DEFAULT_CUTOFF_HZ;
    fir_taps = 0;

    clear_filter_state_locked();

    k_mutex_unlock(&cfg_mutex);

    LOG_INF("Sistema resetado");
}

static int32_t apply_filter(int32_t mv)
{
    int32_t output;

    k_mutex_lock(&cfg_mutex, K_FOREVER);

    if (!filter_enabled || fir_taps <= 1) {
        output = mv;
        k_mutex_unlock(&cfg_mutex);
        return output;
    }

    fir_sum -= fir_buffer[fir_index];
    fir_buffer[fir_index] = mv;
    fir_sum += mv;

    fir_index++;

    if (fir_index >= fir_taps) {
        fir_index = 0;
    }

    if (fir_count < fir_taps) {
        fir_count++;
    }

    output = (int32_t)(fir_sum / fir_count);

    k_mutex_unlock(&cfg_mutex);

    return output;
}

static int32_t adc_raw_to_mv_safe(int16_t raw)
{
    int32_t mv = raw;

    int ret = adc_raw_to_millivolts(
        ADC_REF_MV,
        ADC_GAIN_1,
        ADC_RESOLUTION,
        &mv
    );

    if (ret < 0) {
        mv = ((int32_t)raw * ADC_REF_MV) / ((1 << ADC_RESOLUTION) - 1);
    }

    return mv;
}

static void handle_command(char *line)
{
    char cmd[96];

    strncpy(cmd, line, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';

    for (size_t i = 0; cmd[i] != '\0'; i++) {
        cmd[i] = (char)toupper((unsigned char)cmd[i]);
    }

    LOG_INF("Comando recebido: %s", cmd);

    if (strcmp(cmd, "STATUS") == 0) {
        k_mutex_lock(&print_mutex, K_FOREVER);
        printk("ACK,STATUS\n");
        k_mutex_unlock(&print_mutex);

        print_cfg();
        return;
    }

    if (strcmp(cmd, "RESET") == 0) {
        reset_system();

        k_mutex_lock(&print_mutex, K_FOREVER);
        printk("ACK,RESET\n");
        k_mutex_unlock(&print_mutex);

        print_cfg();
        return;
    }

    if (strcmp(cmd, "FILTER,NONE") == 0) {
        set_filter_none();

        k_mutex_lock(&print_mutex, K_FOREVER);
        printk("ACK,FILTER,NONE\n");
        k_mutex_unlock(&print_mutex);

        print_cfg();
        return;
    }

    if (strncmp(cmd, "RATE,", 5) == 0) {
        uint32_t rate = (uint32_t)atoi(&cmd[5]);

        if (set_rate(rate) == 0) {
            k_mutex_lock(&print_mutex, K_FOREVER);
            printk("ACK,RATE,%u\n", rate);
            k_mutex_unlock(&print_mutex);

            print_cfg();
        } else {
            k_mutex_lock(&print_mutex, K_FOREVER);
            printk("ERR,RATE_INVALIDA\n");
            k_mutex_unlock(&print_mutex);

            LOG_WRN("Frequencia invalida: %u", rate);
        }

        return;
    }

    if (strncmp(cmd, "FILTER,LOWPASS,", 15) == 0) {
        uint32_t cutoff = (uint32_t)atoi(&cmd[15]);

        if (set_lowpass(cutoff) == 0) {
            k_mutex_lock(&print_mutex, K_FOREVER);
            printk("ACK,FILTER,LOWPASS,%u\n", cutoff);
            k_mutex_unlock(&print_mutex);

            print_cfg();
        } else {
            k_mutex_lock(&print_mutex, K_FOREVER);
            printk("ERR,CUTOFF_INVALIDO\n");
            k_mutex_unlock(&print_mutex);

            LOG_WRN("Corte invalido: %u", cutoff);
        }

        return;
    }

    k_mutex_lock(&print_mutex, K_FOREVER);
    printk("ERR,COMANDO_DESCONHECIDO\n");
    k_mutex_unlock(&print_mutex);

    LOG_WRN("Comando desconhecido: %s", cmd);
}

static void acquisition_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t seq = 0;
    uint64_t last_stat_us = now_us();

    while (1) {
        int ret = adc_read(adc_dev, &adc_seq);

        if (ret == 0) {
            int16_t raw = adc_sample_buffer;
            int32_t mv = adc_raw_to_mv_safe(raw);
            int32_t out_mv = apply_filter(mv);
            uint64_t timestamp_us = now_us();

            atomic_inc(&produced_samples);

            k_mutex_lock(&print_mutex, K_FOREVER);

            printk(
                "DATA,%u,%llu,%d,%d,%d,%u\n",
                seq,
                (unsigned long long)timestamp_us,
                raw,
                mv,
                out_mv,
                0
            );

            k_mutex_unlock(&print_mutex);

            atomic_inc(&transmitted_samples);

            seq++;
        } else {
            atomic_inc(&adc_errors);
            LOG_ERR("Erro na leitura ADC: %d", ret);
        }

        uint64_t current_us = now_us();

        if ((current_us - last_stat_us) >= 1000000ULL) {
            uint32_t rate_snapshot;

            k_mutex_lock(&cfg_mutex, K_FOREVER);
            rate_snapshot = acq_rate_hz;
            k_mutex_unlock(&cfg_mutex);

            k_mutex_lock(&print_mutex, K_FOREVER);

            printk(
                "STAT,%llu,%d,%d,%d,%d,%u,%u\n",
                (unsigned long long)current_us,
                (int)atomic_get(&produced_samples),
                (int)atomic_get(&transmitted_samples),
                0,
                (int)atomic_get(&adc_errors),
                rate_snapshot,
                rate_snapshot
            );

            k_mutex_unlock(&print_mutex);

            last_stat_us = current_us;
        }

        uint32_t sleep_us;

        k_mutex_lock(&cfg_mutex, K_FOREVER);
        sleep_us = acq_period_us;
        k_mutex_unlock(&cfg_mutex);

        k_usleep(sleep_us);
    }
}

static void command_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    char line[96];
    size_t pos = 0;

    while (1) {
        uint8_t ch;

        if (uart_poll_in(uart_dev, &ch) == 0) {
            if (ch == '\n' || ch == '\r') {
                if (pos > 0) {
                    line[pos] = '\0';
                    handle_command(line);
                    pos = 0;
                }
            } else {
                if (pos < sizeof(line) - 1) {
                    line[pos++] = (char)ch;
                }
            }
        } else {
            k_msleep(5);
        }
    }
}

K_THREAD_STACK_DEFINE(acq_stack, ACQ_STACK_SIZE);
K_THREAD_STACK_DEFINE(cmd_stack, CMD_STACK_SIZE);

static struct k_thread acq_thread_data;
static struct k_thread cmd_thread_data;

int main(void)
{
    k_mutex_init(&cfg_mutex);
    k_mutex_init(&print_mutex);

    LOG_INF("Inicializando sistema");

    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC nao esta pronto");

        k_mutex_lock(&print_mutex, K_FOREVER);
        printk("ERR,ADC_NAO_PRONTO\n");
        k_mutex_unlock(&print_mutex);

        return 0;
    }

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART nao esta pronta");

        k_mutex_lock(&print_mutex, K_FOREVER);
        printk("ERR,UART_NAO_PRONTA\n");
        k_mutex_unlock(&print_mutex);

        return 0;
    }

    int ret = adc_channel_setup(adc_dev, &adc_cfg);

    if (ret < 0) {
        LOG_ERR("Erro no adc_channel_setup: %d", ret);

        k_mutex_lock(&print_mutex, K_FOREVER);
        printk("ERR,ADC_CHANNEL_SETUP,%d\n", ret);
        k_mutex_unlock(&print_mutex);

        return 0;
    }

    reset_system();

    k_mutex_lock(&print_mutex, K_FOREVER);
    printk("READY,FRDM_KL25Z_ADC_SERIAL_FILTER\n");
    k_mutex_unlock(&print_mutex);

    print_cfg();

    k_thread_create(
        &acq_thread_data,
        acq_stack,
        K_THREAD_STACK_SIZEOF(acq_stack),
        acquisition_thread,
        NULL,
        NULL,
        NULL,
        ACQ_PRIORITY,
        0,
        K_NO_WAIT
    );

    k_thread_create(
        &cmd_thread_data,
        cmd_stack,
        K_THREAD_STACK_SIZEOF(cmd_stack),
        command_thread,
        NULL,
        NULL,
        NULL,
        CMD_PRIORITY,
        0,
        K_NO_WAIT
    );

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
