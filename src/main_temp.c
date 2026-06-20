#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/time_units.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

#define ADC_NODE  DT_NODELABEL(adc0)
#define UART_NODE DT_CHOSEN(zephyr_console)

#if !DT_NODE_HAS_STATUS(ADC_NODE, okay)
#error "ADC0 nao esta habilitado no device tree."
#endif

#if !DT_NODE_HAS_STATUS(UART_NODE, okay)
#error "UART console nao esta habilitada no device tree."
#endif

/* ADC */
#define ADC_CHANNEL_ID 0
#define ADC_RESOLUTION 12
#define ADC_REF_MV     3300

/* Configuracao da aquisicao */
#define DEFAULT_ACQ_RATE_HZ 100U
#define DEFAULT_CUTOFF_HZ   5U
#define MIN_RATE_HZ         1U
#define MAX_RATE_HZ         2000U

/* Filtro FIR de media movel */
#define MAX_FIR_TAPS 128U

/* Fila entre a thread de aquisicao e a thread de comunicacao */
#define SAMPLE_QUEUE_LENGTH 16U

/* Threads */
#define ACQ_STACK_SIZE 1024
#define TX_STACK_SIZE  1536
#define CMD_STACK_SIZE 1024

/* Numero menor = prioridade maior no Zephyr. */
#define CMD_PRIORITY 4
#define ACQ_PRIORITY 5
#define TX_PRIORITY  5

#define COMMAND_BUFFER_SIZE 96U
#define STAT_PERIOD_US      1000000ULL

struct sample_message {
    uint32_t seq;
    uint64_t timestamp_us;
    int16_t raw_adc;
    int32_t mv;
    int32_t output_mv;
};

K_MSGQ_DEFINE(
    sample_queue,
    sizeof(struct sample_message),
    SAMPLE_QUEUE_LENGTH,
    4
);

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
static struct k_spinlock time_lock;

static uint32_t acq_rate_hz = DEFAULT_ACQ_RATE_HZ;
static uint32_t acq_period_us = 1000000U / DEFAULT_ACQ_RATE_HZ;

static bool filter_enabled;
static uint32_t cutoff_hz = DEFAULT_CUTOFF_HZ;
static uint16_t fir_taps;

static int32_t fir_buffer[MAX_FIR_TAPS];
static int64_t fir_sum;
static uint16_t fir_index;
static uint16_t fir_count;

/* Contadores compartilhados. */
static atomic_t sample_sequence;
static atomic_t produced_samples;
static atomic_t transmitted_samples;
static atomic_t queue_drops;
static atomic_t adc_errors;

/* Forca a thread de comunicacao a reiniciar a janela de medicao. */
static atomic_t stats_epoch;

/* Extensao do contador de ciclos de 32 bits para 64 bits. */
static uint32_t last_cycle_32;
static uint64_t cycle_base_64;

static uint64_t now_us(void)
{
    k_spinlock_key_t key = k_spin_lock(&time_lock);
    uint32_t current_cycle = k_cycle_get_32();

    if (current_cycle < last_cycle_32) {
        cycle_base_64 += (1ULL << 32);
    }

    last_cycle_32 = current_cycle;
    uint64_t extended_cycle = cycle_base_64 + current_cycle;

    k_spin_unlock(&time_lock, key);

    return k_cyc_to_us_floor64(extended_cycle);
}

static uint16_t compute_fir_taps(uint32_t rate_hz, uint32_t cutoff)
{
    if (cutoff == 0U) {
        return 1U;
    }

    uint32_t taps = rate_hz / cutoff;

    if (taps < 1U) {
        taps = 1U;
    }

    if (taps > MAX_FIR_TAPS) {
        taps = MAX_FIR_TAPS;
    }

    return (uint16_t)taps;
}

/* Deve ser chamada com cfg_mutex adquirido. */
static void clear_filter_state_locked(void)
{
    memset(fir_buffer, 0, sizeof(fir_buffer));
    fir_sum = 0;
    fir_index = 0U;
    fir_count = 0U;
}

static void print_cfg(void)
{
    uint32_t rate;
    uint32_t period;
    uint32_t cutoff;
    uint16_t taps;
    bool enabled;

    k_mutex_lock(&cfg_mutex, K_FOREVER);
    rate = acq_rate_hz;
    period = acq_period_us;
    cutoff = cutoff_hz;
    taps = fir_taps;
    enabled = filter_enabled;
    k_mutex_unlock(&cfg_mutex);

    k_mutex_lock(&print_mutex, K_FOREVER);
    printk(
        "CFG,%u,%u,%s,%u,%u\n",
        rate,
        period,
        enabled ? "LOWPASS_FIR" : "NONE",
        cutoff,
        enabled ? taps : 0U
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

    /* Evita que uma taxa STAT misture duas configuracoes diferentes. */
    atomic_inc(&stats_epoch);

    LOG_INF("Frequencia solicitada alterada para %u Hz", rate);
    return 0;
}

static int set_lowpass(uint32_t cutoff)
{
    if (cutoff == 0U) {
        return -EINVAL;
    }

    k_mutex_lock(&cfg_mutex, K_FOREVER);

    filter_enabled = true;
    cutoff_hz = cutoff;
    fir_taps = compute_fir_taps(acq_rate_hz, cutoff_hz);
    clear_filter_state_locked();

    k_mutex_unlock(&cfg_mutex);

    atomic_inc(&stats_epoch);

    LOG_INF(
        "Filtro FIR ativado: corte nominal=%u Hz, taps=%u",
        cutoff,
        fir_taps
    );

    return 0;
}

static void set_filter_none(void)
{
    k_mutex_lock(&cfg_mutex, K_FOREVER);

    filter_enabled = false;
    fir_taps = 0U;
    clear_filter_state_locked();

    k_mutex_unlock(&cfg_mutex);

    atomic_inc(&stats_epoch);
    LOG_INF("Filtro desativado");
}

static void reset_system(void)
{
    /* Remove amostras antigas ainda aguardando transmissao. */
    k_msgq_purge(&sample_queue);

    atomic_clear(&sample_sequence);
    atomic_clear(&produced_samples);
    atomic_clear(&transmitted_samples);
    atomic_clear(&queue_drops);
    atomic_clear(&adc_errors);

    k_mutex_lock(&cfg_mutex, K_FOREVER);

    acq_rate_hz = DEFAULT_ACQ_RATE_HZ;
    acq_period_us = 1000000U / DEFAULT_ACQ_RATE_HZ;

    filter_enabled = false;
    cutoff_hz = DEFAULT_CUTOFF_HZ;
    fir_taps = 0U;
    clear_filter_state_locked();

    k_mutex_unlock(&cfg_mutex);

    atomic_inc(&stats_epoch);
    LOG_INF("Sistema resetado");
}

static int32_t apply_filter(int32_t mv)
{
    int32_t output;

    k_mutex_lock(&cfg_mutex, K_FOREVER);

    if (!filter_enabled || fir_taps <= 1U) {
        output = mv;
        k_mutex_unlock(&cfg_mutex);
        return output;
    }

    fir_sum -= fir_buffer[fir_index];
    fir_buffer[fir_index] = mv;
    fir_sum += mv;

    fir_index++;
    if (fir_index >= fir_taps) {
        fir_index = 0U;
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
        mv = ((int32_t)raw * ADC_REF_MV) /
             ((1 << ADC_RESOLUTION) - 1);
    }

    return mv;
}

static void print_ack(const char *text)
{
    k_mutex_lock(&print_mutex, K_FOREVER);
    printk("ACK,%s\n", text);
    k_mutex_unlock(&print_mutex);
}

static void print_error(const char *text)
{
    k_mutex_lock(&print_mutex, K_FOREVER);
    printk("ERR,%s\n", text);
    k_mutex_unlock(&print_mutex);
}

static void handle_command(char *line)
{
    char cmd[COMMAND_BUFFER_SIZE];

    strncpy(cmd, line, sizeof(cmd) - 1U);
    cmd[sizeof(cmd) - 1U] = '\0';

    for (size_t i = 0U; cmd[i] != '\0'; i++) {
        cmd[i] = (char)toupper((unsigned char)cmd[i]);
    }

    LOG_INF("Comando recebido: %s", cmd);

    if (strcmp(cmd, "STATUS") == 0) {
        print_ack("STATUS");
        print_cfg();
        return;
    }

    if (strcmp(cmd, "RESET") == 0) {
        reset_system();
        print_ack("RESET");
        print_cfg();
        return;
    }

    if (strcmp(cmd, "FILTER,NONE") == 0) {
        set_filter_none();
        print_ack("FILTER,NONE");
        print_cfg();
        return;
    }

    if (strncmp(cmd, "RATE,", 5U) == 0) {
        uint32_t rate = (uint32_t)strtoul(&cmd[5], NULL, 10);

        if (set_rate(rate) == 0) {
            k_mutex_lock(&print_mutex, K_FOREVER);
            printk("ACK,RATE,%u\n", rate);
            k_mutex_unlock(&print_mutex);
            print_cfg();
        } else {
            print_error("RATE_INVALIDA");
            LOG_WRN("Frequencia invalida: %u", rate);
        }
        return;
    }

    if (strncmp(cmd, "FILTER,LOWPASS,", 15U) == 0) {
        uint32_t cutoff = (uint32_t)strtoul(&cmd[15], NULL, 10);

        if (set_lowpass(cutoff) == 0) {
            k_mutex_lock(&print_mutex, K_FOREVER);
            printk("ACK,FILTER,LOWPASS,%u\n", cutoff);
            k_mutex_unlock(&print_mutex);
            print_cfg();
        } else {
            print_error("CUTOFF_INVALIDO");
            LOG_WRN("Corte invalido: %u", cutoff);
        }
        return;
    }

    print_error("COMANDO_DESCONHECIDO");
    LOG_WRN("Comando desconhecido: %s", cmd);
}

static void acquisition_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    uint32_t previous_period_us = 0U;
    uint64_t next_deadline_us = now_us();

    while (1) {
        int ret = adc_read(adc_dev, &adc_seq);

        if (ret == 0) {
            struct sample_message sample;

            sample.seq = (uint32_t)atomic_inc(&sample_sequence);
            sample.timestamp_us = now_us();
            sample.raw_adc = adc_sample_buffer;
            sample.mv = adc_raw_to_mv_safe(sample.raw_adc);
            sample.output_mv = apply_filter(sample.mv);

            atomic_inc(&produced_samples);

            /*
             * A aquisicao nunca espera pela UART. Se a fila estiver cheia,
             * a amostra atual e descartada e a perda e contabilizada.
             */
            if (k_msgq_put(&sample_queue, &sample, K_NO_WAIT) != 0) {
                atomic_inc(&queue_drops);
            }
        } else {
            atomic_inc(&adc_errors);
            LOG_ERR("Erro na leitura ADC: %d", ret);
        }

        uint32_t period_us;

        k_mutex_lock(&cfg_mutex, K_FOREVER);
        period_us = acq_period_us;
        k_mutex_unlock(&cfg_mutex);

        uint64_t current_us = now_us();

        if (period_us != previous_period_us) {
            /* Reinicia o escalonamento quando RATE muda. */
            previous_period_us = period_us;
            next_deadline_us = current_us + period_us;
        } else {
            next_deadline_us += period_us;
        }

        if (next_deadline_us > current_us) {
            uint64_t wait_us = next_deadline_us - current_us;
            k_usleep((int32_t)wait_us);
        } else {
            /*
             * A thread atrasou. Nao adicionamos outro periodo inteiro ao
             * atraso e cedemos a CPU para a thread de transmissao.
             */
            if ((current_us - next_deadline_us) > (4ULL * period_us)) {
                next_deadline_us = current_us;
            }
            k_yield();
        }
    }
}

static void communication_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    struct sample_message sample;

    uint32_t observed_epoch = (uint32_t)atomic_get(&stats_epoch);
    uint64_t interval_start_us = now_us();
    uint32_t produced_start = (uint32_t)atomic_get(&produced_samples);
    uint32_t transmitted_start = (uint32_t)atomic_get(&transmitted_samples);

    while (1) {
        int ret = k_msgq_get(&sample_queue, &sample, K_MSEC(20));

        if (ret == 0) {
            uint32_t drops = (uint32_t)atomic_get(&queue_drops);

            k_mutex_lock(&print_mutex, K_FOREVER);
            printk(
                "DATA,%u,%llu,%d,%d,%d,%u\n",
                sample.seq,
                (unsigned long long)sample.timestamp_us,
                sample.raw_adc,
                sample.mv,
                sample.output_mv,
                drops
            );
            k_mutex_unlock(&print_mutex);

            atomic_inc(&transmitted_samples);

            /* Permite alternancia com a aquisicao, pois ambas tem prioridade 5. */
            k_yield();
        }

        uint64_t current_us = now_us();
        uint32_t current_epoch = (uint32_t)atomic_get(&stats_epoch);

        if (current_epoch != observed_epoch) {
            observed_epoch = current_epoch;
            interval_start_us = current_us;
            produced_start = (uint32_t)atomic_get(&produced_samples);
            transmitted_start = (uint32_t)atomic_get(&transmitted_samples);
            continue;
        }

        uint64_t elapsed_us = current_us - interval_start_us;

        if (elapsed_us >= STAT_PERIOD_US) {
            uint32_t produced_now =
                (uint32_t)atomic_get(&produced_samples);
            uint32_t transmitted_now =
                (uint32_t)atomic_get(&transmitted_samples);

            uint32_t produced_delta = produced_now - produced_start;
            uint32_t transmitted_delta = transmitted_now - transmitted_start;

            uint32_t effective_acq_rate = (uint32_t)(
                ((uint64_t)produced_delta * 1000000ULL) / elapsed_us
            );

            uint32_t effective_tx_rate = (uint32_t)(
                ((uint64_t)transmitted_delta * 1000000ULL) / elapsed_us
            );

            uint32_t drops = (uint32_t)atomic_get(&queue_drops);
            uint32_t errors = (uint32_t)atomic_get(&adc_errors);

            k_mutex_lock(&print_mutex, K_FOREVER);
            printk(
                "STAT,%llu,%u,%u,%u,%u,%u,%u\n",
                (unsigned long long)current_us,
                produced_now,
                transmitted_now,
                drops,
                errors,
                effective_acq_rate,
                effective_tx_rate
            );
            k_mutex_unlock(&print_mutex);

            interval_start_us = current_us;
            produced_start = produced_now;
            transmitted_start = transmitted_now;
        }
    }
}

static void command_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    char line[COMMAND_BUFFER_SIZE];
    size_t pos = 0U;

    while (1) {
        uint8_t ch;

        if (uart_poll_in(uart_dev, &ch) == 0) {
            if (ch == '\n' || ch == '\r') {
                if (pos > 0U) {
                    line[pos] = '\0';
                    handle_command(line);
                    pos = 0U;
                }
            } else if (pos < (sizeof(line) - 1U)) {
                line[pos++] = (char)ch;
            } else {
                /* Descarta comandos maiores que o buffer. */
                pos = 0U;
                print_error("COMANDO_MUITO_LONGO");
            }
        } else {
            k_msleep(2);
        }
    }
}

K_THREAD_STACK_DEFINE(acq_stack, ACQ_STACK_SIZE);
K_THREAD_STACK_DEFINE(tx_stack, TX_STACK_SIZE);
K_THREAD_STACK_DEFINE(cmd_stack, CMD_STACK_SIZE);

static struct k_thread acq_thread_data;
static struct k_thread tx_thread_data;
static struct k_thread cmd_thread_data;

int main(void)
{
    k_mutex_init(&cfg_mutex);
    k_mutex_init(&print_mutex);

    LOG_INF("Inicializando sistema");

    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC nao esta pronto");
        print_error("ADC_NAO_PRONTO");
        return 0;
    }

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART nao esta pronta");
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
    printk("READY,FRDM_KL25Z_ADC_QUEUE_FIR\n");
    k_mutex_unlock(&print_mutex);

    print_cfg();

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
        &tx_thread_data,
        tx_stack,
        K_THREAD_STACK_SIZEOF(tx_stack),
        communication_thread,
        NULL,
        NULL,
        NULL,
        TX_PRIORITY,
        0,
        K_NO_WAIT
    );

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
