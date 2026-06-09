#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

#include <stdlib.h>
#include <errno.h>

/* ============================================================
 * CONFIGURAÇÕES GERAIS
 * ============================================================ */

#define ADC_THREAD_STACK_SIZE    1024
#define ACCEL_THREAD_STACK_SIZE  1024

/* Mesma prioridade inicial, como pedido na atividade */
#define ADC_THREAD_PRIORITY      5
#define ACCEL_THREAD_PRIORITY    4

/* ============================================================
 * ADC
 * ============================================================ */

#define ADC_RESOLUTION           12
#define ADC_GAIN                 ADC_GAIN_1
#define ADC_REFERENCE            ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME     ADC_ACQ_TIME_DEFAULT
#define ADC_CHANNEL_ID           0
#define ADC_VREF_MV              3300

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc0));
static int16_t sample_buffer;

static struct adc_channel_cfg adc_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_ID,
    .differential = 0,
};

static struct adc_sequence adc_sequence_cfg = {
    .channels = BIT(ADC_CHANNEL_ID),
    .buffer = &sample_buffer,
    .buffer_size = sizeof(sample_buffer),
    .resolution = ADC_RESOLUTION,
};

/* ============================================================
 * ACELERÔMETRO MMA8451Q
 * ============================================================ */

static const struct device *accel = DEVICE_DT_GET(DT_NODELABEL(mma8451q));

/* ============================================================
 * BOTÃO
 * ============================================================ */

/*
 * Primeiro tenta usar o alias sw0.
 * Se sua placa/projeto usa user_button_0, troque a linha abaixo por:
 *
 * #define BUTTON_NODE DT_NODELABEL(user_button_0)
 */
#define BUTTON_NODE DT_NODELABEL(user_button_0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static struct gpio_callback button_cb_data;

/* 
 * modo_completo = 0 -> mostra apenas ADC
 * modo_completo = 1 -> mostra ADC + acelerômetro
 */
static atomic_t modo_completo = ATOMIC_INIT(0);
static atomic_t modo_mudou = ATOMIC_INIT(0);

static int64_t ultimo_aperto_ms = 0;

/* Mutex para evitar prints misturados entre as threads */
K_MUTEX_DEFINE(print_mutex);

/* ============================================================
 * STACKS E THREADS
 * ============================================================ */

K_THREAD_STACK_DEFINE(adc_stack, ADC_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(accel_stack, ACCEL_THREAD_STACK_SIZE);

static struct k_thread adc_thread_data;
static struct k_thread accel_thread_data;

/* ============================================================
 * INTERRUPÇÃO DO BOTÃO
 * ============================================================ */

static void button_isr(const struct device *dev,
                       struct gpio_callback *cb,
                       uint32_t pins)
{
    int64_t agora_ms = k_uptime_get();

    /* Debounce simples de 200 ms */
    if ((agora_ms - ultimo_aperto_ms) < 200) {
        return;
    }

    ultimo_aperto_ms = agora_ms;

    if (atomic_get(&modo_completo)) {
        atomic_set(&modo_completo, 0);
    } else {
        atomic_set(&modo_completo, 1);
    }

    atomic_set(&modo_mudou, 1);
}

/* ============================================================
 * THREAD DO ADC
 * ============================================================ */

static void adc_thread(void *p1, void *p2, void *p3)
{
    while (1) {
        if (atomic_cas(&modo_mudou, 1, 0)) {
            if (atomic_get(&modo_completo)) {
                printk("\n[Botao] Modo COMPLETO: ADC + acelerometro\n\n");
            } else {
                printk("\n[Botao] Modo ADC: apenas ADC\n\n");
            }
        }

        int err = adc_read(adc_dev, &adc_sequence_cfg);

        if (err != 0) {
            printk("[ADC] Falha na leitura: %d\n", err);
        } else {
            int32_t mv = sample_buffer;

            adc_raw_to_millivolts(
                ADC_VREF_MV,
                ADC_GAIN,
                ADC_RESOLUTION,
                &mv
            );

            printk("[ADC] raw: %d", sample_buffer);
            k_yield();
            printk(" | tensao: %d mV\n", mv);
        }

        k_sleep(K_MSEC(500));
    }
}

/* ============================================================
 * THREAD DO ACELERÔMETRO
 * ============================================================ */

static void accel_thread(void *p1, void *p2, void *p3)
{
    struct sensor_value accel_x;
    struct sensor_value accel_y;
    struct sensor_value accel_z;

    while (1) {
        if (atomic_get(&modo_completo)) {
            int ret = sensor_sample_fetch(accel);

            if (ret != 0) {
                printk("[ACCEL] Erro ao ler sensor: %d\n", ret);
            } else {
                sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &accel_x);
                sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &accel_y);
                sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &accel_z);

                printk("[ACCEL] X: %d.%06d", accel_x.val1, abs(accel_x.val2));
                k_yield();
                printk(" | Y: %d.%06d", accel_y.val1, abs(accel_y.val2));
                k_yield();
                printk(" | Z: %d.%06d\n", accel_z.val1, abs(accel_z.val2));
            }
        }

        k_sleep(K_MSEC(1000));
    }
}

/* ============================================================
 * INICIALIZAÇÃO DO ADC
 * ============================================================ */

static int inicializar_adc(void)
{
    if (!device_is_ready(adc_dev)) {
        printk("ERRO: ADC nao esta pronto\n");
        return -ENODEV;
    }

    int ret = adc_channel_setup(adc_dev, &adc_channel_cfg);

    if (ret != 0) {
        printk("ERRO: falha ao configurar canal ADC: %d\n", ret);
        return ret;
    }

    printk("ADC inicializado com sucesso\n");
    return 0;
}

/* ============================================================
 * INICIALIZAÇÃO DO ACELERÔMETRO
 * ============================================================ */

static int inicializar_acelerometro(void)
{
    if (!device_is_ready(accel)) {
        printk("ERRO: acelerometro nao esta pronto\n");
        return -ENODEV;
    }

    printk("Acelerometro inicializado com sucesso\n");
    return 0;
}

/* ============================================================
 * INICIALIZAÇÃO DO BOTÃO
 * ============================================================ */

static int inicializar_botao(void)
{
    int ret;

    if (!gpio_is_ready_dt(&button)) {
        printk("ERRO: botao nao esta pronto\n");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);

    if (ret != 0) {
        printk("ERRO: falha ao configurar botao: %d\n", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_FALLING);

    if (ret != 0) {
        printk("ERRO: falha ao configurar interrupcao do botao: %d\n", ret);
        return ret;
    }

    gpio_init_callback(&button_cb_data, button_isr, BIT(button.pin));

    ret = gpio_add_callback(button.port, &button_cb_data);

    if (ret != 0) {
        printk("ERRO: falha ao adicionar callback do botao: %d\n", ret);
        return ret;
    }

    printk("Botao com interrupcao inicializado com sucesso\n");
    return 0;
}

/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    printk("\n");
    printk("========================================\n");
    printk(" PSI3441 - Atividade 6 - Threads\n");
    printk(" ADC + MMA8451Q + Botao com interrupcao\n");
    printk("========================================\n\n");

    if (inicializar_adc() != 0) {
        return 0;
    }

    if (inicializar_acelerometro() != 0) {
        return 0;
    }

    if (inicializar_botao() != 0) {
        return 0;
    }

    printk("\nModo inicial: ADC\n");
    printk("Pressione o botao para alternar entre:\n");
    printk("- Modo ADC\n");
    printk("- Modo COMPLETO\n\n");

    k_thread_create(
        &adc_thread_data,
        adc_stack,
        K_THREAD_STACK_SIZEOF(adc_stack),
        adc_thread,
        NULL,
        NULL,
        NULL,
        ADC_THREAD_PRIORITY,
        0,
        K_NO_WAIT
    );

    k_thread_create(
        &accel_thread_data,
        accel_stack,
        K_THREAD_STACK_SIZEOF(accel_stack),
        accel_thread,
        NULL,
        NULL,
        NULL,
        ACCEL_THREAD_PRIORITY,
        0,
        K_NO_WAIT
    );

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
