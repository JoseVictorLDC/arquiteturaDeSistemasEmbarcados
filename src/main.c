#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define TEMPO_VERDE_MS    5000
#define TEMPO_AMARELO_MS  2000
#define TEMPO_VERMELHO_MS 5000

#define LED_VERDE_NODE    DT_ALIAS(led0)
#define LED_AZUL_NODE     DT_ALIAS(led1)
#define LED_VERMELHO_NODE DT_ALIAS(led2)

#if !DT_NODE_HAS_STATUS(LED_VERDE_NODE, okay)
#error "led0 não está definido no Device Tree"
#endif

#if !DT_NODE_HAS_STATUS(LED_AZUL_NODE, okay)
#error "led1 não está definido no Device Tree"
#endif

#if !DT_NODE_HAS_STATUS(LED_VERMELHO_NODE, okay)
#error "led2 não está definido no Device Tree"
#endif

static const struct gpio_dt_spec led_verde =
    GPIO_DT_SPEC_GET(LED_VERDE_NODE, gpios);

static const struct gpio_dt_spec led_azul =
    GPIO_DT_SPEC_GET(LED_AZUL_NODE, gpios);

static const struct gpio_dt_spec led_vermelho =
    GPIO_DT_SPEC_GET(LED_VERMELHO_NODE, gpios);

static int configurar_led(const struct gpio_dt_spec *led)
{
    if (!gpio_is_ready_dt(led)) {
        printk("Erro: GPIO %s não está pronto\n", led->port->name);
        return -1;
    }

    return gpio_pin_configure_dt(led, GPIO_OUTPUT_INACTIVE);
}

static void semaforo_set(int verde, int azul, int vermelho)
{
    gpio_pin_set_dt(&led_verde, verde);
    gpio_pin_set_dt(&led_azul, azul);
    gpio_pin_set_dt(&led_vermelho, vermelho);
}

int main(void)
{
    int ret;

    ret = configurar_led(&led_verde);
    if (ret < 0) {
        return ret;
    }

    ret = configurar_led(&led_azul);
    if (ret < 0) {
        return ret;
    }

    ret = configurar_led(&led_vermelho);
    if (ret < 0) {
        return ret;
    }

    printk("Semaforo iniciado\n");

    while (1) {
        // Verde ligado
        semaforo_set(1, 0, 0);
        k_msleep(TEMPO_VERDE_MS);

        // "Amarelo" como combinação de verde + azul
        semaforo_set(1, 0, 1);
        k_msleep(TEMPO_AMARELO_MS);

        // Vermelho ligado
        semaforo_set(0, 0, 1);
        k_msleep(TEMPO_VERMELHO_MS);
    }

    return 0;
}