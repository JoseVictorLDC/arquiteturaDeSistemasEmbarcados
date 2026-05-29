#include "MKL25Z4.h"

// Define os pinos dos LEDs
#define LED_GREEN_PIN  19 // PTB19
#define LED_BLUE_PIN   1  // PTD1
#define ADC_CHANNEL    8  // PTB0 = ADC0_SE8

void init_clocks(void) {
    // 1. Habilita o clock para as portas B e D (onde estão os LEDs e o pino do ADC)
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;
    
    // 2. Habilita o clock para o ADC0 (Como discutimos nas mensagens anteriores!)
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
}

void init_gpio(void) {
    // 1. Configura o multiplexador dos pinos para função GPIO (MUX = 1)
    PORTB->PCR[LED_GREEN_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED_BLUE_PIN]  = PORT_PCR_MUX(1);
    
    // 2. Configura a direção dos pinos como SAÍDA (Output = 1)
    FGPIOB->PDDR |= (1 << LED_GREEN_PIN);
    FGPIOD->PDDR |= (1 << LED_BLUE_PIN);
    
    // 3. Desliga os LEDs inicialmente (Lembra? Active-Low: 1 = Desligado)
    FGPIOB->PSOR = (1 << LED_GREEN_PIN); // PSOR = Port Set Output Register
    FGPIOD->PSOR = (1 << LED_BLUE_PIN);
}

void init_adc0(void) {
    // 1. Configura o pino PTB0 como Analógico (MUX = 0)
    PORTB->PCR[0] = PORT_PCR_MUX(0);
    
    // 2. Configura o ADC: Clock do barramento (Bus Clock), Resolução de 12 bits
    // MODE(1) = 12 bits | ADICLK(0) = Bus Clock
    ADC0->CFG1 = ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(0);
    
    // 3. Configuração padrão (Trigger por software, Referência de tensão padrão)
    ADC0->SC2 = 0; 
    ADC0->SC3 = 0;
}

uint16_t adc_read(uint8_t channel) {
    // 1. Escreve no registrador SC1A para iniciar a conversão no canal desejado
    ADC0->SC1[0] = channel & ADC_SC1_ADCH_MASK;
    
    // 2. Aguarda a flag COCO (Conversion Complete) ficar igual a 1
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {
        // Fica preso aqui em polling até a conversão terminar
    }
    
    // 3. Retorna o resultado (limpa automaticamente a flag COCO ao ler R[0])
    return ADC0->R[0];
}

int main(void) {
    uint16_t adc_value = 0;
    
    // Inicializa o Hardware
    init_clocks();
    init_gpio();
    init_adc0();
    
    while (1) {
        // Dispara e lê o canal 8 (PTB0)
        adc_value = adc_read(ADC_CHANNEL);
        
        // --- Lógica de Decisão ---
        // Resolução de 12 bits vai de 0 a 4095. 
        // 4095 equivale a VREF (~3.3V). 0 equivale ao GND (0V).
        
        if (adc_value > 3500) { 
            // Próximo a 3.3V -> Acende AZUL, apaga VERDE
            FGPIOD->PCOR = (1 << LED_BLUE_PIN);  // PCOR (Clear) coloca o pino em 0 -> Acende!
            FGPIOB->PSOR = (1 << LED_GREEN_PIN); // PSOR (Set) coloca o pino em 1 -> Apaga!
            
        } else if (adc_value < 500) {
            // Próximo a 0V -> Acende VERDE, apaga AZUL
            FGPIOB->PCOR = (1 << LED_GREEN_PIN); // Verde ON
            FGPIOD->PSOR = (1 << LED_BLUE_PIN);  // Azul OFF
            
        } else {
            // Valores intermediários (Apaga ambos para evitar estado indefinido)
            FGPIOB->PSOR = (1 << LED_GREEN_PIN); // Verde OFF
            FGPIOD->PSOR = (1 << LED_BLUE_PIN);  // Azul OFF
        }
        
        // Um pequeno delay para não saturar o processador (loop rudimentar)
        for(volatile int i = 0; i < 50000; i++);
    }
    return 0;
}
