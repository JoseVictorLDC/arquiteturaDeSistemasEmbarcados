// DEFINIÇÃO DAS MACROS
// Define o endereço do registrador SIM_SCGC5 (CLOCK)
#define SIM_SCGC5 (*((volatile unsigned int*)0x40048038))
// Define o endereço do registrador PORTB_PCR19 (LED VERDE)
#define PORTB_PCR19 (*((volatile unsigned int*)0x4004A04C))
// Define o endereço do registrador GPIOB_PDDR (INPUT/OUTPUT)
#define GPIOB_PDDR (*((volatile unsigned int*)0x400FF054))
// Define o endereço do registrador GPIOB_PSOR (SET)
#define GPIOB_PSOR (*((volatile unsigned int*)0x400FF044))
// Define o endereço do registrador GPIOB_PCOR (CLEAN)
#define GPIOB_PCOR (*((volatile unsigned int*)0x400FF048)) 

void delayMs (int n) {
	volatile int i;
	volatile int j;
	for (i = 0; i < n; i++)
		for (j = 0; j < 7000; j++) {}
}

void main() {
    SIM_SCGC5 |= 0x0400;

    // Configurar LED VERDE
    PORTB_PCR19 = 0x0100; 
    GPIOB_PDDR |= (1 << 19);

    while (1)
    {
        // Habilitar saída
        GPIOB_PSOR |= (1 << 19);
        // Delay
        delayMs(1000);

        // Desabilitar saída
        GPIOB_PCOR |= (1 << 19);
        // Delay
        delayMs(1000);
    }
}
