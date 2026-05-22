#include "pwm_z42.h"
 
bool pwm_tpm_Init(TPM_MemMapPtr tpm, uint16_t clk, uint16_t module, uint8_t clock_mode,
                  uint8_t ps, bool counting_mode)
{
	if(tpm == TPM0)
	{
		// ANTES: SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;
		SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	}
	else if(tpm == TPM1)
	{
		// ANTES: SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
		SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	}
	else if(tpm == TPM2)
	{
		// ANTES: SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK;
		SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	}
	else
	{
		return false;
	}

	// ANTES: SIM_SOPT2 |= SIM_SOPT2_TPMSRC(clk);
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(clk);

	tpm->MOD = module;

	tpm->SC |= TPM_SC_CMOD(clock_mode) | TPM_SC_PS(ps);

	if(counting_mode == CENTER_PWM)
	{
		tpm->SC |= TPM_SC_CPWMS_MASK;
	}
	else if(counting_mode == EDGE_PWM)
	{
		tpm->SC &= ~TPM_SC_CPWMS_MASK;
	}
	else
	{
		return false;
	}
	return true;
}
/****************************************************************************************
*
*****************************************************************************************/
bool pwm_tpm_Ch_Init(TPM_MemMapPtr tpm, uint16_t channel, uint8_t mode,
                     GPIO_MemMapPtr gpio,uint8_t pin)
{
	if(tpm == TPM0)
	{
		if(gpio == GPIOA)
		{
			if((channel<=2)||(channel==5))
			{
				if((pin>=3) || (pin<=5))
				{
					// ANTES: SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
					SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
					// ANTES: PORT_PCR_REG(PORTA_BASE_PTR,pin) = PORT_PCR_MUX(3);
					PORTA->PCR[pin] = PORT_PCR_MUX(3);
				}
				else return false;
			}
			else return false;
		}
		else if(gpio == GPIOC)
		{
			if(channel<=5)
			{
				if((pin==8) || (pin==9))
				{
					SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
					PORTC->PCR[pin] = PORT_PCR_MUX(3);
				}
				if((pin >= 1 && pin <= 4))
				{
					SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
					PORTC->PCR[pin] = PORT_PCR_MUX(4);
				}
				else return false;
			}
			else return false;
		}
		else if(gpio == GPIOD)
		{
			if(channel<=5)
			{
				if(pin<=5)
				{
					SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
					PORTD->PCR[pin] = PORT_PCR_MUX(4);
				}
				else return false;
			}
			else return false;
		}
		else if(gpio == GPIOE)
		{
			if(channel<=4)
			{
				if( (pin>=24 || pin<=25) || (pin>=29 || pin<=31) )
				{
					SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
					PORTE->PCR[pin] = PORT_PCR_MUX(3);
				}
				else return false;
			}
			else return false;
		}
		else return false;
	}
	else if(tpm == TPM1)
	{
		// ... E assim por diante para todos os outros...
        // O código abaixo já foi corrigido seguindo o mesmo padrão
		if(channel <= 1)
		{
			if(gpio == GPIOA)
			{
				if(pin>=12 || pin<=13)
				{
					SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
					PORTA->PCR[pin] = PORT_PCR_MUX(3);
				}
				else return false;
			}
			else if(gpio == GPIOB)
			{
				if(pin<=1)
				{
					SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
					PORTB->PCR[pin] = PORT_PCR_MUX(3);
				}
				else return false;
			}
			else if(gpio == GPIOE)
			{
				if(pin>=20 || pin<=21)
				{
					SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
					PORTE->PCR[pin] = PORT_PCR_MUX(3);
				}
				else return false;
			}
			else return false;
		}
		else return false;
	}
	else if(tpm == TPM2)
	{
		if(channel <= 1)
		{
			if(gpio == GPIOA)
			{
				if(pin>=1 || pin<=2)
				{
					SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
					PORTA->PCR[pin] = PORT_PCR_MUX(3);
				}
				else return false;
			}
			else if(gpio == GPIOB)
			{
				if((pin>=2||pin<=3) || (pin>=18||pin<19))
				{
					SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
					PORTB->PCR[pin] = PORT_PCR_MUX(3);
				}
				else return false;
			}
			else if(gpio == GPIOE)
			{
				if(pin>=22 || pin<=23)
				{
					SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
					PORTE->PCR[pin] = PORT_PCR_MUX(3);
				}
				else return false;
			}
			else return false;
		}
		else return false;
	}
	else return false;

	// Esta linha já estava correta da nossa depuração anterior
	tpm->CONTROLS[channel].CnSC |= mode;

	return true;
}
/****************************************************************************************
*
*****************************************************************************************/
void pwm_tpm_CnV(TPM_MemMapPtr tpm, uint16_t channel, uint16_t value)
{
	// Esta linha já estava correta da nossa depuração anterior
	tpm->CONTROLS[channel].CnV = value;
}
/***************************************************************************************/