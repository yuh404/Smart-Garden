#ifndef __HCSR04_H
#define __HCSR04_H

//#define	TIMER2_CR1					(*((volatile unsigned long *)  0x40000000))
//#define TIMER2_PSC					(*((volatile unsigned long *)  0x40000028))
//#define TIMER2_ARR					(*((volatile unsigned long *)  0x4000002C))
//#define TIMER2_DIER					(*((volatile unsigned long *)  0x4000000C))
//#define	TIMER2_CCMR1				(*((volatile unsigned long *)  0x40000018))
//#define	TIMER2_CCER					(*((volatile unsigned long *)  0x40000020))
//#define	TIMER2_CCR1					(*((volatile unsigned long *)  0x40000034))
//#define	TIMER2_SR						(*((volatile unsigned long *)  0x40000010))
//	
//#define NVIC_ISER1          (*((volatile unsigned long *)  0xE000E100))
	
// NVIC bits
#define NVIC_ISER1_RTC_Pos  		(3)        // RTC global interrupt position (IRQ #3 trong ISER1)

// TIMER 2
#define PSC_VALUE 	(71UL) 	// Giá tr? Prescaler: 71
#define CC1S_MASK		(0x3UL)		// Mask 2 bit cho CC1S (Bit 0 và 1)
#define CC1S_TI1		(0x1UL)		// Giá tr? CC1S = 01 (Input Capture)
#define CC1E_BIT		(1UL<<0)	// CC1E (Bit 0 c?a CCER)
#define CC1IE_BIT 	(1UL<<1)	// CC1IE (Bit 1 c?a DIER)
#define CEN_BIT			(0x01UL)	// CEN (Bit 0 c?a CR1)

void TIMER2_INIT();
void HC_SR04_TRIG ();
void HC_SR04_IRQHANDLER();
void HCSR04_Trig (void);


#endif /* __HCSR04_H */