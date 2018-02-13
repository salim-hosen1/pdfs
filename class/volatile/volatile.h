/*
 * const and volatile, new for ANSI (ca. 1988)
 *
"The purpose of const is to announce objects that may be placed in read-only memory, and perhaps 
to increase opportunities for optimization. The purpose of volatile is to force an implementation
to suppress optimization that could otherwise occur. For example, for a machine with memory-mapped 
input/output, a pointer to a device register might be declared as a pointer to volatile, in order 
to prevent the compiler from removing apparently redundant references through the pointer."
--Reference Manual, Appendix A, from The C Programming Language, page 211

ISO Standard:

"A volatile declaration may be used to describe an object corresponding to a memory-mapped
input/output port or an object accessed by an asynchronously interrupting function. Actions on
objects so declared shall not be ‘‘optimized out’’ by an implementation or reordered except as
permitted by the rules for evaluating expressions."
--Footnote 134, N1570 ISO Standard, 6.7.3 Type Qualifiers (pg. 122)

References:

See talk on YouTube by Herb Sutter, "Atomic Weapons."
-- Helpful primer on C/C++11 Memory Model and processor pipeline optimizations
-- Processors try to defer going to main memory, prefer cached data
-- Hardware exposes memory barrier intrinsics to prevent this
-- Lock free programming using relaxed atomics (std::atomic C++)
  
Also Ch. 5 in C++ Concurrency in Action, Practical Multithreading, by Anthony Williams
-- C\C++11 Memory Model

Finally, in the Linux Kernel Documentation:
-- Why the "volatile" type class should not be used (in the Linux Kernel)
-- Documents/memory-barriers.txt, Howells, McKenney, Deacon, Zijlstra
-- Unreliable Guide to Locking, Rusty Russell
*/

/* Memory mapped register addresses from STM32F401xx headers */ 

#define FLASH_BASE            0x08000000U /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define SRAM1_BASE            0x20000000U /*!< SRAM1(64 KB) base address in the alias region                              */
#define PERIPH_BASE           0x40000000U /*!< Peripheral base address in the alias region                                */
#define BKPSRAM_BASE          0x40024000U /*!< Backup SRAM(4 KB) base address in the alias region                         */
#define SRAM1_BB_BASE         0x22000000U /*!< SRAM1(64 KB) base address in the bit-band region                           */
#define PERIPH_BB_BASE        0x42000000U /*!< Peripheral base address in the bit-band region                             */
#define BKPSRAM_BB_BASE       0x42480000U /*!< Backup SRAM(4 KB) base address in the bit-band region                      */
#define FLASH_END             0x0803FFFFU /*!< FLASH end address                                                          */
#define FLASH_OTP_BASE        0x1FFF7800U /*!< Base address of : (up to 528 Bytes) embedded FLASH OTP Area                */
#define FLASH_OTP_END         0x1FFF7A0FU /*!< End address of : (up to 528 Bytes) embedded FLASH OTP Area                 */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000U)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000U)

/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000U)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400U)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800U)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00U)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800U)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00U)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000U)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x3400U)
/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000U)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00U)


/* GPIOA is a pointer to struct with volatile fields */
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)

/*__IO is a typedef for volatile */

typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

/* consider this constant name PORTC meant to point to GPIO_TypeDef */
PORTC           equ 0x40020800    
IDR             equ 0x10  /*offset GPIO port input data register*/
PORTC_IDR       equ PORTC + IDR /*for reading button state*/

/* consider this "polling" code that spins in a loop or fires on timer interrupt.
 * The code is checking to see if a button is pressed. */
PRESSED         equ 0x00002000

scan_idr:
        ldr     r2, =PORTC_IDR  /* fetch addr from mapped memory */
        ldr     r0, [r2]        /* dereference, get the value */
        ldr     r1, =PRESSED    /* setting up a compare*/
        cmp     r0, r1
        ite     eq
        /* if equal clause */
        /* else, clause */

/*
This is the canonical use case for volatile. A volatile object can be changed by an
external context, in our case, by the press of a button that sets some bits on an
input register.  This tells the optimizer not to generate code that perhaps caches
the value in an unused register for future use rather than fetch from memory on
each load call.  
*/
