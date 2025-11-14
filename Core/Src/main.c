/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SQRT3_DIV_3 0.57735027f  // 1 / sqrt(3)
#define SQRT3_DIV_2 0.86602540f  // sqrt(3) / 2
#define TWO_DIV_3   0.66666667f  // 2 / 3
#define PI 3.14159265f
#define TWO_PI 6.2831853f
// limites do integrador (exemplos)
#define SERROW_MAX  10.0f
#define SERROW_MIN -10.0f
#define SERRQ_MAX   10.0f
#define SERRQ_MIN  -10.0f
#define SERRD_MAX   10.0f
#define SERRD_MIN  -10.0f



#define R_PHASE 0.2f
#define L_PHASE 0.2f

#define F_PWM 42457.5f
#define Fs (F_PWM / 10.0f)
#define Ts (1.0f / Fs)

#define BEMF_FILTER_ALPHA 0.02f
#define CURRENT_FILTER_ALPHA 0.25f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac1;

UART_HandleTypeDef hlpuart1;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint16_t adc1_buffer[2];
uint16_t adc2_buffer[4];

volatile int flagdadosprontos=0;
volatile int opamp1selector=0;
volatile int opamp2selector=0;
volatile int itcounter=0;

volatile uint16_t offset_iu_raw_op2=0;
volatile uint16_t offset_iv_raw=0;
volatile uint16_t offset_iw_raw=0;
volatile uint16_t offset_iu_raw_op1=0;
volatile int observador = 0;
volatile float iu=0;
volatile float iv=0;
volatile float iw=0;
volatile float vu=0;
volatile float vv=0;
volatile float vw=0;
volatile float vbus=0;

volatile uint16_t iu_raw;
volatile uint16_t iu_raw_op1;
volatile uint16_t iu_raw_op2;
volatile uint16_t iv_raw;
volatile uint16_t iw_raw;
volatile uint16_t vu_raw;
volatile uint16_t vv_raw;
volatile uint16_t vw_raw;
volatile uint16_t vbus_raw;
volatile float va=0;
volatile float vb=0;
volatile float vd=0;
volatile float vq=0;


volatile float wref=0;
volatile float wch=0;
volatile float w=0;
volatile float dw=0;
volatile float errow=0;
volatile float serrow=0;

volatile float iq=0;
volatile float id=0;

volatile float ia_filtrada = 0.0f;
volatile float ib_filtrada = 0.0f;
volatile float erroiq=0;
volatile float erroid=0;
volatile float serroiq=0;
volatile float serroid=0;

volatile float diqref=0;
volatile float iqref=0;

volatile float didref=0;
volatile float idref=0;


volatile float ia=0;
volatile float ib=0;
volatile float ea=0;
volatile float eb=0;
volatile float dia=0;
volatile float dib=0;
volatile float dea=0;
volatile float deb=0;

volatile float iach=0;
volatile float diach=0;
volatile float each=0;
volatile float deach=0;
volatile float erroia=0;

volatile float ibch=0;
volatile float dibch=0;
volatile float ebch=0;
volatile float debch=0;
volatile float erroib=0;

volatile float vdref=0;
volatile float vqref=0;



volatile float ia_prev = 0.0f; // Corrente alpha do ciclo anterior
volatile float ib_prev = 0.0f; // Corrente beta do ciclo anterior

volatile float ea_filtrada = 0.0f; // BEMF alpha FILTRADA
volatile float eb_filtrada = 0.0f; // BEMF beta FILTRADA

volatile float thetavelho=0;
volatile float theta=0;
volatile float dtheta=0;
volatile float thetach=0;

float pll_kp = 20.0f; // Ganho Proporcional do PLL (Sintonizável)
float pll_ki = 2000.0f; // Ganho Integral do PLL (Sintonizável)
float pll_angle_error_integral = 0.0f; // Integrador do PI do PLL
float pll_estimated_w = 0.0f;     // (w) Velocidade estimada pelo PLL
float pll_estimated_theta = 0.0f; // (theta) Ângulo estimado pelo PLL

float smo_z_alpha = 0.0f;     // BEMF Alfa estimado (Saída do SMO)
float smo_z_beta = 0.0f;      // BEMF Beta estimado (Saída do SMO)
float smo_ia_est = 0.0f;    // Corrente Alfa estimada (Interna do SMO)
float smo_ib_est = 0.0f;    // Corrente Beta estimada (Interna do SMO)
float smo_gain = 200.0f;    // Ganho do SMO (Sintonizável, ex: 200.0f)


//ganhos chutados

volatile float kpw = 0.1f; // Ganho Proporcional
volatile float kiw = 0.05f; // Ganho Integral

// Ganhos do PI de Corrente (Eixo Q)
volatile float kpq = 0.1f;
volatile float kiq = 0.05f;

// Ganhos do PI de Corrente (Eixo D)
volatile float kpd = 0.1f;
volatile float kid = 0.05f;

float L_GAIN_1 = 20;
float L_GAIN_2 = 10;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_ADC1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
  	TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 900;

    HAL_TIM_Base_Start(&htim1);


    // 4. PREPARAR O ADC
     HAL_OPAMP_Start(&hopamp1);
     HAL_Delay(100);
     HAL_OPAMP_Start(&hopamp2);
     HAL_Delay(100);

     // 5. CALIBRAR O ADC (DEVE VIR ANTES DO START_DMA)
     HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
     HAL_Delay(100);
     HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
     HAL_Delay(100);

     HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);


     for (int calibrando= 0; calibrando < (1<<4); calibrando++){

    	 while(flagdadosprontos==0){}

   		  offset_iu_raw_op1= offset_iu_raw_op1 + iu_raw_op1;
   		  offset_iu_raw_op2= offset_iu_raw_op2 + iu_raw_op2;
   		  offset_iv_raw= offset_iv_raw + iv_raw;
   		  offset_iw_raw= offset_iw_raw + iw_raw;

   		  flagdadosprontos=0;

	}

     offset_iu_raw_op1= offset_iu_raw_op1/(1<<4);
     offset_iu_raw_op2= offset_iu_raw_op2/(1<<4);
     offset_iv_raw= offset_iv_raw/(1<<4);
     offset_iw_raw= offset_iw_raw/(1<<4);


     HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
     HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

	 int init = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 if(flagdadosprontos==1){

	__disable_irq();


	flagdadosprontos=0;

	iu =  ((float)iu_raw_op1+iu_raw_op2-offset_iu_raw_op2-offset_iu_raw_op1)/2 * (3.3f / 4095.0f) / 0.02f;
	iv =  ((float)iv_raw - offset_iv_raw) * (3.3f / 4095.0f) / 0.02f;
	iw =  ((float)iw_raw- offset_iw_raw) * (3.3f / 4095.0f) / 0.02f;
	vu =  (((float)vu_raw)* (3.3f / 4095.0f)) * 21.6f;
	vv =  (((float)vv_raw)* (3.3f / 4095.0f)) * 21.6f;
	vw =  (((float)vw_raw)* (3.3f / 4095.0f)) * 21.6f;
	vbus =(((float)vbus_raw)* (3.3f / 4095.0f)) * 21.6f;
	 __enable_irq();

	 //  Medidor de contra-eletromotriz
	 // --- ETAPA 1: TRANSFORMAÇÃO DE CORRENTE (BRUTA) ---
	 	 		        		 	ia = (TWO_DIV_3 * iu) - (TWO_DIV_3/2.0f * iv) - (TWO_DIV_3/2.0f * iw);
	 	 		        		 	ib = SQRT3_DIV_3 * (iv - iw);

	 	 		        		 	// Filtro para os PIs do FOC (o FOC gosta de corrente limpa)
	 	 		        		 	ia_filtrada = (CURRENT_FILTER_ALPHA * ia) + (1.0f - CURRENT_FILTER_ALPHA) * ia_filtrada;
	 	 		        		 	ib_filtrada = (CURRENT_FILTER_ALPHA * ib) + (1.0f - CURRENT_FILTER_ALPHA) * ib_filtrada;

	 	 		        		 	// --- ETAPA 2: SLIDING MODE OBSERVER (SMO) ---
	 	 		        		 	// O SMO estima o BEMF. Ele usa as correntes *brutas* (ia, ib)
	 	 		        		 	// e as tensões comandadas (va, vb) do *ciclo anterior*.

	 	 		        		 	// Calcule o erro de corrente (Real vs. Modelo)
	 	 		        		 	float ia_err = smo_ia_est - ia;
	 	 		        		 	float ib_err = smo_ib_est - ib;

	 	 		        		 	// Função de "sliding" (saturação) para evitar "chattering"
	 	 		        		 	float sat_limit = 0.4f; // Sintonizável
	 	 		        		 	float slide_a = (ia_err > sat_limit) ? sat_limit : ((ia_err < -sat_limit) ? -sat_limit : ia_err);
	 	 		        		 	float slide_b = (ib_err > sat_limit) ? sat_limit : ((ib_err < -sat_limit) ? -sat_limit : ib_err);

	 	 		        		 	// Modelo do motor: di_est/dt = (1/L) * (V - R*i_est - Z)
	 	 		        		 	// Onde Z (smo_z_alpha) é o BEMF estimado.
	 	 		        		 	float L_inv = 1.0f / L_PHASE; // 1.0 / 0.2 = 5.0

	 	 		        		 	smo_ia_est = smo_ia_est + Ts * ( L_inv * (va - R_PHASE*smo_ia_est - smo_z_alpha) );
	 	 		        		 	smo_ib_est = smo_ib_est + Ts * ( L_inv * (vb - R_PHASE*smo_ib_est - smo_z_beta) );

	 	 		        		 	// O SMO "força" o BEMF estimado (Z) a corrigir o erro de corrente
	 	 		        		 	// dZ/dt = G * slide_signal
	 	 		        		 	smo_z_alpha = smo_z_alpha + Ts * smo_gain * slide_a;
	 	 		        		 	smo_z_beta = smo_z_beta + Ts * smo_gain * slide_b;

	 	 		        		 	// 'smo_z_alpha' e 'smo_z_beta' agora contêm o BEMF limpo e estimado

	 	 		        		 	// --- ETAPA 3: OBSERVADOR PLL (Trava no BEMF do SMO) ---
	 	 		        		 	float32_t sin_pll, cos_pll;
	 	 		        		 	sin_pll = arm_sin_f32(pll_estimated_theta);
	 	 		        		 	cos_pll = arm_cos_f32(pll_estimated_theta);

	 	 		        		 	// Erro = BEMF do SMO projetado no ângulo do PLL
	 	 		        		 	float e_d_error = -smo_z_alpha * sin_pll + smo_z_beta * cos_pll;

	 	 		        		 	// PI do PLL
	 	 		        		 	pll_angle_error_integral = pll_angle_error_integral + e_d_error * Ts;
	 	 		        		 	pll_estimated_w = (pll_kp * e_d_error) + (pll_ki * pll_angle_error_integral);

	 	 		        		 	// Integrador do PLL
	 	 		        		 	pll_estimated_theta = pll_estimated_theta + pll_estimated_w * Ts;

	 	 		        		 	if (pll_estimated_theta > TWO_PI) pll_estimated_theta -= TWO_PI;
	 	 		        		 	else if (pll_estimated_theta < 0.0f) pll_estimated_theta += TWO_PI;

	 	 		        		 	// --- FIM DO OBSERVADOR ---


	 	 // --- INÍCIO DA MÁQUINA DE ESTADOS ---

	 	 if(vbus < 12)
	 	     {
	 	         // ESTADO 1: FALHA
	 	         TIM1->CCR1 = 0;
	 	         TIM1->CCR2 = 0;
	 	         TIM1->CCR3 = 0;
	 	         init = 1;
	 	         theta = 0;
	 	         w = 0;
	 	         serroid=0;
	 	         serroiq=0;
	 	         thetach=0;
	 	         thetavelho=0;
	 	         serrow=0;
	 	         observador=0;
	 	     }
	 	 else if(vbus > 12 && init == 1)
	 	      {
	 	     	 init = 0;
	 	          // ESTADO 2: ALINHAMENTO
	 	          HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
	 	          va = 0.1*vbus;
	 	          vb = 0.0f;
	 	          vu = va;
	 	          vv = (-0.5f * va);
	 	          vw = (-0.5f * va);

	 	          // Use 800, como o resto do seu código
	 	          TIM1->CCR1 = (800 / 2) * (1.0f + (vu / vbus));
	 	          TIM1->CCR2 = (800 / 2) * (1.0f + (vv / vbus));
	 	          TIM1->CCR3 = (800 / 2) * (1.0f + (vw / vbus));

	 	          HAL_Delay(2000);

	 	          // --- ZERAR OS ESTADOS DEPOIS DO DELAY ---
	 	          ia_prev = 0.0f;     // (Não é mais usado pelo observador, mas limpa)
	 	          ib_prev = 0.0f;
	 	          ia_filtrada = 0.0f;
	 	          ib_filtrada = 0.0f;
	 	          ea_filtrada = 0.0f; // (Não é mais usado)
	 	          eb_filtrada = 0.0f; // (Não é mais usado)

	 	          // Zera os estados do NOVO PLL
	 	          pll_angle_error_integral = 0.0f;
	 	          pll_estimated_w = 0.0f;
	 	          pll_estimated_theta = 0.0f; // Alinhado com o ângulo 0

	 	          // Zera os estados do NOVO SMO
	 	          smo_z_alpha = 0.0f;
	 	          smo_z_beta = 0.0f;
	 	          smo_ia_est = 0.0f;
	 	          smo_ib_est = 0.0f;

	 	          // Zera os estados do FOC
	 	          serrow = 0.0f;
	 	          serroiq = 0.0f;
	 	          serroid = 0.0f;

	 	          theta = 0.0f;
	 	          w = 0.0f;
	 	          dtheta = 0.0f;
	 	           // Mude para o ESTADO 3
	 	          HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);

	 	     }
	 	     else if(vbus > 12 && init != 1 && observador!=2)
	 	     {
	 	         // ESTADO 3: RAMPA

	 	    	if(w > 440)
	 	    		         {
	 	    		        	 observador=2;
	 	    		        	 serrow = 0.0f;
	 	    		        	 serroiq = 0.0f;
	 	    		        	 serroid = 0.0f;
	 	    		         }

	 	    		         dtheta = w*Ts;
	 	    		         theta = theta + dtheta;

	 	    		         if (theta > TWO_PI)
	 	    		         {
	 	    		             theta = theta - TWO_PI;
	 	    		         }

	 	    		         float32_t sin_theta, cos_theta;
	 	    		         sin_theta = arm_sin_f32(theta);
	 	    		         cos_theta = arm_cos_f32(theta);


	 	    		         arm_inv_park_f32(vd, (vbus/3 + 2*vbus*w/(3*450))*0.5, &va, &vb, sin_theta, cos_theta);
	 	         vu = va;
	 	         vv = (-0.5 * va) + (SQRT3_DIV_2 * vb);
	 	         vw = (-0.5 * va) - (SQRT3_DIV_2 * vb);

	 	         TIM1->CCR1 = (800 / 2) * (1.0f + (vu / vbus));
	 	         TIM1->CCR2 = (800 / 2) * (1.0f + (vv / vbus));
	 	         TIM1->CCR3 = (800 / 2) * (1.0f + (vw / vbus));
	 	        // O DAC agora mostra o ângulo do PLL
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095*pll_estimated_theta/TWO_PI );
				// O DAC 2 mostra a corrente *filtrada*
				uint32_t dac_corrente = (uint32_t)( 4095*theta/TWO_PI );
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_corrente );

	 	     }

	 	     // ESTADO 4: CONTROLO FOC "SENSORLESS" (MALHA FECHADA)
	 	     else if(vbus > 12 && init != 1 && observador!=0){ // (observador!=0 cobre o estado 2)

	 	    	 	 	 	 	 float32_t sin_theta, cos_theta;

	 	    	                       // Use o ângulo limpo e compensado do PLL
	 	    	 	    	          sin_theta = arm_sin_f32(pll_estimated_theta);
	 	    	 	    	          cos_theta = arm_cos_f32(pll_estimated_theta);

	 	    	 	    	          // Transformada de Park (Mede Id, Iq reais)
	 	    	 	    	          // (Use a corrente filtrada para o FOC)
	 	    	 	    	          arm_park_f32(ia_filtrada, ib_filtrada, &id, &iq, sin_theta, cos_theta);

	 	    	 	    	          // BÓNUS: Use a velocidade estimada pelo PLL
	 	    	                       // É muito mais limpa e não tem ruído de 'dtheta/dt'
	 	    	 	    	          w = pll_estimated_w;


	 	    	          // --- CONTROLO PI DE VELOCIDADE (CASCATA EXTERNA) ---

	 	    	          errow = wref - w; // 'w' agora é o valor correto do PLL
	 	    	          serrow = serrow + errow*Ts;

	 	    	          // Anti-Windup (para o limite de corrente 'iqref' de 3.0)
	 	    	          float SERROW_MAX_VAL = 3.0f / kiw; // kiw não pode ser zero
	 	    	          float SERROW_MIN_VAL = -3.0f / kiw;
	 	    	          if (serrow > SERROW_MAX_VAL) serrow = SERROW_MAX_VAL;
	 	    	          else if (serrow < SERROW_MIN_VAL) serrow = SERROW_MIN_VAL;

	 	    	          iqref = kpw*errow + kiw*serrow;
	 	    	          // Saturação da corrente de referência (Já estava correto)
	 	    	          if (iqref < -3.0f) { iqref = -3.0f; } else if (iqref > 3.0f) { iqref = 3.0f; }


	 	    	          // --- DEFINIÇÃO DO LIMITE DE TENSÃO DINÂMICO ---
	 	    	          // v_limite = Vbus * (1/sqrt(3))
	 	    	          float v_limite = vbus * SQRT3_DIV_3; // SQRT3_DIV_3 está definido como 0.577...


	 	    	          // --- CONTROLO PI DE CORRENTE Q (CASCATA INTERNA) ---
	 	    	          erroiq = iqref - iq;
	 	    	          serroiq = serroiq + erroiq*Ts;

	 	    	          // Anti-Windup (IMPLEMENTAÇÃO DINÂMICA)
	 	    	          float SERRQ_MAX_VAL = v_limite / kiq; // kiq não pode ser zero
	 	    	          float SERRQ_MIN_VAL = -v_limite / kiq;
	 	    	          if (serroiq > SERRQ_MAX_VAL) serroiq = SERRQ_MAX_VAL;
	 	    	          else if (serroiq < SERRQ_MIN_VAL) serroiq = SERRQ_MIN_VAL;

	 	    	          vq = kpq*erroiq + kiq*serroiq;
	 	    	          // Saturação da Tensão de Saída
	 	    	          if (vq > v_limite) vq = v_limite;
	 	    	          else if (vq < -v_limite) vq = -v_limite;


	 	    	          // --- CONTROLO PI DE CORRENTE D (CASCATA INTERNA) ---
	 	    	          erroid = 0.0f - id; // Referência de Id é 0
	 	    	          serroid = serroid + erroid*Ts;

	 	    	          // Anti-Windup (IMPLEMENTAÇÃO DINÂMICA E CORREÇÃO DO BUG C&P)
	 	    	          float SERRD_MAX_VAL = v_limite / kid; // kid não pode ser zero
	 	    	          float SERRD_MIN_VAL = -v_limite / kid;

	 	    	          // AQUI ESTAVA O SEU BUG: usava 'serroiq' em vez de 'serroid'
	 	    	          if (serroid > SERRD_MAX_VAL) serroid = SERRD_MAX_VAL;
	 	    	          else if (serroid < SERRD_MIN_VAL) serroid = SERRD_MIN_VAL;

	 	    	          vd = kpd*erroid + kid*serroid;
	 	    	          // Saturação da Tensão de Saída
	 	    	          if (vd > v_limite) vd = v_limite;
	 	    	          else if (vd < -v_limite) vd = -v_limite;


	 	    	          // --- APLICAÇÃO (INVERSA DE PARK & PWM) ---
	 	    	          // (Usa o sin_theta/cos_theta do PLL calculado no início deste bloco)
	 	    	          arm_inv_park_f32(vd, vq, &va, &vb, sin_theta, cos_theta);

	 	    	          vu = va;
	 	    	          vv = (-0.5 * va) + (SQRT3_DIV_2 * vb);
	 	    	          vw = (-0.5 * va) - (SQRT3_DIV_2 * vb);

	 	    	          TIM1->CCR1 = (800 / 2) * (1.0f + (vu / vbus));
	 	    	          TIM1->CCR2 = (800 / 2) * (1.0f + (vv / vbus));
	 	    	          TIM1->CCR3 = (800 / 2) * (1.0f + (vw / vbus));

	 	    	         // O DAC agora mostra o ângulo do PLL
	 	    	          HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095*pll_estimated_theta/TWO_PI );
							// O DAC 2 mostra a corrente *filtrada*
							uint32_t dac_corrente = (uint32_t)( 4095*theta/TWO_PI );
							HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_corrente );


	     	 }

	     }
	 }




//--------------------------------------------------------------------------------------------------
/*		ESSA ÁREA PODE SER IGNORADA AGORA, VAI SER ADIANTE PARA IMPLEMENTAR UM OBSERVADOR DE LUENBERGER
			 erroia = ia - iach;

			 diach = (1.0f / L_PHASE) * (va - R_PHASE * iach - each) + (L_GAIN_1 * erroia);
			 deach = L_GAIN_2 * erroia;

			 erroib = ib - ibch;
			 dibch = (1.0f / L_PHASE) * (vb - R_PHASE * ibch - ebch) + (L_GAIN_1 * erroib);
			 debch = L_GAIN_2 * erroib;

			 iach = iach + (diach * Ts);
			 ibch = ibch + (dibch * Ts);
			 each = each + (deach * Ts);
			 ebch = ebch + (debch * Ts);

			 thetach = atan2f(ebch, each);
*/





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 4;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InternalOutput = ENABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_2_OR_MINUS_1;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;
  hopamp2.Init.InternalOutput = ENABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_2_OR_MINUS_1;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 20;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB10 PB3 PB4 PB5
                           PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* A conversão do ADC1 terminou? */
  if (hadc->Instance == hadc1.Instance)
  {
    if(opamp1selector == 0)
    {
      // 1. Mude o MUX para a *próxima* leitura (PA7)
      MODIFY_REG(hopamp1.Instance->CSR, OPAMP_CSR_VPSEL, OPAMP_NONINVERTINGINPUT_IO2);

      // 2. Salve os dados da leitura que *acabou de terminar* (de PA1)
      iw_raw = adc1_buffer[0];
      vu_raw = adc1_buffer[1];

      // 3. Atualize o estado
      opamp1selector = 1;
    }
    else
    {
      // 1. Mude o MUX de volta para a *próxima* leitura (PA1)
      MODIFY_REG(hopamp1.Instance->CSR, OPAMP_CSR_VPSEL, OPAMP_NONINVERTINGINPUT_IO0);

      // 2. Salve os dados da leitura que *acabou de terminar* (de PA7)
      iu_raw_op1 = adc1_buffer[0];
      vu_raw = adc1_buffer[1]; // Você está lendo vu_raw nos dois casos, ok

      // 3. Atualize o estado
      opamp1selector = 0;
    }

    // 4. Inicie a próxima etapa da cadeia: ADC2
    HAL_ADC_Start_DMA(&hadc2, adc2_buffer, 4);
  }

  /* A conversão do ADC2 terminou? */
  if (hadc->Instance == hadc2.Instance)
  {
    if(opamp2selector == 0)
    {
      // 1. Mude o MUX para a *próxima* leitura (PB0)
      MODIFY_REG(hopamp2.Instance->CSR, OPAMP_CSR_VPSEL, OPAMP_NONINVERTINGINPUT_IO2);

      // 2. Salve os dados da leitura que *acabou de terminar* (de PB14)
      iv_raw = adc2_buffer[0];
      vv_raw = adc2_buffer[1];
      vw_raw = adc2_buffer[2];
      vbus_raw = adc2_buffer[3];

      // 3. Atualize o estado
      opamp2selector = 1;
    }
    else
    {
      // 1. Mude o MUX de volta para a *próxima* leitura (PB14)
      MODIFY_REG(hopamp2.Instance->CSR, OPAMP_CSR_VPSEL, OPAMP_NONINVERTINGINPUT_IO1);

      // 2. Salve os dados da leitura que *acabou de terminar* (de PB0)
      iu_raw_op2 = adc2_buffer[0];
      vv_raw = adc2_buffer[1]; // Cuidado, sobrescrevendo vv_raw
      vw_raw = adc2_buffer[2]; // Cuidado, sobrescrevendo vw_raw
      vbus_raw = adc2_buffer[3]; // Cuidado, sobrescrevendo vbus_raw

      // 3. Atualize o estado
      opamp2selector = 0;

      // 4. FIM DA CADEIA! Avise o main loop e resete o timer.
      flagdadosprontos = 1;
      // Sua nova lógica
    }
itcounter=0;
  }

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
