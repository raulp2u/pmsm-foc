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
#include "stm32g4xx_ll_dma.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    // Coeficientes (A Receita do Filtro)
    float b0, b1, b2; // Pesos da Entrada
    float a1, a2;     // Pesos da Saída (Feedback)

    // Memória (O Histórico)
    float x1, x2; // Entradas passadas (Ontem, Anteontem)
    float y1, y2; // Saídas passadas (Ontem, Anteontem)
} EstruturaFiltro;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SQRT3_DIV_3 0.57735027f  // 	1 / sqrt(3)
#define SQRT3_DIV_2 0.86602540f  // 	sqrt(3) / 2
#define TWO_DIV_3   0.66666667f  // 	2 / 3
#define PI 3.14159265f
#define TWO_PI 6.2831853f

#define SERROW_MAX  10.0f
#define SERROW_MIN -10.0f
#define SERRQ_MAX   10.0f
#define SERRQ_MIN  -10.0f
#define SERRD_MAX   10.0f
#define SERRD_MIN  -10.0f

#define R 0.138f
#define L 0.0001f
#define Fs 10000.0f
#define Ts (1 / Fs)

#define CCM_VAR  __attribute__((section(".ccmram")))
#define CCM_FUNC __attribute__((section(".ccmram.text")))

// --- Ganhos do PLL (Observador de Ângulo) ---

// KP: "Rigidez" do rastreamento. Define quão rápido o PLL reage a um erro de ângulo.
// Valor muito baixo: Motor perde sincronismo se acelerar rápido.
// Valor muito alto: O ângulo oscila e faz barulho.
#define PLL_KP  150.0f

// KI: "Memória" da velocidade. Ajuda a travar na velocidade correta.
// Geralmente segue a relação de amortecimento crítico: Ki = (Kp^2) / 4
// Para Kp=250 -> Ki ~= 15625. Mas em FOC costuma-se usar um pouco mais alto.
#define PLL_KI  60000.0f

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
uint16_t adc1_buffer[1];
uint16_t adc2_buffer[2];

volatile int flagdadosprontos=0;

volatile uint16_t offset_iu_raw_op2=0;
volatile uint16_t offset_iv_raw=0;
volatile uint16_t offset_iw_raw=0;
volatile uint16_t offset_iu_raw_op1=0;

volatile int observador = 0;
// Structs dos Filtros (COLOQUE NA CCMRAM)
CCM_VAR EstruturaFiltro filtro_iv;
CCM_VAR EstruturaFiltro filtro_iw;

// Variáveis de Cálculo (Pode colocar na CCMRAM também)
CCM_VAR float iv, iw, iu, vbus;


volatile float vu=0;
volatile float vv=0;
volatile float vw=0;


volatile uint16_t iu_raw;
volatile uint16_t iu_raw_op1;
volatile uint16_t iu_raw_op2;
volatile uint16_t iv_raw;
volatile uint16_t iw_raw;
volatile float iv_raw_filtrada;
volatile float iw_raw_filtrada;
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

// Ganhos do Filtro (Ajuste Fino)
// Sugestão inicial: Alpha entre 0.05 e 0.2.
float ALPHA = 0.01f;
// Beta geralmente segue a relação: Beta ~= Alpha^2 / (2 - Alpha)
// Para Alpha = 0.1, Beta ~= 0.005
float BETA = 0.0001f;

//ganhos chutados

volatile float kpw = 0.1f; // Ganho Proporcional
volatile float kiw = 0.05f; // Ganho Integral

// Ganhos do PI de Corrente (Eixo Q)
volatile float kpq = 0.1f;
volatile float kiq = 0.05f;

// Ganhos do PI de Corrente (Eixo D)
volatile float kpd = 0.1f;
volatile float kid = 0.05f;


// Estados do Filtro Alpha-Beta
float i_alpha_est = 0.0f, di_alpha_est = 0.0f;
float i_beta_est = 0.0f,  di_beta_est  = 0.0f;

// Estados do PLL
float pll_theta = 0.0f;
float pll_omega = 0.0f;
float pll_integrator = 0.0f;

// Saídas Finais para o FOC
float theta_foc = 0.0f;    // Ângulo para usar em Park/InvPark
float rpm_foc = 0.0f;      // Velocidade para controle

// Inputs e Outputs do ciclo anterior (Necessário para BEMF)
float valpha_prev = 0.0f;
float vbeta_prev = 0.0f;

 float32_t sin_foc, cos_foc;


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
CCM_FUNC void Fast_Loop(void);
 void Inicializar_Filtros(void);
CCM_FUNC float Processar_Filtro(EstruturaFiltro *f, float entrada);

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



  HAL_TIM_Base_Start(&htim1);

  // 1. PREPARAR OS OPAMPS
  // (Têm um pequeno tempo de inicialização, T_SU_OPAMP)
  if (HAL_OPAMP_Start(&hopamp1) != HAL_OK)
  {
      Error_Handler();
  }
  if (HAL_OPAMP_Start(&hopamp2) != HAL_OK)
  {
      Error_Handler();
  }
  // Espere 1ms. T_SU_OPAMP é geralmente < 100us, então 1ms é muito seguro.
  HAL_Delay(1);


  // 2. CALIBRAR OS ADCS
  // Esta função é BLOQUEANTE (já espera a calibração terminar).
  // O HAL_Delay(100) original era desnecessário.
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
      Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  {
      Error_Handler();
  }
   HAL_ADC_Start_DMA(&hadc1, adc1_buffer, 1);
   HAL_ADC_Start_DMA(&hadc2, adc2_buffer, 2);

   TIM1->CCR4=(TIM1->ARR)*0.95;

   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

   // 6. LOOP DE CALIBRAÇÃO DE OFFSET (MAIS ROBUSTO)

   // Ajuste de Amostras:
   // 4096 a 10kHz = ~0.4 segundos (Já é EXCELENTE para média)
   // 8192 a 10kHz = ~0.8 segundos (Extremamente estável)
   #define OFFSET_CAL_SAMPLES 8192

   // Timeout:
   // Deve ser maior que o tempo esperado.
   // 8192 * 0.0001s = 819ms. Vamos dar 2000ms de folga.
   #define OFFSET_CAL_TIMEOUT_MS 2000

   uint32_t cal_start_tick;

   // Acumuladores de 64 bits (Essencial para não estourar)
   uint64_t temp_offset_iv = 0;
   uint64_t temp_offset_iw = 0;

   // Garante que o flag comece zerado
   flagdadosprontos = 0;

   // Pega o tempo inicial
   cal_start_tick = HAL_GetTick();

   for (int i = 0; i < OFFSET_CAL_SAMPLES; i++)
   {
       // Espera a interrupção do ADC setar o flag (ex: virar 1 ou 2)
       // Nota: Adicionei 'volatile' na declaração da flag lá no topo do seu código? É importante.
       while (flagdadosprontos == 0)
       {
           // Verifica se o tempo TOTAL excedeu o limite
           if ((HAL_GetTick() - cal_start_tick) > OFFSET_CAL_TIMEOUT_MS)
           {
               // Se entrou aqui: O ADC parou ou a amostragem é mais lenta do que pensávamos.
               Error_Handler();
           }
       }

       // Limpa o flag para esperar a próxima
       // (Se sua ISR incrementa até 2, talvez seja melhor: flagdadosprontos--)
       flagdadosprontos = 0;

       // Acumula
       temp_offset_iv += iv_raw;
       temp_offset_iw += iw_raw;

       // Opcional: Se você usar Watchdog (IWDG), deve dar o "kick" aqui dentro
       // HAL_IWDG_Refresh(&hiwdg);
   }

   // Calcula a média final
   offset_iv_raw = (uint32_t)(temp_offset_iv / OFFSET_CAL_SAMPLES);
   offset_iw_raw = (uint32_t)(temp_offset_iw / OFFSET_CAL_SAMPLES);

   // Opcional: Resetar as variáveis filtradas iniciais para evitar "pulo" no filtro Alpha-Beta
   iv = 0.0f; iw = 0.0f; iu = 0.0f;


// 7. INICIAR DACS (para debug ou setpoints)
if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
{
    Error_Handler();
}
if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK)
{
    Error_Handler();
}

	 int init = 1;


	 Inicializar_Filtros();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 while (1)
	 {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		 if(flagdadosprontos>=2){

			Fast_Loop();

			 // 2. CLARKE TRANSFORM (Obtemos Ia e Ib no estator)
			 ia = (TWO_DIV_3 * iu) - (TWO_DIV_3/2.0f * iv) - (TWO_DIV_3/2.0f * iw);
			 ib = SQRT3_DIV_3 * (iv - iw);


			 // =========================================================
			 // BLOCO DO OBSERVADOR (RODA SEMPRE, INDEPENDENTE DO ESTADO)
			 // =========================================================

			 // --- PASSO 1: FILTRO ALPHA-BETA ---
			 // Eixo Alpha
			 float ia_pred = i_alpha_est + (di_alpha_est * Ts);
			 float res_a   = ia - ia_pred;
			 i_alpha_est   = ia_pred + (ALPHA * res_a);
			 di_alpha_est  = di_alpha_est + ((BETA/Ts) * res_a);

			 // Eixo Beta
			 float ib_pred = i_beta_est + (di_beta_est * Ts);
			 float res_b   = ib - ib_pred;
			 i_beta_est    = ib_pred + (ALPHA * res_b);
			 di_beta_est   = di_beta_est + ((BETA/Ts) * res_b);

			 // --- PASSO 2: ESTIMADOR DE BEMF ---

			 float e_alpha = valpha_prev - (R * i_alpha_est) - (L * di_alpha_est);
			 float e_beta  = vbeta_prev  - (R * i_beta_est)  - (L * di_beta_est);

			 // --- PASSO 3: PLL (Rastreamento de Ângulo) ---
			 float32_t sin_pll, cos_pll;
			 sin_pll = arm_sin_f32(pll_theta);
			 cos_pll = arm_cos_f32(pll_theta);


			 float error_pll = (e_alpha * cos_pll) + (e_beta * sin_pll);

			 pll_integrator += error_pll * PLL_KI * Ts;
			 pll_omega = (error_pll * PLL_KP) + pll_integrator;

			 pll_theta += pll_omega * Ts;

			 // Normalização Theta PLL
			 if (pll_theta > (2.0f * PI)) pll_theta -= (2.0f * PI);
			 else if (pll_theta < 0.0f)   pll_theta += (2.0f * PI);

			 // --- PASSO 4: SAÍDA DO FOC (Theta Compensado) ---
			 float theta_foc = pll_theta + (pll_omega * Ts * 1.5f);

			 // Normalização Theta FOC
			 if (theta_foc > (2.0f * PI)) theta_foc -= (2.0f * PI);
			 else if (theta_foc < 0.0f)   theta_foc += (2.0f * PI);


			 // =========================================================
			 // MÁQUINA DE ESTADOS
			 // =========================================================

			 if(vbus < 12) // ESTADO 1: FALHA / BAIXA TENSÃO
			 {
				 TIM1->CCR1 = ((TIM1->ARR)/2);
				 TIM1->CCR2 = ((TIM1->ARR)/2);
				 TIM1->CCR3 = ((TIM1->ARR)/2);

				 init = 1;
				 // Zera variáveis do observador para evitar loucura na volta
				 pll_integrator = 0; pll_theta = 0; pll_omega = 0;
				 w=0;
			 }
			 else if(vbus > 12 && init == 1) // ESTADO 2: ALINHAMENTO
			 {
				 init = 0;
				 va = 0.1f * vbus; vb = 0.0f; // Vetor fixo
				 // ... (Cálculo do PWM mantido) ...
				 vu = va;
				 vv = (-0.5f * va);
				 vw = (-0.5f * va);
				 // Atualiza PWM...
				 TIM1->CCR1 = ((TIM1->ARR)*0.8f / 2) * (1.0f + (vu / vbus));
				 TIM1->CCR2 = ((TIM1->ARR)*0.8f / 2) * (1.0f + (vv / vbus));
				 TIM1->CCR3 = ((TIM1->ARR)*0.8f / 2) * (1.0f + (vw / vbus));


				 HAL_Delay(2000);
				 // Define ângulo inicial do PLL igual ao alinhamento (0 graus)
				 pll_theta = 0;
			 }

			 //--------------------------------------
			 // ----ESTADO 3: RAMPA (MALHA ABERTA)---

			 else if(vbus > 12 && init != 1 && observador!=2)
			 {
				 // 1. Aceleração
				 if (w < 450.0f) { w = w + 1*Ts; }
				 wref = w;

				 // 2. Integração do Ângulo
				 dtheta = w*Ts;
				 theta = theta + dtheta;

				 if (theta > TWO_PI){ theta = theta - TWO_PI;}


				 // 5. Transição para Sensorless
				 if(w > 440.0f)
				 {
					 observador=2;
					 serrow = 0.0f; serroiq = 0.0f; serroid = 0.0f;

					 // Sincroniza o PLL
					 pll_integrator = w;
					 // Opcional: forçar o ângulo do PLL ser igual ao da rampa para evitar tranco
					 pll_theta = theta;
				 }


				 // 4. Geração dos Vetores (USANDO INVERSA DE PARK COMO ANTES)
				 float32_t sin_ramp, cos_ramp;

				 sin_ramp = arm_sin_f32(theta);
				 cos_ramp = arm_cos_f32(theta);
				 arm_inv_park_f32(0, (2*0.7*vbus/8 + 6*vbus*w/(8*450)), &va, &vb, sin_ramp, cos_ramp);

				 // 6. Atualiza PWM
				 vu = va;
				 vv = (-0.5f * va) + (SQRT3_DIV_2 * vb);
				 vw = (-0.5f * va) - (SQRT3_DIV_2 * vb);
				 TIM1->CCR1 = ((TIM1->ARR)*0.8f / 2) * (1.0f + (vu / vbus));
				 TIM1->CCR2 = ((TIM1->ARR)*0.8f / 2) * (1.0f + (vv / vbus));
				 TIM1->CCR3 = ((TIM1->ARR)*0.8f / 2) * (1.0f + (vw / vbus));

			 }

			 //--------------------------------------------------------
			 // ESTADO 4: CONTROLO FOC "SENSORLESS" (MALHA FECHADA)----
			 //--------------------------------------------------------

			 else if(vbus > 12 && init != 1 && observador!=0) // (observador==2)
			 {
				 // 1. FEEDBACK DE VELOCIDADE (Vem do PLL)
				 w = pll_omega;

				 // 2. PARK TRANSFORM (Calcula Id e Iq atuais)
				 float32_t sin_foc, cos_foc;

				 sin_foc = arm_sin_f32(theta_foc);
				 cos_foc = arm_cos_f32(theta_foc);


				 id = (i_alpha_est * cos_foc) + (i_beta_est * sin_foc);
				 iq = (i_beta_est * cos_foc) - (i_alpha_est * sin_foc);


				 // --- 3. CONTROLADOR DE VELOCIDADE ---
				 errow = wref - w;
				 serrow = serrow + errow*Ts;

				 // Anti-Windup Velocidade (Seu código original)

				 if(kiw > 0.00001f) {
					 float SERROW_MAX_VAL = 3.0f / kiw;
					 float SERROW_MIN_VAL = -3.0f / kiw;
					 if (serrow > SERROW_MAX_VAL) serrow = SERROW_MAX_VAL;
					 else if (serrow < SERROW_MIN_VAL) serrow = SERROW_MIN_VAL;
				 }

				 iqref = kpw*errow + kiw*serrow;

				 // Saturação da Corrente de Referência (Limite de 3.0A)
				 if (iqref < -3.0f) iqref = -3.0f;
				 else if (iqref > 3.0f) iqref = 3.0f;


				 // --- 4. CÁLCULO DO LIMITE DE TENSÃO (SVPWM) ---
				 float v_limite = vbus * SQRT3_DIV_3; // Max tensão disponível no vetor


				 // --- 5. CONTROLADOR DE CORRENTE Q ---
				 erroiq = iqref - iq;
				 serroiq = serroiq + erroiq*Ts;

				 // Anti-Windup IQ (Dinâmico baseada no Vbus)
				 if(kiq > 0.00001f) {
					 float SERRQ_MAX_VAL = v_limite / kiq;
					 float SERRQ_MIN_VAL = -v_limite / kiq;
					 if (serroiq > SERRQ_MAX_VAL) serroiq = SERRQ_MAX_VAL;
					 else if (serroiq < SERRQ_MIN_VAL) serroiq = SERRQ_MIN_VAL;
				 }

				 vq = kpq*erroiq + kiq*serroiq;

				 // Saturação da Tensão Q
				 if (vq > v_limite) vq = v_limite;
				 else if (vq < -v_limite) vq = -v_limite;

				 // --- 6. CONTROLADOR DE CORRENTE D ---
				 erroid = 0.0f - id; // IdRef é 0
				 serroid = serroid + erroid*Ts;

				 // Anti-Windup ID
				 if(kid > 0.00001f) {
					 float SERRD_MAX_VAL = v_limite / kid;
					 float SERRD_MIN_VAL = -v_limite / kid;
					 if (serroid > SERRD_MAX_VAL) serroid = SERRD_MAX_VAL;
					 else if (serroid < SERRD_MIN_VAL) serroid = SERRD_MIN_VAL;
				 }

				 vd = kpd*erroid + kid*serroid;

				 // Saturação da Tensão D
				 if (vd > v_limite) vd = v_limite;
				 else if (vd < -v_limite) vd = -v_limite;


				 // --- 7. INVERSE PARK TRANSFORM ---
				 arm_inv_park_f32(vd, vq, &va, &vb, sin_foc, cos_foc);

				 // --- 8. ATUALIZAÇÃO DO PWM (SVPWM) ---
				 vu = va;
				 vv = (-0.5f * va) + (SQRT3_DIV_2 * vb);
				 vw = (-0.5f * va) - (SQRT3_DIV_2 * vb);

				 TIM1->CCR1 = ((TIM1->ARR)*0.8f / 2) * (1.0f + (vu / vbus));
				 TIM1->CCR2 = ((TIM1->ARR)*0.8f / 2) * (1.0f + (vv / vbus));
				 TIM1->CCR3 = ((TIM1->ARR)*0.8f / 2) * (1.0f + (vw / vbus));




			 } // fim sensorless

			 // =========================================================
			 // DEBUG DAC (VISUALIZAÇÃO EM TEMPO REAL)
			 // =========================================================

			 // --- CANAL 1: THETA ESTIMADO (0 a 3.3V = 0 a 360 graus) ---
			 // Normaliza de 0..2PI para 0..4095
			 uint32_t dac_theta_val = (uint32_t)( (theta_foc / (2.0f * PI)) * 4095.0f );
			 // Proteção simples (caso o theta passe um pouquinho de 2PI por erro numérico)
			 if (dac_theta_val > 4095) dac_theta_val = 4095;
			 HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_theta_val);
			 // --- CANAL 2: CORRENTE ALPHA ESTIMADA (Senoide Centralizada) ---
			 // O DAC vai oscilar entre 1248 e 2848.
			 float DAC_GAIN_I = 200.0f;
			 int32_t dac_current_val = 2048 + (int32_t)(i_alpha_est * DAC_GAIN_I);
			 // Saturação para não estourar o DAC (0 a 4095)
			 if (dac_current_val > 4095) dac_current_val = 4095;
			 else if (dac_current_val < 0) dac_current_val = 0;
			 HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)dac_current_val);
			 // =========================================================
			 // O estimador de BEMF precisa saber que tensão foi aplicada NESTE ciclo
			 // para calcular a BEMF no PRÓXIMO ciclo.

			 valpha_prev = va;
			 vbeta_prev  = vb;
		 }//Fim do if flag
		 } // Fim do while

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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
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
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 8499;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5
                           PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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

CCM_FUNC void Fast_Loop(void) {

    // ZONA CRÍTICA
    __disable_irq();
    flagdadosprontos = 0;

    // Captura snapshots (Cópia rápida)
    // Nota: iv_raw está na RAM lenta, mas a leitura é única e rápida.
    float raw_v = (float)iv_raw;
    float raw_w = (float)iw_raw;
    float raw_bus = (float)vbus_raw;
    __enable_irq();

    // O processamento pesado acontece agora usando dados e código na CCMRAM

    // 1. Filtros
    float iv_filtrada = Processar_Filtro(&filtro_iv, raw_v);
    float iw_filtrada = Processar_Filtro(&filtro_iw, raw_w);

    // 2. Conversão Física
    iv = -1.0f * (iv_filtrada - offset_iv_raw) * (3.3f / 4095.0f) / 0.02f;
    iw = -1.0f * (iw_filtrada - offset_iw_raw) * (3.3f / 4095.0f) / 0.02f;

    // 3. Kirchhoff e Vbus
    iu = -iv - iw;
    vbus = (raw_bus * (3.3f / 4095.0f)) * 21.6f;


}


// --- Configura os coeficientes para 250Hz ---
CCM_FUNC void Inicializar_Filtros(void) {
    // --- COEFICIENTES DE 1ª ORDEM (Bessel/Butterworth) ---

    // Numerador (Entrada)
    float B0 = 0.30926852f;
    float B1 = 0.30926852f;
    float B2 = 0.0f; // Zero (Filtro de 1ª ordem não tem z^-2)

    // Denominador (Saída/Feedback)
    // ATENÇÃO: Invertemos o sinal de A1 para negativo para manter o ganho unitário
    // na fórmula de subtração: saida = ... - (a1 * y1)
    float A1 = -0.38146295f;
    float A2 =  0.0f; // Zero

    // --- Configuração FASE V ---
    filtro_iv.b0 = B0; filtro_iv.b1 = B1; filtro_iv.b2 = B2;
    filtro_iv.a1 = A1; filtro_iv.a2 = A2;

    // Zera memória
    filtro_iv.x1 = 0; filtro_iv.x2 = 0;
    filtro_iv.y1 = 0; filtro_iv.y2 = 0;

    // --- Configuração FASE W ---
    filtro_iw = filtro_iv; // Copia tudo
}

// Mantém a função de processamento igual (ela funciona para 1ª e 2ª ordem)
CCM_FUNC float Processar_Filtro(EstruturaFiltro *f, float entrada) {
    float saida = (f->b0 * entrada) + (f->b1 * f->x1) + (f->b2 * f->x2)
                - (f->a1 * f->y1) - (f->a2 * f->y2);

    f->x2 = f->x1;
    f->x1 = entrada;

    f->y2 = f->y1;
    f->y1 = saida;

    return saida;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{



  if (hadc->Instance == hadc1.Instance)
  {

	  GPIOB->ODR ^= GPIO_PIN_12;
      iw_raw = adc1_buffer[0];

      flagdadosprontos++;
  }


  if (hadc->Instance == hadc2.Instance)
  {
	  GPIOB->ODR ^= GPIO_PIN_12;
      iv_raw = adc2_buffer[0];
      vbus_raw = adc2_buffer[1];
      flagdadosprontos++;
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
