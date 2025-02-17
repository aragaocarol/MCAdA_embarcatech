#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2818b.pio.h"
#include <math.h>
#include "hardware/flash.h"
#include "hardware/sync.h"


// Definindo os pinos e constantes
const uint I2C_SDA = 14;
const uint I2C_SCL = 15;

#define MIC_CHANNEL 2
#define MIC_PIN (26 + MIC_CHANNEL)
#define BUTTON_PIN 5
#define LED_PIN 13
#define LED2_PIN 12
#define LED3_PIN 11

// Definição do número de LEDs e pino.
#define LED_COUNT 25
#define LEDM_PIN 7

#define CALIBRATION_TIME 2000   // Tempo de calibração em milissegundos (4 segundos)
#define SAMPLES 1000            // Número de amostras para calcular o RMS
#define MAX_CALIBRATION_SAMPLES 1000  // Número de amostras durante a calibração
#define RMS_THRESHOLD 0.25f     // Limiar de RMS para detecção de som

#define DEBOUNCE_TIME 200      // Tempo de debounce em milissegundos

#define FLASH_TARGET_OFFSET (256 * 1024)  // Offset na memória flash
#define MAGIC_NUMBER 0xA5A5A5A5  // Número mágico para validar os dados


float adc_buffer[SAMPLES];
float calibration_rms = 0.0f; // Limiar de calibração

bool button_pressed = false;  // Flag para indicar que o botão foi pressionado

// Protótipos das funções
void init_hardware();
void display_message(char *linha1, char *linha2);
void sample_mic();
float calculate_rms();
void calibrate();
void set_rgb_led(uint8_t r, uint8_t g, uint8_t b);
void button_isr(uint gpio, uint32_t events);
void npInit(uint pin);
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b);
void npClear();
void npWrite(bool mod);
void save_calibration(float rms);
bool load_calibration(float *rms);


 // Preparar área de renderização para o display
 struct render_area frame_area = {
    start_column : 0,
    end_column : ssd1306_width - 1,
    start_page : 0,
    end_page : ssd1306_n_pages - 1
};

// Definição de pixel GRB
struct pixel_t {
    uint8_t G, R, B; // Três valores de 8-bits compõem um pixel.
  };
  typedef struct pixel_t pixel_t;
  typedef pixel_t npLED_t; // Mudança de nome de "struct pixel_t" para "npLED_t" por clareza.
  
  // Declaração do buffer de pixels que formam a matriz.
  npLED_t leds[LED_COUNT];
  
  // Variáveis para uso da máquina PIO.
  PIO np_pio;
  uint sm;

  // Estrutura para armazenar dados na memória flash
typedef struct {
    uint32_t magic;
    float calibration_rms;
} calibration_data_t;


  
  void save_calibration(float rms) {
    uint32_t ints = save_and_disable_interrupts();
    calibration_data_t data = {MAGIC_NUMBER, rms};
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, (uint8_t *)&data, sizeof(data));
    restore_interrupts(ints);
}

bool load_calibration(float *rms) {
    calibration_data_t *data = (calibration_data_t *)(XIP_BASE + FLASH_TARGET_OFFSET);
    if (data->magic == MAGIC_NUMBER) {
        *rms = data->calibration_rms;
        return true;
    }
    return false;
}

int main() {

    stdio_init_all();
    init_hardware();
    npInit(LEDM_PIN);
    npClear();

    bool calibrated = false;

    calculate_render_area_buffer_length(&frame_area);
    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);
    display_message("  Sistema ", "  Iniciado");
    set_rgb_led(255, 255, 255);
    sleep_ms(2000);
    set_rgb_led(0, 0, 0);

    if (!load_calibration(&calibration_rms)) {
        display_message("  Necessario", "  calibrar");
        sleep_ms(2000);
        calibrate();
        save_calibration(calibration_rms);
    } else {
        display_message("  Calibracao", "  Carregada");
        calibrated = 1;
        sleep_ms(2000);
    }
    

    
    while (true) {
        if (button_pressed) {
            button_pressed = false;
            display_message("Confirmar", "calibracao?");
            set_rgb_led(255, 255, 0);
            uint32_t start_time = time_us_32();
            bool confirm = false;
            while ((time_us_32() - start_time) < 3000000) {
                if (button_pressed) {
                    button_pressed = false;
                    confirm = true;
                    break;
                }
                sleep_ms(100);
            }
            if (confirm) {
                display_message("Calibracao", "iniciada");
                set_rgb_led(255, 0, 0);
                calibrate();
                save_calibration(calibration_rms);
                set_rgb_led(0, 255, 0);
                display_message("Calibracao", "concluida!");
                calibrated = 1;
                sleep_ms(1500);
                set_rgb_led(0, 0, 0);
            } else {
                display_message("Calibracao", "cancelada");
                set_rgb_led(0, 0, 0);
                sleep_ms(1500);
            }
        }
        if(calibrated){
            uint8_t ssd[ssd1306_buffer_length];
            memset(ssd, 0, ssd1306_buffer_length);
            render_on_display(ssd, &frame_area);

            float rms_value = calculate_rms();
            if (rms_value >= calibration_rms * 0.9f && rms_value <= calibration_rms * 1.1f) {
                set_rgb_led(0, 0, 255);
                display_message("Som detectado!", NULL);
                npWrite(1);
                sleep_ms(500);
                npClear();
                npWrite(0);
                sleep_ms(500);
                npWrite(1);
                sleep_ms(500);
                npClear();
                npWrite(0);
                sleep_ms(500);
                set_rgb_led(0, 0, 0);
            }
            sleep_ms(100);
        }
    }
}

// Inicializa a máquina PIO para controle da matriz de LEDs.
 //
void npInit(uint pin) {

  // Cria programa PIO.
  uint offset = pio_add_program(pio0, &ws2818b_program);
  np_pio = pio0;

  // Toma posse de uma máquina PIO.
  sm = pio_claim_unused_sm(np_pio, false);
  if (sm < 0) {
    np_pio = pio1;
    sm = pio_claim_unused_sm(np_pio, true); // Se nenhuma máquina estiver livre, panic!
  }

  // Inicia programa na máquina PIO obtida.
  ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

  // Limpa buffer de pixels.
  for (uint i = 0; i < LED_COUNT; ++i) {
    leds[i].R = 0;
    leds[i].G = 0;
    leds[i].B = 0;
  }
}

/**
 * Atribui uma cor RGB a um LED.
 */
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
  leds[index].R = r;
  leds[index].G = g;
  leds[index].B = b;
}

/**
 * Limpa o buffer de pixels.
 */
void npClear() {
  for (uint i = 0; i < LED_COUNT; ++i)
    npSetLED(i, 0, 0, 0);
}

/**
 * Escreve os dados do buffer nos LEDs.
 */
void npWrite(bool mod) {
    //    printf("Escrevendo dados nos LEDs...\n");  // Mensagem de depuração
    if (mod == 1 ) {
     for (uint i = 0; i < LED_COUNT; ++i)
        npSetLED(i, 0, 0, 100);
    }
    for (uint i = 0; i < LED_COUNT; ++i) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    //    printf("Dados escritos.\n");  // Mensagem de depuração
        sleep_us(500);  // Atraso
 }



void init_hardware() {
    // Inicializa ADC e GPIOs
    adc_gpio_init(MIC_PIN);
    adc_init();
    adc_select_input(MIC_CHANNEL);
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(LED2_PIN);
    gpio_set_dir(LED2_PIN, GPIO_OUT);
    gpio_init(LED3_PIN);
    gpio_set_dir(LED3_PIN, GPIO_OUT);
    
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);
    gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, &button_isr);
    
    // Inicializa I2C e display OLED
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init();
}

void display_message(char *linha1, char *linha2) {
    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);
    // Desenha a primeira linha se existir
    if (linha1 != NULL) {
        ssd1306_draw_string(ssd, 10, 16, linha1);
    }

    // Desenha a segunda linha se existir
    if (linha2 != NULL) {
        ssd1306_draw_string(ssd, 10, 24, linha2);
    }
    render_on_display(ssd, &frame_area);  // Atualiza o display
}

// Função para amostrar o microfone
void sample_mic() {
    for (int i = 0; i < SAMPLES; i++) {
        adc_buffer[i] = adc_read();  // Amostra o microfone
    }
}

// Função para calcular o RMS (Root Mean Square) das amostras
float calculate_rms() {
    float sum = 0.0f;

    // Realiza a amostragem dentro do cálculo de RMS para garantir leituras atualizadas
    sample_mic();

    // Calcula a soma dos quadrados das amostras
    for (int i = 0; i < SAMPLES; i++) {
        float amplitude = (adc_buffer[i] * 3.3f / (1 << 12)) - 1.65f; // Normaliza para o intervalo -1.65f a 1.65f
        sum += amplitude * amplitude;
    }

    // Calcula o RMS (raiz quadrada da média dos quadrados)
    float rms = sqrt(sum / SAMPLES);
    printf("RMS: %f\n", rms);

    return rms;
}

// Função de calibração ajustada para considerar os 5 maiores valores lidos
void calibrate() {
    printf("Iniciando calibracao...\n");

    unsigned long start_time = time_us_32();
    unsigned long current_time;

    float rms_values[MAX_CALIBRATION_SAMPLES];
    int sample_count = 0;

    // Captura valores RMS durante o tempo de calibração
    while (((current_time = time_us_32()) - start_time) < CALIBRATION_TIME * 1000) {
        if (sample_count < MAX_CALIBRATION_SAMPLES) {
            rms_values[sample_count] = calculate_rms();  // Armazena cada RMS lido
            sample_count++;
        }
        sleep_ms(10);  // Pequeno atraso entre amostras
    }

    // Se houver menos de 5 amostras, usa todas disponíveis
    int num_values = sample_count < 5 ? sample_count : 5;

    // Ordena os valores RMS em ordem decrescente
    for (int i = 0; i < sample_count - 1; i++) {
        for (int j = i + 1; j < sample_count; j++) {
            if (rms_values[j] > rms_values[i]) {  // Se j for maior que i, troca
                float temp = rms_values[i];
                rms_values[i] = rms_values[j];
                rms_values[j] = temp;
            }
        }
    }

    // Calcula a média dos 5 maiores valores
    float sum_top5 = 0.0f;
    for (int i = 0; i < num_values; i++) {
        sum_top5 += rms_values[i];
    }
    calibration_rms = sum_top5 / num_values;

    printf("Calibracao concluida! Limiar ajustado para: %f\n", calibration_rms);
}
// Função para controlar o LED RGB
void set_rgb_led(uint8_t r, uint8_t g, uint8_t b) {
    gpio_put(LED_PIN, r > 0);  // Acende o LED vermelho
    gpio_put(LED3_PIN, g > 0);  // Acende o LED verde
    gpio_put(LED2_PIN, b > 0);   // Acende o LED azul
}

// Função de interrupção para o botão
void button_isr(uint gpio, uint32_t events) {
    static uint32_t last_interrupt_time = 0;
    uint32_t current_time = time_us_32();

    // Debounce: ignora interrupções se o tempo entre elas for muito curto
    if (current_time - last_interrupt_time > DEBOUNCE_TIME * 1000) {
        button_pressed = true;  // Marca que o botão foi pressionado
        last_interrupt_time = current_time;  // Atualiza o último tempo de interrupção
    }
}
