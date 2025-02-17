------Detector de Som Inteligente para Deficientes Auditivos-------


Descrição do Projeto

Este projeto visa desenvolver um dispositivo assistivo para pessoas com deficiência auditiva. Utilizando um Raspberry Pi Pico W como microcontrolador, o sistema é capaz de captar sons do ambiente por meio de um microfone de eletreto, processar o sinal através do cálculo do valor RMS e, se um som significativo for detectado, acionar diversos feedbacks visuais. Esses feedbacks incluem um LED RGB, uma matriz de LEDs 5x5 controlada via PIO e um display OLED, que exibe mensagens informativas. O sistema também possibilita a calibração da sensibilidade de detecção, armazenando o valor calibrado na memória não volátil para uso posterior.

----Funcionalidades---

Captação de Som: Utiliza um microfone de eletreto para captar os sons do ambiente.
Processamento de Sinal: Calcula o valor RMS do sinal captado para identificar sons relevantes.
Calibração: Permite ajustar a sensibilidade do sistema com base no ambiente e armazena o valor calibrado na memória flash.
Feedback Visual: Fornece alertas visuais por meio de um LED RGB, uma matriz de LEDs 5x5 (controlada via PIO) e um display OLED.

----Hardware---

Raspberry Pi Pico W: Microcontrolador principal.
Microfone de Eletreto: Captura do som ambiente.
Amplificador e ADC: Amplificação e conversão do sinal analógico do microfone.
Botões: Permitem iniciar a calibração e interagir com o sistema.
LED RGB: Indica os estados do sistema (por exemplo, calibração e detecção de som).
Matriz de LEDs 5x5 RGB: Exibe padrões visuais e cores, controlada via PIO.
Display OLED: Interface visual, conectada via I²C, para exibição de mensagens e status.
Memória Flash: Armazena dados críticos, como o valor calibrado, garantindo sua persistência mesmo após reinicializações.

-----Software-----

O software foi desenvolvido em C utilizando o SDK do Raspberry Pi Pico. Entre as principais funcionalidades, destacam-se:
-Captura e Processamento de Áudio: Leitura dos valores do ADC e cálculo do valor RMS.
-Calibração: Permite a calibração manual do sistema, com o valor calibrado sendo salvo na memória não volátil.
-Controle de Feedback Visual: Gerencia a exibição de informações através dos LEDs RGB, da matriz de LEDs e do display OLED.
-Uso do PIO: O PIO do Raspberry Pi Pico é utilizado para controlar a matriz de LEDs, garantindo temporizações precisas para a exibição dos dados de cor no formato GRB.
-Comunicação I²C: Gerencia a comunicação com o display OLED, enviando comandos e dados gráficos conforme o protocolo do SSD1306.

----Fluxo do Software--

O software inicia configurando todos os periféricos (ADC, GPIOs, I²C, PIO e Flash) e, em seguida, tenta carregar um valor de calibração armazenado na memória. Caso não haja calibração prévia, o sistema solicita que o usuário inicie o processo de calibração. Uma vez calibrado, o dispositivo entra em um loop de monitoramento contínuo, onde o som é captado, processado e comparado com o valor calibrado. Se o som estiver dentro dos parâmetros definidos, os feedbacks visuais são acionados para alertar o usuário.

----Instalação e Compilação----

Dependências e Configuração do Ambiente
SDK do Raspberry Pi Pico: Necessário para compilar e programar o microcontrolador.
Bibliotecas Utilizadas:
-pico/stdlib.h
-hardware/adc.h
-hardware/gpio.h
-hardware/i2c.h
-hardware/pio.h
-ssd1306.h (para o display OLED)
-ws2818b.pio.h (para a matriz de LEDs)
-Outras bibliotecas padrão do C.

----Passos para Compilação---

Configurar o Ambiente:
-Instale o SDK do Raspberry Pi Pico conforme as instruções oficiais.
-Configure a IDE (por exemplo, Visual Studio Code) para desenvolvimento em C/C++ com o SDK do Pico.
-Gravar o Firmware:
  Conecte o Raspberry Pi Pico W ao computador e grave o arquivo UF2 gerado, seguindo as instruções de gravação do dispositivo.

----Uso e Operação---

Após a inicialização, o sistema exibe a mensagem "Sistema Iniciado" no display OLED e acende o LED RGB como indicação de operação correta. Se um valor de calibração não estiver presente, o dispositivo solicita que o usuário inicie a calibração. Uma vez calibrado, o dispositivo monitora continuamente o ambiente. Quando um som com intensidade relevante é detectado, o sistema aciona os feedbacks visuais (LED RGB, matriz de LEDs e display OLED) para alertar o usuário.


Licença
Este projeto é licenciado sob a MIT License.


