# ESP Guard Vision

Sistema embarcado de detecção de intrusão com visão computacional na borda. Um ESP32-S3 executa inferência local de detecção de pedestres (ESP-DL) sobre frames de uma câmera OV5640, acionado por sensor PIR, e reporta evidências a um backend FastAPI com dashboard de monitoramento em tempo real.

O objetivo é reduzir falsos positivos de sistemas baseados apenas em PIR: o sensor de presença atua somente como gatilho de baixo custo energético, e o alarme só é registrado após confirmação por rede neural de que há de fato uma pessoa no campo de visão.

## Arquitetura

O sistema é composto por três camadas:

| Camada | Tecnologia | Função |
|---|---|---|
| Firmware | C++ / ESP-IDF 6.0.1 | Captura, inferência, evidências, upload |
| Backend | FastAPI + SQLite | Recepção, validação e persistência de detecções |
| Dashboard | HTML/JS autocontido | Feed ao vivo, alertas, estatísticas |

> Este repositório contém atualmente o **firmware**. Backend e dashboard serão incorporados em breve.

### Fluxo de operação (FSM)

O firmware é estruturado como uma máquina de estados finitos:

```
IDLE ──(gatilho PIR)──► VERIFY ──(confirmado)──► registro de evidências ──► COOLDOWN ──► IDLE
                          │
                          └──(timeout / sem confirmação)──► IDLE
```

- **IDLE** — inferência desligada; a câmera permanece aquecida (AEC/AWB ativos via dreno periódico de frames) para que o primeiro frame pós-gatilho já esteja bem exposto.
- **VERIFY** — janela curta de inferência (até 8 frames / 1,5 s). A confirmação exige múltiplos acertos (`CONFIRM_HITS = 2`) ou um único frame de alta confiança (`score ≥ 0.85`), eliminando falsos positivos do PIR.
- **Registro** — até 3 evidências JPEG espaçadas no tempo (documentam o deslocamento do intruso), gravadas no MicroSD e enviadas ao backend via HTTP multipart com retentativas.
- **COOLDOWN** — 5 s ignorando novos gatilhos, evitando alertas duplicados em cascata.

## Hardware

- **ESP32-S3** (dual-core LX7, PSRAM Octal SPI) — inferência acelerada por instruções vetoriais
- **Câmera OV5640** — DVP 8 bits, captura RGB565 em XGA (1024×768)
- **Sensor PIR** — gatilho de presença (GPIO 41, borda de subida)
- **Display OLED SSD1306** — status do dispositivo (I²C0)
- **Buzzer** — alerta sonoro local (PWM/LEDC)
- **MicroSD** — armazenamento local de evidências (SDMMC 1-bit)

### Pinagem

| Periférico | Sinais | GPIOs |
|---|---|---|
| OV5640 dados | D7–D0 | 16, 17, 18, 12, 10, 8, 9, 11 |
| OV5640 controle | XCLK / PCLK / VSYNC / HREF | 15 / 13 / 6 / 7 |
| OV5640 SCCB | SIOD / SIOC | 4 / 5 |
| SSD1306 | SDA / SCL | 14 / 21 |
| PIR | OUT | 41 |
| Buzzer | PWM (LEDC timer 1, canal 1) | 3 |
| MicroSD | CLK / CMD / D0 | 39 / 38 / 40 |

O buzzer usa deliberadamente `LEDC_TIMER_1`/`LEDC_CHANNEL_1`: o timer 0 e o canal 0 geram o XCLK da câmera, e compartilhá-los corromperia o frame seguinte a cada beep.

## Detalhes de implementação

- **Inferência**: modelo `pedestrian_detect` (esp-dl 3.3.4), quantizado int8, entrada RGB565BE diretamente do framebuffer em PSRAM.
- **Estabilidade de imagem**: 3 framebuffers em PSRAM com `CAMERA_GRAB_WHEN_EMPTY` evitam tearing por contenção entre o DMA da câmera e a leitura da CPU durante inferência.
- **Validação no backend**: uploads são verificados (magic bytes JPEG, limite de tamanho) antes da gravação atômica em disco e registro no banco.
- **Provisionamento**: na ausência de credenciais salvas, o dispositivo sobe um AP (`KIIRA-Config`) com portal web para configurar SSID, senha e IP do backend, persistidos em NVS.
- **Configuração remota**: o firmware consulta periodicamente o endpoint `/config` do backend (limiar de score, intervalo, arme/desarme).
- **Resiliência**: evidências são sempre gravadas no MicroSD, independentemente do sucesso do upload; o display sinaliza `BACKEND OFFLINE` após falhas consecutivas.

## Compilação e gravação

Requisitos: [ESP-IDF 6.0.1](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/) com target `esp32s3`. As dependências de componentes (esp32-camera, esp-dl, pedestrian_detect, cJSON) são resolvidas automaticamente pelo IDF Component Manager via `dependencies.lock`.

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p <PORTA> flash monitor
```

O projeto inclui um `.devcontainer` para desenvolvimento em ambiente Docker com o toolchain completo.

## Protocolo de comunicação

As evidências são enviadas via `POST /upload` em `multipart/form-data`, contendo o binário JPEG e os metadados da detecção (device_id, score, bounding box em coordenadas XGA). A resposta do backend confirma o registro; o firmware aplica até 3 retentativas por evidência.

## Estrutura do repositório

```
.
├── main/                # Firmware (app_main.cpp, componentes)
├── CMakeLists.txt
├── partitions.csv       # Tabela de partições
├── dependencies.lock    # Versões fixadas dos componentes
├── sdkconfig.defaults
└── .devcontainer/       # Ambiente Docker de desenvolvimento
```

## Roadmap

- [ ] Publicação do backend (FastAPI) e do dashboard neste repositório
- [ ] Validação em campo do parâmetro de dreno de frames de evidência
- [ ] Documentação de montagem do hardware

## Licença

Projeto acadêmico — plataforma KIIRA.
