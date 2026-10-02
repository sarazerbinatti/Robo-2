# Robo 2 — Soccer IR

Firmware de um robô de futebol autônomo com movimentação omnidirecional, desenvolvido em **C++/Arduino** sobre **PlatformIO** para **Arduino Mega 2560**.

O projeto organiza o controle do robô em módulos independentes para motores, movimentação, sensores, bússola e controle PID. No estado atual, o robô utiliza um sensor infravermelho para localizar a bola e uma bússola para corrigir sua orientação durante o movimento.

> **Status:** em desenvolvimento. Algumas interfaces e pinos já estão preparados para funcionalidades que ainda não participam da lógica principal de movimento.

---

## Funcionalidades atuais

- Controle de **4 motores DC** com PWM e direção;
- Cinemática para movimentação **omnidirecional** com 4 rodas;
- Normalização proporcional das velocidades dos motores;
- Leitura de sensor IR de bola usando `HTInfraredSeeker`;
- Controle lateral do robô conforme a direção detectada da bola;
- Leitura de bússola via **I²C**;
- Correção de orientação por **PID**;
- Filtro passa-baixas no termo derivativo do PID;
- Limitação do termo integral e da derivada;
- Ajuste dos ganhos PID em tempo real pela porta serial;
- Leitura de 4 sensores de linha QRE1113;
- Telemetria serial para depuração.

## Ainda em desenvolvimento

- Reação automática aos sensores de linha;
- Uso dos sensores ultrassônicos configurados em `Config.h`;
- Controle de avanço/recuo em direção à bola;
- Estratégias completas de Soccer IR;
- Testes automatizados na pasta `test/`.

---

## Hardware previsto

| Componente | Uso no projeto |
|---|---|
| Arduino Mega 2560 | Controlador principal |
| 4 motores DC | Locomoção omnidirecional |
| Drivers de motor | Acionamento dos motores |
| 4 rodas omni | Movimento em múltiplas direções |
| HiTechnic IR Seeker | Detecção da direção e intensidade da bola |
| 4 sensores QRE1113 | Detecção das linhas do campo |
| Bússola I²C | Correção da orientação do robô |
| Sensores ultrassônicos | Reservados para futura detecção de distância |

---

## Tecnologias

- **C++**
- **Arduino Framework**
- **PlatformIO**
- **Arduino Mega 2560 / ATmega2560**
- **I²C**
- Controle **PID**

---

## Estrutura do projeto

```text
Robo-2/
├── include/
│   ├── config/
│   │   └── Config.h
│   ├── control/
│   │   └── PIDController.h
│   ├── hardware/
│   │   ├── BallSensor.h
│   │   ├── Compass.h
│   │   ├── LineSensor.h
│   │   ├── Motor.h
│   │   └── OmniDrive.h
│   └── robot/
│       └── Robot.h
│
├── lib/
│   └── HTInfraredSeeker/
│       ├── HTInfraredSeeker.cpp
│       └── HTInfraredSeeker.h
│
├── src/
│   ├── control/
│   ├── hardware/
│   ├── robot/
│   └── main.cpp
│
├── test/
├── platformio.ini
└── README.md
```

### Responsabilidade dos módulos

| Módulo | Responsabilidade |
|---|---|
| `Robot` | Coordena sensores, PID, movimento, serial e ciclo principal |
| `Motor` | Controla direção e PWM de um motor individual |
| `OmniDrive` | Converte frente/lateral/giro nas velocidades dos quatro motores |
| `BallSensor` | Encapsula o sensor infravermelho de bola |
| `Compass` | Faz leitura I²C, calibração e cálculo de erro angular |
| `LineSensor` | Lê e classifica os quatro sensores QRE1113 |
| `PIDController` | Executa o controle PID de orientação |
| `Config.h` | Centraliza pinos, calibrações e constantes do robô |

---

## Fluxo principal

O `main.cpp` é propositalmente simples:

```cpp
Robot robo;

void setup()
{
    robo.begin();
}

void loop()
{
    robo.update();
}
```

A cada chamada de `Robot::update()` o firmware executa, nesta ordem:

1. Processamento dos comandos recebidos pela serial;
2. Atualização do sensor IR, sensores de linha e bússola;
3. Atualização do PID de orientação;
4. Definição do movimento lateral conforme a posição da bola;
5. Cálculo e aplicação das velocidades dos quatro motores;
6. Envio periódico da telemetria de debug.

---

## Controle omnidirecional

O movimento é representado por três componentes:

- `frente` — avanço ou recuo;
- `lateral` — movimento para esquerda ou direita;
- `giro` — rotação do robô.

A classe `OmniDrive` combina essas três componentes para produzir os comandos dos quatro motores:

```text
A = -frente + lateral + giro
B =  frente - lateral + giro
C =  frente + lateral + giro
D = -frente - lateral + giro
```

Quando algum valor ultrapassa o intervalo PWM de `-255` a `255`, todos os motores são escalados proporcionalmente. Isso preserva o vetor de movimento em vez de simplesmente cortar os valores individualmente.

No comportamento atual, `frente` permanece em `0`, enquanto `lateral` é definido pelo sensor IR e `giro` pelo PID da bússola.

---

## Sensor IR da bola

O sensor é lido através da biblioteca `HTInfraredSeeker`.

O firmware utiliza atualmente as direções centrais do sensor para determinar o movimento lateral:

| Direção IR | Movimento lateral |
|---:|---:|
| `7` | `-120` |
| `6` | `-90` |
| `5` | `0` |
| `4` | `90` |
| `3` | `120` |
| Outras | `0` |

Esses valores podem ser alterados em `include/config/Config.h`.

---

## Controle de orientação

A bússola é consultada via I²C. O erro angular é normalizado para o intervalo de `-180°` a `180°` e utilizado como entrada do controlador PID.

Valores padrão:

```cpp
PID_KP = 6.0
PID_KI = 0.0
PID_KD = 1.5
```

O PID possui:

- intervalo mínimo entre cálculos;
- limitação do acumulador integral;
- limitação da derivada;
- filtro passa-baixas no termo derivativo;
- limitação da saída final;
- zona morta da bússola.

A saída é convertida no componente `giro` da movimentação omnidirecional.

---

## Sensores de linha

São utilizados quatro sensores analógicos QRE1113 nos pinos:

```text
QRE 1 -> A1
QRE 2 -> A2
QRE 3 -> A3
QRE 4 -> A4
```

Cada sensor possui valores de referência para o piso verde e para a linha branca. O limiar é calculado pela média desses valores.

No estado atual do firmware, os sensores já são lidos e a classe `LineSensor` consegue identificar a linha, porém essa informação **ainda não altera o movimento do robô**.

---

## Pinagem atual

### Motores

| Motor | PWM | IN1 | IN2 |
|---|---:|---:|---:|
| A | 2 | 4 | 5 |
| B | 3 | 6 | 7 |
| C | 8 | 10 | 11 |
| D | 9 | 12 | 13 |

### Sensores de linha

| Sensor | Pino |
|---|---|
| QRE 1 | A1 |
| QRE 2 | A2 |
| QRE 3 | A3 |
| QRE 4 | A4 |

### Ultrassônicos reservados

| Sensor | TRIG | ECHO |
|---|---:|---:|
| 1 | 22 | 23 |
| 2 | 24 | 25 |
| 3 | 26 | 27 |

> Os ultrassônicos possuem pinagem definida, mas ainda não possuem implementação no fluxo principal do firmware.

---

## Comunicação serial

A serial opera a **115200 baud**.

Além da telemetria automática, o firmware aceita comandos terminados por quebra de linha (`\n`).

### Consultar PID

```text
GET PID
```

Resposta esperada:

```text
OK PID KP=6.000000 KI=0.000000 KD=1.500000
```

### Alterar PID

```text
SET PID <KP> <KI> <KD>
```

Exemplo:

```text
SET PID 5.5 0.0 1.2
```

### Resetar o controlador

```text
RESET PID
```

### Ativar PID

```text
PID ON
```

### Desativar PID

```text
PID OFF
```

---

## Telemetria

A cada intervalo definido por `DEBUG_INTERVAL_MS`, o robô envia uma linha semelhante a:

```text
[DEBUG] IR: dir=5 str=120 | COMPASS: angle=2 target=0 err=-2 | PID: P=-12.00 I=0.00 D=... OUT=... | MOVE: F=0 L=0 G=... | MOTOR: A=... B=... C=... D=...
```

Essa saída permite acompanhar:

- direção e intensidade do sinal IR;
- ângulo da bússola;
- erro angular;
- termos P, I e D;
- saída do controlador;
- vetor de movimento;
- comandos enviados aos quatro motores.

---

## Como compilar

### Requisitos

- [Visual Studio Code](https://code.visualstudio.com/) com PlatformIO, ou PlatformIO Core;
- Cabo USB compatível com o Arduino Mega;
- Drivers da placa instalados no sistema.

### Usando VS Code + PlatformIO

1. Clone o repositório:

```bash
git clone https://github.com/sarazerbinatti/Robo-2.git
cd Robo-2
```

2. Abra a pasta no VS Code.

3. Aguarde o PlatformIO carregar o ambiente `megaatmega2560`.

4. Compile usando **PlatformIO: Build**.

5. Conecte o Arduino Mega e use **PlatformIO: Upload**.

6. Para acompanhar a telemetria, abra o monitor serial em `115200` baud.

### Usando o terminal

```bash
pio run
```

Upload:

```bash
pio run --target upload
```

Monitor serial:

```bash
pio device monitor -b 115200
```

---

## Configuração

As principais constantes ficam em:

```text
include/config/Config.h
```

É nesse arquivo que devem ser ajustados:

- pinagem;
- calibração dos sensores QRE;
- endereço da bússola;
- ganhos PID padrão;
- limites e filtros do PID;
- velocidades laterais;
- baud rate da serial;
- intervalo de debug;
- zona morta da bússola.

---

## Observações de desenvolvimento

O projeto ainda está evoluindo e alguns pontos devem ser revisados conforme o hardware definitivo do robô for fechado.

Em especial:

- recalibrar os QRE no campo real antes da competição;
- validar sentido e posição física dos motores após qualquer alteração mecânica;
- ajustar os ganhos do PID com o robô montado;
- implementar a lógica de escape das linhas;
- integrar os sensores ultrassônicos, caso sejam mantidos no projeto;
- adicionar testes unitários para os módulos que não dependem diretamente do hardware.

---

## Licença

Este projeto é disponibilizado sob a **PolyForm Noncommercial License 1.0.0**.

Você pode **usar, estudar, modificar e redistribuir** este projeto para fins educacionais, acadêmicos, experimentais, de pesquisa e outros usos **não comerciais**, desde que os avisos de autoria e a licença original sejam preservados.

O uso comercial deste projeto, de partes substanciais do código ou de versões derivadas **não é permitido sem autorização prévia dos autores**.

Ao reutilizar ou adaptar o projeto, mantenha os créditos aos autores e uma referência ao repositório original.

> Por possuir restrição de uso comercial, este projeto não se enquadra na definição formal de *Open Source* da Open Source Initiative (OSI). Ele é disponibilizado como código-fonte aberto para uso não comercial.

Consulte o arquivo [`LICENSE`](LICENSE) para os termos completos.
