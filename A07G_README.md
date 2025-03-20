# a07g-exploring-the-CLI

* Team Number: 13
* Team Name: DreamCatcher
* Team Members: Abhik, Toma
* GitHub Repository URL: https://github.com/ese5160/final-project-a07g-a14g-t13-dreamcatcher.git
* Description of test hardware: (development boards, sensors, actuators, laptop + OS, etc): A07 (CLI)

## 0. Install Percepio
![Percepio](IMAGESA07/Percepio_License.png)

## 1. Software Architecture


## 2. Understanding the Starter Code

### 1. What does `InitializeSerialConsole()` do?
The function `InitializeSerialConsole()` initializes the UART channel and sets up asynchronous communication. Specifically, it:
- Initializes the RX and TX circular buffers using `circular_buf_init()` (defined in `circular_buffer.c`).
- Configures the USART for UART operation at **115200 baud**, **8N1** format using `configure_usart()` function.
- Registers the callbacks for RX and TX using `configure_usart_callbacks()` function.
- Starts a continuous UART reading job with `usart_read_buffer_job()`.

---

### 2. In said function, what is `cbufRx` and `cbufTx`? What type of data structure is it?
`cbufRx` and `cbufTx` are **handles** for circular buffers used to store received and transmitted UART data.  
- Type: `cbuf_handle_t` (defined as `circular_buf_t *` in `circular_buffer.h`).  
- Circular buffer is implemented in `circular_buffer.c` as a `struct` containing:
  - `buffer`: pointer to the buffer.
  - `head`, `tail`: indexes to track the position of data.
  - `max`: buffer size.
  - `full`: flag to indicate if the buffer is full.

---

### 3. How are `cbufRx` and `cbufTx` initialized? Where is the library that defines them (please list the *.C file they come from)?
They are initialized in `InitializeSerialConsole()` in **`SerialConsole.c`**:
```c
cbufRx = circular_buf_init((uint8_t *)rxCharacterBuffer, RX_BUFFER_SIZE);
cbufTx = circular_buf_init((uint8_t *)txCharacterBuffer, TX_BUFFER_SIZE);
```
- `circular_buf_init()` allocates memory and sets up the circular buffer.

**Library:**  
- Defined in **`circular_buffer.c`**  
- Declared in **`circular_buffer.h`**  

---

### 4. Where are the character arrays where the RX and TX characters are being stored at the end? Please mention their name and size.
- **RX Buffer:** `rxCharacterBuffer` (Size: 512 bytes)  
- **TX Buffer:** `txCharacterBuffer` (Size: 512 bytes)  

Defined globally in `SerialConsole.c`:
```c
char rxCharacterBuffer[RX_BUFFER_SIZE];
char txCharacterBuffer[TX_BUFFER_SIZE];
```

---

### 5. Where are the interrupts for UART character received and UART character sent defined?
The interrupts are registered in `configure_usart_callbacks()` in **`SerialConsole.c`**:
```c
usart_register_callback(&usart_instance, usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
usart_register_callback(&usart_instance, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
```
The USART is configured with:
```c
usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
```

---

### 6. What are the callback functions that are called when:
#### a) A character is received? (RX)
- **Callback Function:** `usart_read_callback()`  
- **Actions:**
  - Adds the character to `cbufRx` using `circular_buf_put()`.
  - Initiates another read job using `usart_read_buffer_job()`.

#### b) A character has been sent? (TX)
- **Callback Function:** `usart_write_callback()`  
- **Actions:**
  - Gets the next character from `cbufTx` using `circular_buf_get()`.
  - Writes it using `usart_write_buffer_job()`.

---

### 7. Explain what is being done on each of these two callbacks and how they relate to the `cbufRx` and `cbufTx` buffers.
- **RX Callback:**  
  - When a character is received, `usart_read_callback()` is triggered.  
  - The received character is added to `cbufRx` using `circular_buf_put()` to store it in the RX buffer.  
  - Starts another read job using `usart_read_buffer_job()` to receive more characters continuously.  

- **TX Callback:**  
  - When a character is transmitted, `usart_write_callback()` is triggered.  
  - The next character is fetched from `cbufTx` using `circular_buf_get()`.  
  - The character is sent using `usart_write_buffer_job()`.  
  - The process continues until the TX buffer is empty.  

---

### 8. Draw a diagram that explains the program flow for UART receive – starting with the user typing a character and ending with how that character ends up in the circular buffer `cbufRx`. Please make reference to specific functions in the starter code.
```plaintext
[User Types Character] ---> [USART Hardware] ---> [USART Interrupt Triggered] ---> 
[usart_read_callback()] ---> [circular_buf_put(cbufRx)] ---> [Data stored in rxCharacterBuffer]
```

1. User types a character in the terminal.  
2. USART hardware detects the character and generates an interrupt.  
3. `usart_read_callback()` is triggered.  
4. The character is added to `cbufRx` using `circular_buf_put()`.  
5. The character is stored in `rxCharacterBuffer[]`.  

![UART_Receive](IMAGESA07/UART_receive.png)

---

### 9. Draw a diagram that explains the program flow for the UART transmission – starting from a string added by the program to the circular buffer `cbufTx` and ending on characters being shown on the screen of a PC (On Teraterm, for example). Please make reference to specific functions in the starter code.
```plaintext
[Program Adds String to cbufTx] ---> [usart_write_callback()] ---> 
[circular_buf_get(cbufTx)] ---> [usart_write_buffer_job()] ---> 
[Character Displayed on Terminal]
```

1. Program adds string to `cbufTx` using `circular_buf_put()`.  
2. If USART is ready, `usart_write_callback()` is triggered.  
3. The next character is fetched from `cbufTx` using `circular_buf_get()`.  
4. The character is transmitted using `usart_write_buffer_job()`.  
5. Characters are shown on the terminal.  

![UART_Transmit](IMAGESA07/UART_Transmit.png)
---

### 10. What is done on the function `StartTasks()` in main.c? How many threads are started?
- Initializes tasks required for the system.
- Specifically, it creates the **CLI task** using:
```c
if (xTaskCreate(vCommandConsoleTask, "CLI_TASK", CLI_TASK_SIZE, NULL, CLI_PRIORITY, &cliTaskHandle) != pdPASS)
```
- **Two threads** are created:
  - **CLI Task** – Handles the command-line interface.
  - **Daemon Task** – Created via `vApplicationDaemonTaskStartupHook()`.

Flow:
```plaintext
vApplicationDaemonTaskStartupHook()
    -> StartTasks()
        -> vCommandConsoleTask()
```

## 3. Debug Logger Module

```c
void LogMessage(enum eDebugLogLevels level, const char *format, ...)
{
    // Only log if the level is equal to or higher than the current debug level
    if (level < currentDebugLevel) {
        return;
    }

    char buffer[256];
    va_list args;
    va_start(args, format);

    // Format the message using vsnprintf()
    vsnprintf(buffer, sizeof(buffer), format, args);

    // Write the message to the serial console
    SerialConsoleWriteString(buffer);

    va_end(args);
}
```


```c
void usart_read_callback(struct usart_module *const usart_module)
{
    // Add the received character to the circular buffer
    circular_buf_put(cbufRx, latestRx);

    // Kick off another read operation to continuously receive data
    usart_read_buffer_job(&usart_instance, (uint8_t *)&latestRx, 1);
}
```


```c
#include "SerialConsole.h"

void testLogger(void) {
    int sensorTemperature = 75;

    setLogLevel(LOG_ERROR_LVL);

    LogMessage(LOG_INFO_LVL, "Starting test...\r\n"); // Won't print
    LogMessage(LOG_ERROR_LVL, "System error detected!\r\n"); // Will print
    LogMessage(LOG_FATAL_LVL, "Temperature over %d degrees!\r\n", sensorTemperature); // Will print
}

int main(void) {
    system_init();
    InitializeSerialConsole();

    testLogger();

    while (1) {
        // Main loop
    }

    return 0;
}
```

![Serial_Output](IMAGESA07/Debug_Logger_Output.png)

Serial Monitor Output

![Serial_Output](IMAGESA07/Logger_Output_Second.png)

Serial Monitor Output from the to check custom case

## 4. Wiretap the convo!


### 1. Which Nets to Hook the Logic Analyzer To?
Because the code in **SerialConsole.c** configures a standard UART (SERCOM) for communication with the EDBG, we need the following signals:

1. **SAMD21/SAMW25 TX** – This is the line going from the microcontroller to the EDBG (the EDBG’s RX pin) PB11.  
2. **SAMD21/SAMW25 RX** – This is the line going from the EDBG to the microcontroller (the EDBG’s TX pin) PB10.  
3. **Ground Reference** – Common GND is required so the logic analyzer readings have the same reference.

From the code snippet:

```c
config_usart.baudrate      = 115200;
config_usart.mux_setting   = EDBG_CDC_SERCOM_MUX_SETTING;
config_usart.pinmux_pad0   = EDBG_CDC_SERCOM_PINMUX_PAD0;
config_usart.pinmux_pad1   = EDBG_CDC_SERCOM_PINMUX_PAD1;
config_usart.pinmux_pad2   = EDBG_CDC_SERCOM_PINMUX_PAD2;
config_usart.pinmux_pad3   = EDBG_CDC_SERCOM_PINMUX_PAD3;
```

Those `pinmux_padX` macros map SERCOM pins to actual device pins that connect to the EDBG’s TX/RX lines. Physically, on the SAMW25 Xplained board, these are the nets that go directly to the on-board EDBG chip.


---

### 2. Where to Attach / Solder on the Board: : PB10 (UART_TX)
Depending on your hardware revision, you have a few options:

It also have UART DEBUG connetions.

#### Embedded Debugger Implementation
SAM W25 Xplained Pro contains an Embedded Debugger (EDBG) that can be used to program and debug the ATSAMW25H18-MR510PB using Serial Wire Debug (SWD). The Embedded Debugger also include a Virtual Com port interface over UART, an Atmel Data Gateway Interface over SPI and TWI and it monitors four of the SAM W25 GPIOs. Atmel Studio can be used as a front end for the Embedded Debugger.

In all cases, we should also locate a **GND pin or Ground test point** and connect that to the Saleae’s ground line.

![WireTapPin](IMAGESA07/WireTapPin.png)
[Reference for PIN: Pg 10](hhttps://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/UserGuides/Atmel-42463-SAM-W25-Xplained-Pro_UserGuide.pdf)

---

### 3. Critical Logic Analyzer Settings
When using the Saleae Logic software to decode UART traffic, configure:

![SALEA_Settings](IMAGESA07/SALEAE_ASYNC_Settings.png)

1. **Protocol**: UART (Asynchronous Serial).  
2. **Baud Rate**: 115200 (from `config_usart.baudrate`).  
3. **Data Format**: 8 data bits, No parity, 1 stop bit (8N1).  


![SAM_Connection](IMAGESA07/SAM_Saleae_Whole.jpeg)
![SAM_Connection](IMAGESA07/SAM_Saleae_Connection.jpeg)

![SALEA_Output](IMAGESA07/SALEAE_UART.png)

LOGIG_ANALYSER_CAPTURE_FILE:
[LOGIG_ANALYSER_CAPTURE_FILE](IMAGESA07/USART_LOGIC_ANYL.sal)

---

## 5. Complete the CLI


```c
/******************************************************************************
 * Includes
 ******************************************************************************/
#include "CliThread.h"

/******************************************************************************
 * Globals / Statics
 ******************************************************************************/
static SemaphoreHandle_t xRxSemaphore = NULL;  // Used to signal that a character is ready

/******************************************************************************
 * CLI Thread
 ******************************************************************************/
void vCommandConsoleTask(void *pvParameters)
{
    // Create a counting semaphore with a max count that matches or exceeds
    // your RX ring buffer size. Initial count is 0.
    xRxSemaphore = xSemaphoreCreateCounting(512, 0);
    configASSERT(xRxSemaphore != NULL);

    // Register CLI commands here ...
    FreeRTOS_CLIRegisterCommand(&xClearScreen);
    FreeRTOS_CLIRegisterCommand(&xResetCommand);

    // Print welcome, etc. omitted for brevity

    // The main loop:
    uint8_t cRxedChar[2], cInputIndex = 0;
    // ...
    // Existing variables: pcOutputString, pcInputString, etc.
    // ...

    for (;;)
    {
        // Block here until a character is received
        FreeRTOS_read(&cRxedChar[0]);

        // The rest of the CLI parsing logic remains the same
        // (processing newline, storing characters, backspace, etc.)
        // ...
    }
}
```

```c

/**
 * @brief  Blocks until a character is received into the ring buffer,
 *         then reads one character out of the buffer.
 * @param  character Pointer to store the retrieved character.
 */
static void FreeRTOS_read(char *character)
{
    // Wait indefinitely until the semaphore is given by the USART read callback
    if (pdTRUE == xSemaphoreTake(xRxSemaphore, portMAX_DELAY))
    {
        // We now expect at least 1 character in the ring buffer.
        // Pull one from the ring buffer with SerialConsoleReadCharacter().
        // That returns 0 on success, -1 if the buffer is empty.
        while (SerialConsoleReadCharacter((uint8_t*)character) == -1)
        {
            // Rare edge case: if the interrupt signaled but the buffer is empty,
            // loop briefly (should rarely happen unless concurrency is involved).
            // In practice, this loop ends immediately with a character.
        }
    }
}

```



```c

/**
 * @brief Callback invoked after receiving one character. Puts character into ring buffer
 *        and signals the CLI thread via semaphore.
 * @param[in] usart_module Pointer to the USART module (interrupt context).
 */

#include "CliThread.h"   // to access xRxSemaphore (or extern xRxSemaphore)

void usart_read_callback(struct usart_module *const usart_module)
{
    // Store the newly received character
    circular_buf_put(cbufRx, latestRx);

    // Continuously receive the next character
    usart_read_buffer_job(&usart_instance, (uint8_t *)&latestRx, 1);

    // Notify the CLI thread a character is available
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xRxSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

```

[CLI Starter Code](https://github.com/ese5160/ese5160s25-final-project-a07g-a14g-final-project-a07g-a14g-skeleton/blob/f630d9191c0a50245e7548707b4272f620e4db59/CLI%20Starter%20Code)

## 6. Add CLI commands

```c
/******************************************************************************
 * Includes
 ******************************************************************************/
#include "CliThread.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "SerialConsole.h"
#include "FreeRTOS_CLI.h"

/** 
 * @brief Macro for firmware version string 
 */
#define FIRMWARE_VERSION  "0.0.1"

/******************************************************************************
 * Command Definitions
 ******************************************************************************/

/**
 * @brief Command definition for "version".
 *
 * The user will type "version" to display the firmware version.
 */
static const CLI_Command_Definition_t xVersionCommand =
{
    "version",                               /* The command string to type. */
    "version: Prints the firmware version.\r\n",  /* Help text. */
    CLI_VersionCommand,                      /* The function to run. */
    0                                        /* No parameters needed. */
};

/**
 * @brief Command definition for "ticks".
 *
 * The user will type "ticks" to display the FreeRTOS tick count.
 */
static const CLI_Command_Definition_t xTicksCommand =
{
    "ticks",
    "ticks: Prints the current FreeRTOS tick count.\r\n",
    CLI_TicksCommand,
    0
};


```


```c
/**
 * @brief CLI callback for the "version" command.
 *
 * Prints the firmware version number as defined by #FIRMWARE_VERSION.
 *
 * @param[in]  pcWriteBuffer   Output buffer for the CLI response.
 * @param[in]  xWriteBufferLen Length of pcWriteBuffer.
 * @param[in]  pcCommandString The full command string as typed by the user (unused).
 * @return     pdFALSE indicating there is no more output to return.
 */
BaseType_t CLI_VersionCommand(int8_t *pcWriteBuffer,
                              size_t xWriteBufferLen,
                              const int8_t *pcCommandString)
{
    snprintf((char *)pcWriteBuffer, xWriteBufferLen,
             "Firmware Version: %s\r\n", FIRMWARE_VERSION);
    return pdFALSE; 
}

/**
 * @brief CLI callback for the "ticks" command.
 *
 * Prints the current FreeRTOS tick count (the number of ticks since the scheduler started).
 *
 * @param[in]  pcWriteBuffer   Output buffer for the CLI response.
 * @param[in]  xWriteBufferLen Length of pcWriteBuffer.
 * @param[in]  pcCommandString The full command string as typed by the user (unused).
 * @return     pdFALSE indicating there is no more output to return.
 */
BaseType_t CLI_TicksCommand(int8_t *pcWriteBuffer,
                            size_t xWriteBufferLen,
                            const int8_t *pcCommandString)
{
    TickType_t ticks = xTaskGetTickCount();
    snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Ticks: %u\r\n", (unsigned)ticks);
    return pdFALSE;
}
```


```c
void vCommandConsoleTask(void *pvParameters)
{
    // Create counting semaphore, etc. (already present)

    // Register your existing commands
    FreeRTOS_CLIRegisterCommand(&xClearScreen);
    FreeRTOS_CLIRegisterCommand(&xResetCommand);

    // REGISTER YOUR NEW COMMANDS
    FreeRTOS_CLIRegisterCommand(&xVersionCommand);
    FreeRTOS_CLIRegisterCommand(&xTicksCommand);

    // ... rest of your CLI loop remains unchanged ...
}

```


```c
#include "CliThread.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "SerialConsole.h"
#include "FreeRTOS_CLI.h"

#define FIRMWARE_VERSION "0.0.1"

/*-----------------------------------------------------------*/
// Commands
static const CLI_Command_Definition_t xVersionCommand =
{
    "version",
    "version: Prints the firmware version.\r\n",
    CLI_VersionCommand,
    0
};

static const CLI_Command_Definition_t xTicksCommand =
{
    "ticks",
    "ticks: Prints the current FreeRTOS tick count.\r\n",
    CLI_TicksCommand,
    0
};

/*-----------------------------------------------------------*/
// Callbacks
BaseType_t CLI_VersionCommand(int8_t *pcWriteBuffer,
                              size_t xWriteBufferLen,
                              const int8_t *pcCommandString)
{
    snprintf((char *)pcWriteBuffer, xWriteBufferLen,
             "Firmware Version: %s\r\n", FIRMWARE_VERSION);
    return pdFALSE; 
}

BaseType_t CLI_TicksCommand(int8_t *pcWriteBuffer,
                            size_t xWriteBufferLen,
                            const int8_t *pcCommandString)
{
    TickType_t ticks = xTaskGetTickCount();
    snprintf((char *)pcWriteBuffer, xWriteBufferLen,
             "Ticks: %u\r\n", (unsigned)ticks);
    return pdFALSE;
}

/*-----------------------------------------------------------*/
void vCommandConsoleTask(void *pvParameters)
{
    // ... existing code that sets up xRxSemaphore, etc. ...

    // Register all commands:
    FreeRTOS_CLIRegisterCommand(&xClearScreen);
    FreeRTOS_CLIRegisterCommand(&xResetCommand);
    FreeRTOS_CLIRegisterCommand(&xVersionCommand);
    FreeRTOS_CLIRegisterCommand(&xTicksCommand);

    // ... rest of your CLI loop ...
}

```


## 7. Using Percepio