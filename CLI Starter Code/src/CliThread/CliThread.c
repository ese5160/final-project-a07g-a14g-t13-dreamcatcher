/**************************************************************************/ /**
 * @file      CliThread.c
 * @brief     File for the CLI Thread handler. Uses FREERTOS + CLI
 * @author    Eduardo Garcia
 * @date      2020-02-15

 ******************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "CliThread.h"

/******************************************************************************
 * Defines
 ******************************************************************************/

/******************************************************************************
 * Globals / Statics (ABHIK)
 ******************************************************************************/
SemaphoreHandle_t xRxSemaphore = NULL;  // Remove 'static'

/** 
 * @brief Macro for firmware version string 
 */
#define FIRMWARE_VERSION  "0.0.1"


/******************************************************************************
 * Variables
 ******************************************************************************/
static int8_t *const pcWelcomeMessage =
    "FreeRTOS CLI.\r\nType Help to view a list of registered commands.\r\n";

// Clear screen command
const CLI_Command_Definition_t xClearScreen =
    {
        CLI_COMMAND_CLEAR_SCREEN,
        CLI_HELP_CLEAR_SCREEN,
        CLI_CALLBACK_CLEAR_SCREEN,
        CLI_PARAMS_CLEAR_SCREEN};

static const CLI_Command_Definition_t xResetCommand =
    {
        "reset",
        "reset: Resets the device\r\n",
        (const pdCOMMAND_LINE_CALLBACK)CLI_ResetDevice,
        0};

/******************************************************************************
 * Forward Declarations
 ******************************************************************************/
static void FreeRTOS_read(char *character);



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




/******************************************************************************
 * Callback Functions
 ******************************************************************************/
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


/******************************************************************************
 * CLI Thread
 ******************************************************************************/

void vCommandConsoleTask(void *pvParameters)
{
    // Create a counting semaphore with a max count that matches or exceeds
    // your RX ring buffer size. Initial count is 0. (ABHIK)
    xRxSemaphore = xSemaphoreCreateCounting(512, 0);
    configASSERT(xRxSemaphore != NULL);	
	
    // REGISTER COMMANDS HERE

    FreeRTOS_CLIRegisterCommand(&xClearScreen);
    FreeRTOS_CLIRegisterCommand(&xResetCommand);
    FreeRTOS_CLIRegisterCommand(&xVersionCommand);
    FreeRTOS_CLIRegisterCommand(&xTicksCommand);

    uint8_t cRxedChar[2], cInputIndex = 0;
    BaseType_t xMoreDataToFollow;
    /* The input and output buffers are declared static to keep them off the stack. */
    static char pcOutputString[MAX_OUTPUT_LENGTH_CLI], pcInputString[MAX_INPUT_LENGTH_CLI];
    static char pcLastCommand[MAX_INPUT_LENGTH_CLI];
    static bool isEscapeCode = false;
    static char pcEscapeCodes[4];
    static uint8_t pcEscapeCodePos = 0;

    // Any semaphores/mutexes/etc you needed to be initialized, you can do them here

    /* This code assumes the peripheral being used as the console has already
    been opened and configured, and is passed into the task as the task
    parameter.  Cast the task parameter to the correct type. */

    /* Send a welcome message to the user knows they are connected. */
    SerialConsoleWriteString(pcWelcomeMessage);
    char rxChar;
    for (;;)
    {
        /* This implementation reads a single character at a time.  Wait in the
        Blocked state until a character is received. */

        FreeRTOS_read(&cRxedChar);

        if (cRxedChar[0] == '\n' || cRxedChar[0] == '\r')
        {
            /* A newline character was received, so the input command string is
            complete and can be processed.  Transmit a line separator, just to
            make the output easier to read. */
            SerialConsoleWriteString("\r\n");
            // Copy for last command
            isEscapeCode = false;
            pcEscapeCodePos = 0;
            strncpy(pcLastCommand, pcInputString, MAX_INPUT_LENGTH_CLI - 1);
            pcLastCommand[MAX_INPUT_LENGTH_CLI - 1] = 0; // Ensure null termination

            /* The command interpreter is called repeatedly until it returns
            pdFALSE.  See the "Implementing a command" documentation for an
            explanation of why this is. */
            do
            {
                /* Send the command string to the command interpreter.  Any
                output generated by the command interpreter will be placed in the
                pcOutputString buffer. */
                xMoreDataToFollow = FreeRTOS_CLIProcessCommand(
                    pcInputString,        /* The command string.*/
                    pcOutputString,       /* The output buffer. */
                    MAX_OUTPUT_LENGTH_CLI /* The size of the output buffer. */
                );

                /* Write the output generated by the command interpreter to the
                console. */
                // Ensure it is null terminated
                pcOutputString[MAX_OUTPUT_LENGTH_CLI - 1] = 0;
                SerialConsoleWriteString(pcOutputString);

            } while (xMoreDataToFollow != pdFALSE);

            /* All the strings generated by the input command have been sent.
            Processing of the command is complete.  Clear the input string ready
            to receive the next command. */
            cInputIndex = 0;
            memset(pcInputString, 0x00, MAX_INPUT_LENGTH_CLI);
        }
        else
        {
            /* The if() clause performs the processing after a newline character
    is received.  This else clause performs the processing if any other
    character is received. */

            if (true == isEscapeCode)
            {

                if (pcEscapeCodePos < CLI_PC_ESCAPE_CODE_SIZE)
                {
                    pcEscapeCodes[pcEscapeCodePos++] = cRxedChar[0];
                }
                else
                {
                    isEscapeCode = false;
                    pcEscapeCodePos = 0;
                }

                if (pcEscapeCodePos >= CLI_PC_MIN_ESCAPE_CODE_SIZE)
                {

                    // UP ARROW SHOW LAST COMMAND
                    if (strcasecmp(pcEscapeCodes, "oa"))
                    {
                        /// Delete current line and add prompt (">")
                        sprintf(pcInputString, "%c[2K\r>", 27);
                        SerialConsoleWriteString(pcInputString);
                        /// Clear input buffer
                        cInputIndex = 0;
                        memset(pcInputString, 0x00, MAX_INPUT_LENGTH_CLI);
                        /// Send last command
                        strncpy(pcInputString, pcLastCommand, MAX_INPUT_LENGTH_CLI - 1);
                        cInputIndex = (strlen(pcInputString) < MAX_INPUT_LENGTH_CLI - 1) ? strlen(pcLastCommand) : MAX_INPUT_LENGTH_CLI - 1;
                        SerialConsoleWriteString(pcInputString);
                    }

                    isEscapeCode = false;
                    pcEscapeCodePos = 0;
                }
            }
            /* The if() clause performs the processing after a newline character
            is received.  This else clause performs the processing if any other
            character is received. */

            else if (cRxedChar[0] == '\r')
            {
                /* Ignore carriage returns. */
            }
            else if (cRxedChar[0] == ASCII_BACKSPACE || cRxedChar[0] == ASCII_DELETE)
            {
                char erase[4] = {0x08, 0x20, 0x08, 0x00};
                SerialConsoleWriteString(erase);
                /* Backspace was pressed.  Erase the last character in the input
                buffer - if there are any. */
                if (cInputIndex > 0)
                {
                    cInputIndex--;
                    pcInputString[cInputIndex] = 0;
                }
            }
            // ESC
            else if (cRxedChar[0] == ASCII_ESC)
            {
                isEscapeCode = true; // Next characters will be code arguments
                pcEscapeCodePos = 0;
            }
            else
            {
                /* A character was entered.  It was not a new line, backspace
                or carriage return, so it is accepted as part of the input and
                placed into the input buffer.  When a n is entered the complete
                string will be passed to the command interpreter. */
                if (cInputIndex < MAX_INPUT_LENGTH_CLI)
                {
                    pcInputString[cInputIndex] = cRxedChar[0];
                    cInputIndex++;
                }

                // Order Echo
                cRxedChar[1] = 0;
                SerialConsoleWriteString(&cRxedChar[0]);
            }
        }
    }
}

/**************************************************************************/ /**
 * @fn			void FreeRTOS_read(char* character)
 * @brief		STUDENTS TO COMPLETE. This function block the thread unless we received a character. How can we do this?
                 There are multiple solutions! Check all the inter-thread communications available! See https://www.freertos.org/a00113.html
 * @details		STUDENTS TO COMPLETE.
 * @note
 *****************************************************************************/
static void FreeRTOS_read(char *character)
{
	// Wait indefinitely until the semaphore is given by the USART read callback (ABHIK)
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
    // ToDo: Complete this function
    //vTaskSuspend(NULL); // We suspend ourselves. Please remove this when doing your code
}

/******************************************************************************
 * CLI Functions - Define here
 ******************************************************************************/

// THIS COMMAND USES vt100 TERMINAL COMMANDS TO CLEAR THE SCREEN ON A TERMINAL PROGRAM LIKE TERA TERM
// SEE http://www.csie.ntu.edu.tw/~r92094/c++/VT100.html for more info
// CLI SPECIFIC COMMANDS
static char bufCli[CLI_MSG_LEN];
BaseType_t xCliClearTerminalScreen(char *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    char clearScreen = ASCII_ESC;
    snprintf(bufCli, CLI_MSG_LEN - 1, "%c[2J", clearScreen);
    snprintf(pcWriteBuffer, xWriteBufferLen, bufCli);
    return pdFALSE;
}

// Example CLI Command. Resets system.
BaseType_t CLI_ResetDevice(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    system_reset();
    return pdFALSE;
}
