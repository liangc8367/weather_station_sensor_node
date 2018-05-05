/*
 * IoT Sensor Node
 *
 * Revision
 * ========
 * 2018-05-04   liangc  rewrite based on TI's rfWsn example code.
 *
 * Todo List
 * =========
 * - disable Display module, replace display output with System_printf;
 * - rewrite node task to be event-driven;
 * - rewrite sampling process: timer driven;
 * - rewrite button process: a) post event to node task; b) node task to
 *   start debounce timer; c) recheck button state at timeout; d) send
 *   sample-timer setting to IOT hub;
 *
 * - rewrite code in C++;
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/display/Display.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/* Board Header files */
#include "Board.h"

/* Application Header files */ 
#include "NodeRadioTask.h"
#include "NodeTask.h"


/*
 *  ======== main ========
 */
int main(void)
{
    /* Call driver init functions. */
    Board_initGeneral();

    /* Initialize sensor node tasks */
    NodeRadioTask_init();
    NodeTask_init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
