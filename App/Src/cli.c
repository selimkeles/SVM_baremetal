#include "cli.h"
#include "cli_usb.h"
#include "inverter.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define LINE_BUF_LEN  128
#define MAX_ARGS      4

/* ---- forward declarations -------------------------------------------- */
static void cmd_help   (int argc, char **argv);
static void cmd_freq   (int argc, char **argv);
static void cmd_enable (int argc, char **argv);
static void cmd_disable(int argc, char **argv);
static void cmd_status (int argc, char **argv);

/* ---- command table ---------------------------------------------------- */
typedef void (*cli_cmd_fn)(int argc, char **argv);

typedef struct {
    const char  *name;
    cli_cmd_fn   fn;
    const char  *help;
} cli_cmd_t;

static const cli_cmd_t s_cmds[] = {
    { "help",    cmd_help,    "list commands" },
    { "freq",    cmd_freq,    "freq <hz>  set frequency 0-60 Hz" },
    { "enable",  cmd_enable,  "enable PWM output (MOE on)" },
    { "disable", cmd_disable, "disable PWM output (MOE off)" },
    { "status",  cmd_status,  "print freq / m / theta / enabled" },
};
#define N_CMDS  ((int)(sizeof(s_cmds) / sizeof(s_cmds[0])))

/* ---- tokeniser + dispatcher ------------------------------------------ */
static void dispatch(char *line)
{
    char *argv[MAX_ARGS];
    int   argc = 0;
    char *p    = line;

    while (*p && argc < MAX_ARGS) {
        while (*p == ' ') p++;
        if (*p == '\0') break;
        argv[argc++] = p;
        while (*p && *p != ' ') p++;
        if (*p) *p++ = '\0';
    }
    if (argc == 0) return;

    for (int i = 0; i < N_CMDS; i++) {
        if (strcmp(argv[0], s_cmds[i].name) == 0) {
            s_cmds[i].fn(argc, argv);
            return;
        }
    }
    cli_printf("unknown: %s  (type 'help')\r\n", argv[0]);
}

/* ---- command handlers ------------------------------------------------ */
static void cmd_help(int argc, char **argv)
{
    (void)argc; (void)argv;
    for (int i = 0; i < N_CMDS; i++) {
        cli_printf("  %-10s  %s\r\n", s_cmds[i].name, s_cmds[i].help);
    }
}

static void cmd_freq(int argc, char **argv)
{
    if (argc < 2) {
        cli_printf("usage: freq <hz>\r\n");
        return;
    }
    int hz = atoi(argv[1]);
    inverter_set_freq_hz(hz);
    cli_printf("freq=%d Hz\r\n", inverter_get_freq_hz());
}

static void cmd_enable(int argc, char **argv)
{
    (void)argc; (void)argv;
    inverter_enable(true);
    cli_printf("enabled\r\n");
}

static void cmd_disable(int argc, char **argv)
{
    (void)argc; (void)argv;
    inverter_enable(false);
    cli_printf("disabled\r\n");
}

static void cmd_status(int argc, char **argv)
{
    (void)argc; (void)argv;

    int   freq = inverter_get_freq_hz();
    int   en   = inverter_is_enabled() ? 1 : 0;

    /* Avoid printf-float dependency (newlib-nano may omit it).
     * Scale to fixed-point integers for display. */
    float m     = inverter_get_modulation();
    float theta = inverter_get_theta_deg();

    int m_thou  = (int)(m     * 1000.0f + 0.5f);   /* 0–1000 */
    int theta_i = (int)(theta + 0.5f);              /* 0–360  */

    cli_printf("freq=%d Hz  m=%d.%03d  theta=%d deg  en=%d\r\n",
               freq, m_thou / 1000, m_thou % 1000, theta_i, en);
}

/* ---- task entry ------------------------------------------------------ */
void cli_task(void *arg)
{
    (void)arg;

    char    line[LINE_BUF_LEN];
    int     pos = 0;
    uint8_t ch;

    /* Welcome banner — may silently fail if host hasn't opened the port yet. */
    cli_printf("\r\nSVM inverter ready. Type 'help'.\r\n");

    for (;;) {
        if (cli_usb_read(&ch, 1, 100) != 1) {
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            if (pos > 0) {
                line[pos] = '\0';
                dispatch(line);
                pos = 0;
            }
        } else if (ch == 0x08 || ch == 0x7F) {   /* BS / DEL */
            if (pos > 0) pos--;
        } else if (pos < LINE_BUF_LEN - 1) {
            line[pos++] = (char)ch;
        }
    }
}
