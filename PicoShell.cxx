/*
 * PicoShell.cxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#include <ctime>
#include <cstring>
#include <malloc.h>
#include <hardware/clocks.h>
#include <FreeRTOS.h>
#include <task.h>
#include <pico-plat.h>
#include <PicoPlatform.hxx>
#include <PicoShell.hxx>

PicoShell::PicoShell(enum PicoShellDevice device)
    : _device(device)
{
    _noEcho = false;
    _inproc.i = 0;
    _since = time(NULL);
    _help_list.push_back("help");
    _help_list.push_back("version");
    _help_list.push_back("system");
    _help_list.push_back("reboot");
    _help_list.push_back("bootsel");
}

PicoShell::~PicoShell()
{

}

int PicoShell::process(void)
{
    int ret = 0;
    int rx;
    char c;

    while (this->rx_ready() > 0) {
        rx = this->rx_read((uint8_t *) &c, 1);
        if (rx < 0) {
            ret = rx;
            break;
        } else if (rx == 0) {
            break;
        }

        ret += rx;

        if (c == 0xff) {  // IAC received
            static const uint8_t iac_do_tm[3] = { 0xff, 0xfd, 0x06};
            static const uint8_t iac_will_tm[3] = { 0xff, 0xfb, 0x06};
            char iac2;

            ret = this->rx_read((uint8_t *) &iac2, 1);
            if (ret == 1) {
                switch (iac2) {
                case 0xf4:  // IAC IP (interrupt process)
                    ret = this->tx_write(iac_do_tm, sizeof(iac_do_tm));
                    ret = this->tx_write(iac_will_tm, sizeof(iac_will_tm));
                    this->printf("\n> ");
                    _inproc.i = 0;
                    break;
                default:
                    break;
                }
            }

            continue;
        }

        if (c == '\r') {
            _inproc.cmdline[_inproc.i] = '\0';
            if (!_noEcho) {
                this->printf("\n");
            }
            this->exec(_inproc.cmdline);
            this->printf("> ");
            _inproc.i = 0;
            _inproc.cmdline[0] = '\0';
        } else if ((c == '\x7f') || (c == '\x08')) {
            if (_inproc.i > 0) {
                this->printf("\b \b");
                _inproc.i--;
            }
        } else if (c == '\x03') {
            this->printf("^C\n> ");
            _inproc.i = 0;
        } else if ((c != '\n') && isprint(c)) {
            if (_inproc.i < (CMDLINE_SIZE - 1)) {
                if (!_noEcho) {
                    this->printf("%c", c);
                }
                _inproc.cmdline[_inproc.i] = c;
                _inproc.i++;
            }
        }
    }

    return ret;
}

int PicoShell::exec(char *cmdline)
{
    int ret = 0;
    int argc = 0;
    char *argv[32];

    if (cmdline == NULL) {
        ret = -1;
        goto done;
    }

    bzero(argv, sizeof(argv));

    // Tokenize cmdline into argv[]
    while (*cmdline != '\0' && (argc < 32)) {
        while ((*cmdline != '\0') && isspace((int) *cmdline)) {
            cmdline++;
        }

        if (*cmdline == '\0') {
            break;
        }

        argv[argc] = cmdline;
        argc++;

        while ((*cmdline != '\0') && !isspace((int) *cmdline)) {
            cmdline++;
        }

        if (*cmdline == '\0') {
            break;
        }

        *cmdline = '\0';
        cmdline++;
    }

    if (argc < 1) {
        ret = -1;
        goto done;
    }

    if (strcmp(argv[0], "help") == 0) {
        ret = this->help(argc, argv);
    } else if (strcmp(argv[0], "version") == 0) {
        ret = this->version(argc, argv);
    } else if (strcmp(argv[0], "system") == 0) {
        ret = this->system(argc, argv);
    } else if (strcmp(argv[0], "reboot") == 0) {
        ret = this->reboot(argc, argv);
    } else if (strcmp(argv[0], "bootsel") == 0) {
        ret = this->bootsel(argc, argv);
    } else {
        ret = this->unknown_command(argc, argv);
    }

done:

    return ret;
}

int PicoShell::help(int argc, char **argv)
{
    int ret = 0;
    unsigned int i;

    (void)(argc);
    (void)(argv);

    this->printf("Available commands:\n");

    i = 0;
    for (vector<string>::const_iterator it = _help_list.begin();
         it != _help_list.end(); it++, i++) {
        if ((i % 4) == 0) {
            this->printf("\t");
        }

        this->printf("%s\t", it->c_str());

        if ((i % 4) == 3) {
            this->printf("\n");
        }
    }

    if ((i % 4) != 0) {
        this->printf("\n");
    }

    return ret;
}

int PicoShell::version(int argc, char **argv)
{
    int ret = 0;

    (void)(argc);
    (void)(argv);

    this->printf("%s\n", _banner.c_str());
    this->printf("%s\n", _version.c_str());
    this->printf("%s\n", _built.c_str());
    this->printf("-------------------------------------------\n");
    this->printf("%s\n", _copyright.c_str());

    return ret;
}

int PicoShell::system(int argc, char **argv)
{
    int ret = 0;
    time_t now;
    unsigned int uptime, days, hour, min, sec;
    extern char __StackLimit, __bss_end__;
    struct mallinfo m = mallinfo();
    unsigned int total_heap = &__StackLimit  - &__bss_end__;
    unsigned int used_heap = m.uordblks;
    unsigned int free_heap = total_heap - used_heap;
    char cTaskListBuffer[512];
    shared_ptr<PicoPlatform> pico = PicoPlatform::get();

    this->printf("  Platform: %s\n", pico->getName().c_str());
    now = time(NULL);
    uptime = now - _since;
    sec = (uptime % 60);
    min = (uptime / 60) % 60;
    hour = (uptime / 3600) % 24;
    days = (uptime) / 86400;
    if (days == 0) {
        this->printf("   Up-time: %.2u:%.2u:%.2u\n", hour, min, sec);
    } else {
        this->printf("   Up-time: %ud %.2u:%.2u:%.2u\n", days, hour, min, sec);
    }
    this->printf("Total Heap: %8u bytes\n", total_heap);
    this->printf(" Free Heap: %8u bytes\n", free_heap);
    this->printf(" Used Heap: %8u bytes\n", used_heap);
    this->printf("Board Temp:     %.1fC\n", pico->getOnboardTempC());
    if ((argc == 2) && (strcmp(argv[1], "-v") == 0)) {
        this->printf("clk_ref:  %lu Hz\n", clock_get_hz(clk_ref));
        this->printf("clk_sys:  %lu Hz\n", clock_get_hz(clk_sys));
        this->printf("clk_usb:  %lu Hz\n", clock_get_hz(clk_usb));
        this->printf("clk_adc:  %lu Hz\n", clock_get_hz(clk_adc));
        this->printf("clk_peri: %lu Hz\n", clock_get_hz(clk_peri));
    }
    bzero(cTaskListBuffer, sizeof(cTaskListBuffer));
    vTaskListTasks(cTaskListBuffer, sizeof(cTaskListBuffer));
    this->printf("  FreeRTOS:\n");
    this->printf("Name        State  Priority  StackRem   Task#   CPU Affn\n");
    this->printf("--------------------------------------------------------\n");
    this->printf("%s", cTaskListBuffer);

    return ret;
}

int PicoShell::reboot(int argc, char **argv)
{
    int ret = 0;

    (void)(argc);
    (void)(argv);
    this->printf("Rebooting ...\n");
    PicoPlatform::get()->reboot();

    return ret;
}

int PicoShell::bootsel(int argc, char **argv)
{
    int ret = 0;

    (void)(argc);
    (void)(argv);
    this->printf("Rebooting to BOOTSEL mode\n");
    PicoPlatform::get()->bootsel();

    return ret;
}

int PicoShell::unknown_command(int argc, char **argv)
{
    (void)(argc);

    this->printf("Unknown command '%s'!\n", argv[0]);

    return -1;
}

int PicoShell::tx_write(const uint8_t *buf, size_t size)
{
    int ret = 0;

    switch (_device) {
    case PICO_SHELL_USB_CDC: ret = usbcdc_write(buf, size); break;
    case PICO_SHELL_SERIAL0: ret = serial0_write(buf, size); break;
    case PICO_SHELL_SERIAL1: ret = serial1_write(buf, size); break;
    default: break;
    }

    return ret;
}

int PicoShell::printf(const char *format, ...)
{
    int ret = 0;
    va_list ap;

    va_start(ap, format);
    switch (_device) {
    case PICO_SHELL_USB_CDC: ret = usbcdc_vprintf(format, ap); break;
    case PICO_SHELL_SERIAL0: ret = serial0_vprintf(format, ap); break;
    case PICO_SHELL_SERIAL1: ret = serial1_vprintf(format, ap); break;
    default: break;
    }
    va_end(ap);

    return ret;
}

int PicoShell::vprintf(const char *format, va_list ap)
{
    int ret = 0;

    switch (_device) {
    case PICO_SHELL_USB_CDC: ret = usbcdc_vprintf(format, ap); break;
    case PICO_SHELL_SERIAL0: ret = serial0_vprintf(format, ap); break;
    case PICO_SHELL_SERIAL1: ret = serial1_vprintf(format, ap); break;
    default: break;
    }

    return ret;
}

int PicoShell::rx_ready(void) const
{
    int ret = 0;

    switch (_device) {
    case PICO_SHELL_USB_CDC: ret = usbcdc_rx_ready(); break;
    case PICO_SHELL_SERIAL0: ret = serial0_rx_ready(); break;
    case PICO_SHELL_SERIAL1: ret = serial1_rx_ready(); break;
    default: break;
    }

    return ret;
}

int PicoShell::rx_read(uint8_t *buf, size_t size)
{
    int ret = 0;

    switch (_device) {
    case PICO_SHELL_USB_CDC: ret = usbcdc_read(buf, size); break;
    case PICO_SHELL_SERIAL0: ret = serial0_read(buf, size); break;
    case PICO_SHELL_SERIAL1: ret = serial1_read(buf, size); break;
    default: break;
    }

    return ret;
}

bool PicoShell::catch_ctr_c(bool untilFound)
{
    bool result = false;

    do {
        int ret;
        char c;

        ret = this->rx_read((uint8_t *) &c, 1);
        if (ret == -1) {
            break;
        }

        if (c == 0xff) {  // IAC received
            static const uint8_t iac_do_tm[3] = { 0xff, 0xfd, 0x06, };
            static const uint8_t iac_will_tm[3] = { 0xff, 0xfb, 0x06, };
            char iac2;

            ret = this->rx_read((uint8_t *) &iac2, 1);
            if (ret == 1) {
                switch (iac2) {
                case 0xf4:  // IAC IP (interrupt process)
                    ret = this->tx_write(iac_do_tm, sizeof(iac_do_tm));
                    ret = this->tx_write(iac_will_tm, sizeof(iac_will_tm));
                    this->printf("\n> ");
                    _inproc.i = 0;
                    result = true;
                    break;
                default:
                    break;
                }
            }

            break;
        }

        if (c == '\x03') {
            result = true;
            break;
        }
    } while (untilFound);

    return result;
}

/*
 * Local variables:
 * mode: C++
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
