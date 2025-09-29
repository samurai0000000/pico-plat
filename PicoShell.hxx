/*
 * PicoShell.hxx
 *
 * Copyright (C) 2025, Charles Chiou
 */

#ifndef PICOSHELL_HXX
#define PICOSHELL_HXX

#include <string>
#include <memory>
#include <vector>

using namespace std;

enum PicoShellDevice {
    PICO_SHELL_USB_CDC,
    PICO_SHELL_SERIAL0,
    PICO_SHELL_SERIAL1,
};

class PicoShell {

public:

    PicoShell(enum PicoShellDevice device);
    ~PicoShell();

    inline void setBanner(const string &banner) {
        _banner = banner;
    }
    inline void setVersion(const string &version) {
        _version = version;
    }
    inline void setBuilt(const string &built) {
        _built = built;
    }
    inline void setCopyright(const string &copyright) {
        _copyright = copyright;
    }

    inline const string &banner(void) const {
        return _banner;
    }
    inline const string &version(void) const {
        return _version;
    }
    inline const string &built(void) const {
        return _built;
    }
    inline const string &copyright(void) const {
        return _copyright;
    }

    inline void setNoEcho(bool noEcho) {
        _noEcho = noEcho;
    }

    inline virtual void showWelcome(void) {
        this->printf("\n\x1b[2K");
        this->printf("%s\n", _banner.c_str());
        this->printf("%s\n", _version.c_str());
        this->printf("%s\n", _built.c_str());
        this->printf("-------------------------------------------\n");
        this->printf("%s\n", _copyright.c_str());
        this->printf("> ");
    }

    virtual int process(void);

protected:

    virtual int tx_write(const uint8_t *buf, size_t size);
    virtual int printf(const char *format, ...);
    virtual int vprintf(const char *format, va_list ap);
    virtual int rx_ready(void) const;
    virtual int rx_read(uint8_t *buf, size_t size);
    virtual bool catch_ctr_c(bool untilFound = true);

    virtual int exec(char *cmdline);
    virtual int help(int argc, char **argv);
    virtual int version(int argc, char **argv);
    virtual int system(int argc, char **argv);
    virtual int reboot(int argc, char **argv);
    virtual int bootsel(int argc, char **argv);
    virtual int unknown_command(int argc, char **argv);

    time_t _since;

    string _banner;
    string _version;
    string _built;
    string _copyright;

    vector<string> _help_list;

protected:

#define CMDLINE_SIZE 256


    enum PicoShellDevice _device;
    bool _noEcho;

    struct inproc {
        char cmdline[CMDLINE_SIZE];
        unsigned int i;
    };

    struct inproc _inproc;

};

#endif

/*
 * Local variables:
 * mode: C++
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
