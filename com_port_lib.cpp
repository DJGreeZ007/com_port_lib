//
// Created by djgreez on 26.07.2023.
//
#include "com_port_lib.h"

using namespace cpl;

/* C++ libraries */
#include <filesystem>
#include <algorithm>

/* Linux version */
#ifdef __linux__
/* C linux libraries */
#include <fcntl.h>
#include <unistd.h>
#endif

/* Сonstructors and destructor. */
com_port_lib::~com_port_lib() {
    close_port();
}

/* Interface. */
std::pair<com_port_lib::cperr, int> com_port_lib::open_port(const std::string &_port_name, const size_t& _speed) {

    /** Storing the return value in c linux library functions. */
    int error_code{};

    /* Conversion and transmission rate verification. */
    /** Data transfer rate. */
    speed_t speed;
    {
        bool error_flag{};
        speed = port_speed_conversion(_speed, error_flag);
        if (error_flag) {
            return std::make_pair(cperr::ERROR_TRANSFER_RATE, int{});
        }
    }

    /*
     * Open the serial port. Change device path as needed.
     * (currently set to an standard FTDI USB-UART cable type device.)
     */
    /** COM port name with path. */
    const std::string port_location{"/dev/" + _port_name};

    serial_port = open(port_location.c_str(), O_RDWR);
    if(serial_port == -1) {
        /* Port opening error. */
        return std::make_pair(cperr::ERROR_OPENING_FILE_DESCRIPTOR, errno);
    }

    /** Call it 'tty' for convention. */
    termios tty;

    /* Read in existing settings, and handle any error. */
    error_code = tcgetattr(serial_port, &tty);
    if(error_code != 0) {
        /* Closing file descriptor. Even if there is an error in closing. */
        close_port();
        /* Error loading tty structures. */
        return std::make_pair(cperr::ERROR_LOADING_TTY_STRUCTURES, errno);
    }

    /* Setting tty flags. */
    setting_tty_flags(tty);

    /* Set in/out baud rate. */
    cfsetispeed(&tty, speed); /* Input. */
    cfsetospeed(&tty, speed); /* Output. */

    /* Save tty settings, also checking for error. */
    error_code = tcsetattr(serial_port, TCSANOW, &tty);
    if (error_code != 0) {
        /* Closing file descriptor. Even if there is an error in closing. */
        close_port();
        /* Error saving tty settings. */
        return std::make_pair(cperr::ERROR_SAVING_TTY_SETTINGS, errno);
    }

    /* Clear IO buffer. */
    error_code = tcflush(serial_port, TCIOFLUSH);
    if (error_code == -1) {
        /* Closing file descriptor. Even if there is an error in closing. */
        close_port();
        /* Error clear IO buffer. */
        return std::make_pair(cperr::ERROR_CLEAR_IO_BUFFER, errno);
    }

    /* No errors */
    return std::make_pair(cperr::SUCCESSFULLY, int{});
}

std::vector<std::string> com_port_lib::get_list_of_serial_ports() {

    /** The path in linux to devices */
    const std::string inputPath = "/dev";

    std::vector<std::string> result{};

    for (const auto & _iter : std::filesystem::directory_iterator(inputPath)) {
        std::string dev_name = _iter.path().filename();

        /* Filtering of found devices */

        /** Com port filter */
        std::string filter = "ttyUSB";

        if (dev_name.substr(0, filter.size()) == filter) {
            result.push_back(dev_name);
        }
    }
    return result;
}

std::pair<com_port_lib::cperr, int> com_port_lib::close_port() {
    /** Storing the return value in c linux library functions. */
    int error_code{};

    if (serial_port != -1) {

        /* Port closure. */
        error_code = close(serial_port);
        if (error_code == -1) {
            /* Error closing file descriptor. */
            return std::make_pair(cperr::ERROR_CLOSING_FILE_DESCRIPTOR, errno);
        }
    }

    /* No errors */
    serial_port = -1;
    return std::make_pair(cperr::SUCCESSFULLY, int{});
}

std::pair<com_port_lib::cperr, int> com_port_lib::send_message(const std::vector<std::uint8_t> &_data) {
    /** Storing the return value in c linux library functions. */
    int error_code{};

    if(serial_port == -1) {
        return std::make_pair(cperr::ERROR_FILE_DESCRIPTOR_IS_NOT_OPEN, int{});
    }
    /** The number of bytes written to the serial port. */
    int number_of_bytes_written = write(serial_port, _data.data(), _data.size());

    /* Checking the sending of the message. */
    if (number_of_bytes_written == -1) {
        /* Сhecking for the I/O error, if it is, then close the port. */
        if (errno == 5) { /* 5 - I/O error in errno */
            close_port();
        }
        /* Error sending the message. */
        return std::make_pair(cperr::ERROR_SENDING_MESSAGE, errno);
    }

    if (number_of_bytes_written != _data.size()) {
        /* Error sending the message. */
        return std::make_pair(cperr::INCOMPLETE_SENDING_OF_MESSAGE, errno);
    }

//    /* Synchronizes the state of the file in memory with the state on disk. */
//    error_code = fsync(serial_port);
//    if (error_code == -1) {
//        /* Error sending the message. */
//        return std::make_pair(cperr::ERROR_SENDING_MESSAGE, errno);
//    }

    /* No errors */
    return std::make_pair(cperr::SUCCESSFULLY, int{});
}

std::pair<com_port_lib::cperr, int>
com_port_lib::try_receiving_message(std::vector<std::uint8_t> &_out_data, const size_t &_number_of_bytes) {
    /** Storing the return value in c linux library functions. */
    int error_code{};

    if(serial_port == -1) {
        return std::make_pair(cperr::ERROR_FILE_DESCRIPTOR_IS_NOT_OPEN, int{});
    }
    /** Storage of read data. */
    std::vector<uint8_t> buffer(_number_of_bytes);

    /** The number of bytes written to the serial port. */
    int number_of_bytes_read = read(serial_port, buffer.data(), _number_of_bytes);

    /* Checking the sending of the message. */
    if (number_of_bytes_read == -1) {
        /* Сhecking for the I/O error, if it is, then close the port. */
        if (errno == 5) { /* 5 - I/O error in errno */
            close_port();
        }
        /* Error sending the message. */
        return std::make_pair(cperr::ERROR_READING_MESSAGE, errno);
    }
    /* Copying read bytes. */
    std::copy(std::begin(buffer), std::next(std::begin(buffer), number_of_bytes_read), std::back_inserter(_out_data));

    return std::make_pair(cperr::SUCCESSFULLY, int{});
}


/* Auxiliary functions. */

speed_t com_port_lib::port_speed_conversion (size_t _speed, bool& _out_error) {
    speed_t bit_meaning;
    _out_error = false;
    switch(_speed) {
        case 0:
            bit_meaning = B0;
            break;
        case 50:
            bit_meaning = B50;
            break;
        case 75:
            bit_meaning = B75;
            break;
        case 110:
            bit_meaning = B110;
            break;
        case 134:
            bit_meaning = B134;
            break;
        case 150:
            bit_meaning = B150;
            break;
        case 200:
            bit_meaning = B200;
            break;
        case 300:
            bit_meaning = B300;
            break;
        case 600:
            bit_meaning = B600;
            break;
        case 1200:
            bit_meaning = B1200;
            break;
        case 1800:
            bit_meaning = B1800;
            break;
        case 2400:
            bit_meaning = B2400;
            break;
        case 4800:
            bit_meaning = B4800;
            break;
        case 9600:
            bit_meaning = B9600;
            break;
        case 19200:
            bit_meaning = B19200;
            break;
        case 38400:
            bit_meaning = B38400;
            break;
        default:
            bit_meaning = B0;
            _out_error = true;
    }
    return bit_meaning;
}

void com_port_lib::setting_tty_flags(termios& _tty) {
    /* Setting С flags */
    _tty.c_cflag &= ~PARENB;         /* Clear parity bit, disabling parity (most common). */
    _tty.c_cflag &= ~CSTOPB;         /* Clear stop field, only one stop bit used in communication (most common). */
    _tty.c_cflag &= ~CSIZE;          /* Clear all bits that set the data size. */
    _tty.c_cflag |= CS8;             /* 8 bits per byte (most common) */
    _tty.c_cflag &= ~CRTSCTS;        /* Disable RTS/CTS hardware flow control (most common). */
    _tty.c_cflag |= CREAD | CLOCAL;  /* Turn on READ & ignore ctrl lines (CLOCAL = 1). */

    _tty.c_lflag &= ~ICANON;
    _tty.c_lflag &= ~ECHO;           /* Disable echo. */
    _tty.c_lflag &= ~ECHOE;          /* Disable erasure. */
    _tty.c_lflag &= ~ECHONL;         /* Disable new-line echo. */
    _tty.c_lflag &= ~ISIG;           /* Disable interpretation of INTR, QUIT and SUSP. */
    _tty.c_iflag &= ~(IXON | IXOFF | IXANY); /* Turn off s/w flow ctrl. */
    /* Disable any special handling of received bytes. */
    _tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    _tty.c_oflag &= ~OPOST;      /* Prevent special interpretation of output bytes (e.g. newline chars). */
    _tty.c_oflag &= ~ONLCR;      /* Prevent conversion of newline to carriage return/line feed. */
    //tty.c_oflag &= ~OXTABS;      /* Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX). */
    //tty.c_oflag &= ~ONOEOT;      /* Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX). */

    _tty.c_cc[VTIME] = 10;       /* Wait for up to 1s (10 deciseconds), returning as soon as any data is received. */
    _tty.c_cc[VMIN] = 0;
}
