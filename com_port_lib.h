//
// Created by djgreez on 26.07.2023.
//

#ifndef COM_PORT_LIB_H
#define COM_PORT_LIB_H



/* C++ libraries */
#include <string>
#include <vector>

/* Linux version */
#ifdef __linux__
/* C linux libraries */
#include <termios.h> /* Contains POSIX terminal control definitions */

#endif /* #ifdef __linux__ */

namespace cpl { /* Namespace COM port library */
    class com_port_lib {
    public:
        /**
         * ---- Destructor ----
         * @brief Destructor, closes the file descriptor.
         */
        ~com_port_lib();

        /* Errors */
        #ifdef __linux__

            /**
             * ---- cperr ----
             * Errors in the linux.
             * The type of enumeration.
             * Enumeration of errors that methods can return.
             */
            enum cperr {
                SUCCESSFULLY = 0,                     /**< No errors */
                ERROR_OPENING_FILE_DESCRIPTOR = 1,    /**< Port opening error */
                ERROR_LOADING_TTY_STRUCTURES = 2,     /**< Error loading tty structures. */
                ERROR_TRANSFER_RATE = 3,              /**< Error transfer rate. */
                ERROR_SAVING_TTY_SETTINGS = 4,        /**< Error saving tty settings. */
                ERROR_CLEAR_IO_BUFFER = 5,            /**< Error clear IO buffer. */
                ERROR_CLOSING_FILE_DESCRIPTOR = 6,    /**< Error closing file descriptor. */
                ERROR_FILE_DESCRIPTOR_IS_NOT_OPEN = 7,/**< The file descriptor is not open. */
                ERROR_SENDING_MESSAGE = 8,            /**< Error sending the message. */
                INCOMPLETE_SENDING_OF_MESSAGE = 9,    /**< Incomplete sending of a message. */
                ERROR_READING_MESSAGE = 10,           /**< Error reading the message. */
            }; /* enum cpeer */

        #endif /* __linux__ */

        /* Interface */

        /**
         * ---- open_port ----
         * @brief The function initializes the serial port by its name.
         *
         * @param _port_name Port name, for example "ttyUSB0".
         * @param _speed The transfer rate for the selected port in bots.
         *
         * @return A couple of values.
         * The first is at what point in the execution of the function an error occurred, type cperr.
         * The second is the errno error code.
         *
         * @note If the first parameter is com_port_lib::cperr::SUCCESSFULLY,
         * the second parameter is not considered.
         *
         * @see com_port_lib::cperr
         * @see com_port_lib::cperr::SUCCESSFULLY
         */
        std::pair<cperr, int> open_port(const std::string &_port_name, const size_t& _speed);

        /**
         * ---- get_list_of_serial_ports ----
         * @brief The function searches for available serial ports.
         *
         * @return List of available port names.
         */
        std::vector<std::string> get_list_of_serial_ports();

        /**
         * ---- close_port ----
         * @brief The function closes the port.
         *
         * @return A couple of values.
         * The first is at what point in the execution of the function an error occurred, type cperr.
         * The second is the errno error code.
         *
         * @note If the port was not open nothing happens.
         */
        std::pair<cperr, int> close_port();

        /**
         * ---- send_message ----
         * @brief The function sends a message to the serial port.
         *
         * @param _data Data to send.
         *
         * @return A couple of values.
         * The first is at what point in the execution of the function an error occurred, type cperr.
         * The second is the errno error code.
         */
        std::pair<cperr, int> send_message(const std::vector<std::uint8_t>& _data);

        /**
         * ---- send_message ----
         * @brief The function tries to read the specified number of bytes.
         *
         * @param _out_data Read bytes.
         * @param _number_of_bytes How many bytes should I try to count.
         *
         * @return A couple of values.
         * The first is at what point in the execution of the function an error occurred, type cperr.
         * The second is the errno error code.
         *
         * @note The transmitted vector is not changed, and the data is added to the end.
         */
        std::pair<cperr, int> try_receiving_message(std::vector<std::uint8_t>& _out_data, const size_t & _number_of_bytes);

    private:
        /* Auxiliary functions */

        /**
         * ---- port_speed_conversion ----
         * @brief Converts an integer speed value to a constant for the termios structure.
         *
         * @param _speed Integer transfer rate
         * @param _out_error Error flag. true - error.
         *
         * @return Speed of type speed_t
         */
        speed_t port_speed_conversion (size_t _speed, bool& _out_error);

        /**
         * ---- setting_tty_flags ----
         * @brief Sets the tty flags needed for the com port.
         *
         * @param _tty The termios object for which flags will be set.
         */
        void setting_tty_flags(termios& _tty);

        int serial_port{-1}; /** Port descriptor */
    }; /* class com_port_lib */

#endif //COM_PORT_LIB_H

} /* namespace cpl */
