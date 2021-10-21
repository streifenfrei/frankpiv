#ifndef FRANKPIV_EXCEPTIONS
#define FRANKPIV_EXCEPTIONS

namespace frankpiv::exceptions {
    class ConfigError : public std::runtime_error {
        std::string message;
    public:
        explicit ConfigError(const std::string &message) : runtime_error(message) {
            this->message = message;
        }

        const char *what() {
            return this->message.c_str();
        }
    };

    class UnattainablePoseException : public std::runtime_error {
        std::string message;
        const Eigen::Vector2d *boundaries;
        const double *value;
    public:
        explicit UnattainablePoseException(const std::string &message, const Eigen::Vector2d *boundaries = nullptr, const double *value = nullptr) : runtime_error(message) {
            this->message = message;
            this->boundaries = boundaries;
            this->value = value;
        }

        const char *what() {
            std::stringstream what;
            what << this->message;
            if (this->boundaries) {
                what << "\nboundaries: " << this->boundaries;
            }
            if (this->value) {
                what << "\nvalue: " << this->value;
            }
            char what_char = *what.str().c_str();
            const char *what_char_pointer = &what_char;
            return what_char_pointer;
        }
    };

    class CriticalPivotErrorException : public std::runtime_error {
        double actual_value;
        double max_value;
    public:
        explicit CriticalPivotErrorException(double actual_value, double max_value) : runtime_error("") {
            this->actual_value = actual_value;
            this->max_value = max_value;
        }

        const char *what() {
            std::stringstream what;
            what << "Pivot error exceeded specified limit: " << this->actual_value << " > " << this->max_value;
            char what_char = *what.str().c_str();
            const char *what_char_pointer = &what_char;
            return what_char_pointer;
        }
    };
}
#endif //FRANKPIV_EXCEPTIONS
