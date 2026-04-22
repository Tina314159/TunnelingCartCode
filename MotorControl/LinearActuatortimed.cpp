#include <gpiod.h>
#include <iostream>
#include <unistd.h>

int main() {
    // Open GPIO chip
    gpiod_chip* chip = gpiod_chip_open_by_name("gpiochip4");
    if (!chip) {
        std::cerr << "Failed to open chip\n";
        return 1;
    }

    // Get GPIO lines
    gpiod_line* LApin = gpiod_chip_get_line(chip, 17);   // direction pin
    gpiod_line* Power_12Vpin = gpiod_chip_get_line(chip, 10); // power pin
    if (!LApin || !Power_12Vpin) {
        std::cerr << "Failed to get GPIO line\n";
        gpiod_chip_close(chip);
        return 1;
    }

    // Request outputs
    if (gpiod_line_request_output(Power_12Vpin, "linear_actuator", 0) < 0 ||
        gpiod_line_request_output(LApin, "linear_actuator", 0) < 0) {
        std::cerr << "Failed to request GPIO outputs\n";
        gpiod_chip_close(chip);
        return 1;
    }

    const double FULL_STROKE_INCHES = 4.0;
    const double FULL_STROKE_TIME_SEC = 10.0;   // you can change to 9.5 if that is more accurate
    const useconds_t SWITCH_DELAY_US = 100000;  // 100 ms safety delay

    int input = -1;
    double inches = 0.0;
    double run_time_sec = 0.0;

    std::cout << "---- Linear Actuator Control Program ----\n";
    std::cout << "Enter:\n";
    std::cout << "0 = exit\n";
    std::cout << "1 = extend by chosen inches\n";
    std::cout << "2 = retract by chosen inches\n";
    std::cout << "3 = cut power (stop)\n";

    while (input != 0) {
        std::cout << "\nCommand: ";
        if (!(std::cin >> input)) {
            std::cerr << "Invalid input\n";
            break;
        }

        if (input == 1 || input == 2) {
            std::cout << "Enter distance in inches (0 to 4): ";
            if (!(std::cin >> inches)) {
                std::cerr << "Invalid distance input\n";
                break;
            }

            if (inches < 0.0 || inches > FULL_STROKE_INCHES) {
                std::cout << "Distance must be between 0 and 4 inches.\n";
                continue;
            }

            run_time_sec = (inches / FULL_STROKE_INCHES) * FULL_STROKE_TIME_SEC;

            // Cut power before switching direction
            gpiod_line_set_value(Power_12Vpin, 0);
            usleep(SWITCH_DELAY_US);

            if (input == 1) {
                std::cout << "Extending " << inches << " inches for "
                          << run_time_sec << " seconds\n";
                gpiod_line_set_value(LApin, 0);   // extend direction
            } else {
                std::cout << "Retracting " << inches << " inches for "
                          << run_time_sec << " seconds\n";
                gpiod_line_set_value(LApin, 1);   // retract direction
            }

            // Turn on power, wait, then stop
            gpiod_line_set_value(Power_12Vpin, 1);
            usleep((useconds_t)(run_time_sec * 1000000));
            gpiod_line_set_value(Power_12Vpin, 0);

            std::cout << "Motion complete, power cut.\n";
        }
        else if (input == 3) {
            std::cout << "Cut 12V Power\n";
            gpiod_line_set_value(Power_12Vpin, 0);
        }
        else if (input == 0) {
            break;
        }
        else {
            std::cout << "Unknown command\n";
        }
    }

    // Safe shutdown
    gpiod_line_set_value(Power_12Vpin, 0);
    gpiod_line_set_value(LApin, 1);

    gpiod_line_release(LApin);
    gpiod_line_release(Power_12Vpin);
    gpiod_chip_close(chip);

    return 0;
}
