#include <gpiod.h>
#include <iostream>

int main() {
    // Open GPIO chip
    gpiod_chip* chip = gpiod_chip_open_by_name("gpiochip0");

    // Get GPIO lines
    gpiod_line* LApin1 = gpiod_chip_get_line(chip, 17);  //LA = linear actuator pin
    gpiod_line* LApin2 = gpiod_chip_get_line(chip, 18);

    // Set pins as outputs, start LOW
    gpiod_line_request_output(LApin1, "linear_actuator", 0);
    gpiod_line_request_output(LApin2, "linear_actuator", 0);

    int input;

    std::cout << "Enter 1 to extend, 2 to retract, 0 to stop:\n";

    while (true) {
        std::cin >> input;

        if (input == 1) {
            // Extend
            gpiod_line_set_value(LApin1, 1);
            gpiod_line_set_value(LApin2, 1);
        }
        else if (input == 2) {
            // Retract
            gpiod_line_set_value(LApin1, 0);
            gpiod_line_set_value(LApin2, 0);
        }
        else if (input == 0) {
            // Stop
            gpiod_line_set_value(LApin1, 0);
            gpiod_line_set_value(LApin2, 0);
        }
    }
}
