#include <gpiod.h>
#include <iostream>

int main() {
    // Open GPIO chip
    gpiod_chip* chip = gpiod_chip_open_by_name("gpiochip4");
    if (!chip) {
        std::cerr << "Failed to open chip\n";
        return 1;
    }
    
    // Get GPIO lines
    gpiod_line* LApin = gpiod_chip_get_line(chip, 17);  //LA = linear actuator pin
    gpiod_line* Power_12Vpin = gpiod_chip_get_line(chip, 10);
    if (!LApin || !Power_12Vpin) {
        std::cerr << "Failed to get GPIO line\n";
        gpiod_chip_close(chip);
        return 1;
    }
    
    // Set pins as outputs, start LOW
    gpiod_line_request_output(LApin, "linear_actuator", 0);
    gpiod_line_request_output(Power_12Vpin, "linear_actuator", 0);

    int input;

    std::cout << "Enter 0 to exit program, 1 to extend, 2 to retract, 3 to cut power (stop), 4 to power on:\n";
    
    input = 3; //initialization
    while (input != 0) {
        std::cin >> input;

        if (input == 1) {
            // Extend
            gpiod_line_set_value(LApin, 1);
        }
        else if (input == 2) {
            // Retract
            gpiod_line_set_value(LApin, 0);
        }
        else if (input == 3) {
            // cut power
            gpiod_line_set_value(Power_12Vpin, 0);
        } 
        else if (input == 4) {
            // power on
            gpiod_line_set_value(Power_12Vpin, 1);
        }
    }
    // exit program
    gpiod_line_set_value(Power_12Vpin, 0);

    gpiod_line_release(LApin);
    gpiod_line_release(Power_12Vpin);
    gpiod_chip_close(chip);
}
