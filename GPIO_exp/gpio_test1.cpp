 #include <gpiod.h>
#include <iostream>
#include <unistd.h>


#define GPIO_CHIP "gpiochip4"
#define GPIO_LINE 17   // BCM GPIO17 (pin 11)


int main() {
   gpiod_chip* chip = nullptr;
   gpiod_line* line = nullptr;
   int ret;


   chip = gpiod_chip_open_by_name(GPIO_CHIP);
   if (!chip) {
       perror("Open chip failed");
       return 1;
   }


   line = gpiod_chip_get_line(chip, GPIO_LINE);
   if (!line) {
       perror("Get line failed");
       gpiod_chip_close(chip);
       return 1;
   }


   ret = gpiod_line_request_output(line, "gpio-out", 0);
   if (ret < 0) {
       perror("Request line as output failed");
       gpiod_chip_close(chip);
       return 1;
   }


   std::cout << "Toggling GPIO " << GPIO_LINE << "...\n";


   for (int i = 0; i < 10; i++) {
       gpiod_line_set_value(line, 1);
       sleep(1);
       gpiod_line_set_value(line, 0);
       sleep(1);
   }


   gpiod_line_release(line);
   gpiod_chip_close(chip);


   return 0;
}

