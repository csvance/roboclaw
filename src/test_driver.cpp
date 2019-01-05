//
// Created by Carroll Vance on 2019-01-04.
//

#include <iostream>
#include <string>
#include <time.h>
#include "roboclaw_driver.h"

using namespace roboclaw;
using namespace std;

int main() {
    driver *roboclaw = new driver(std::string("/dev/tty.usbserial-FTZ8B103"));
    roboclaw->set_baud(115200);

    roboclaw->set_velocity(driver::BASE_ADDRESS, std::pair<int, int>(5000, 5000));

    for(int i=0;i<10;i++){
        std::pair<int, int> enc = roboclaw->get_encoders(driver::BASE_ADDRESS);

        cout << i << ": " << enc.first << " " << enc.second << endl;

        sleep(1);

    }

    roboclaw->set_duty(driver::BASE_ADDRESS, std::pair<int, int>(0, 0));

    return 0;
}