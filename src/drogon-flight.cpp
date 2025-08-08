/*
 * drogon-flight.cpp
 * 
 * This file is part of Drogon.
 *
 * Drogon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Drogon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Drogon.  If not, see <http://www.gnu.org/licenses/>. 
 *
 * Author: Joseph Monti <joe.monti@gmail.com>
 * Copyright (c) 2013 Joseph Monti All Rights Reserved, http://joemonti.org/
 */

#include "DrogonFlight.h"
#include "RCoreClient.h"

#include <signal.h>
#include <cstdlib>

DrogonFlight* drogon_flight = NULL;

void my_handler(int s) 
{
    printf("Caught signal %d, exiting\n",s);
    if (drogon_flight != NULL) {
        delete drogon_flight;
    }
    exit(1);
}

int main()
{
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    drogon_flight = new DrogonFlight();
    
    drogon_flight->run_debug();

    printf("FINISHED\n");
    return 0;
}
