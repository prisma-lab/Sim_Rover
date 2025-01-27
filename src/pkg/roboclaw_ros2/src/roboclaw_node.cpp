/**
*
* Copyright (c) 2018 Carroll Vance.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
        * the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
        * DEALINGS IN THE SOFTWARE.
*/

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include "roboclaw_roscore.h"



int main(int argc, char **argv) {
    rclcpp::init(argc, argv ) ;

   // roboclaw::roboclaw_roscore node(nh, nh_private);
   // node.run();
    try {
    std::shared_ptr<roboclaw::roboclaw_roscore> rover_node = std::make_shared<roboclaw::roboclaw_roscore>();
 
    // Codice che potrebbe generare un'eccezione std::runtime_error
    //node.executor.add_node(rover_node);

    //rover_node->run(rover_node);
    rclcpp::spin(rover_node);


    } catch (const std::runtime_error& e) {
        // Gestione dell'eccezione
        std::cerr << "Errore durante l'esecuzione del programma: " << e.what() << std::endl;
        // Altre operazioni di recupero o terminazione del programma
        return EXIT_FAILURE; // Ritorna un codice di uscita che indica un fallimento
    }
    return 0;
}

