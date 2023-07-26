#include "../include/interfaces/float_standard.hpp"

/** @author Natalie Chmura
 * 
 * @brief This contains a function that converts each floating point decimal to a nanometer
 * (1 out of 10000000) scale! This is deemed nessesary to offset reach's use of full float for
 * tranlating the quaternion into an Isometry3D and back- thus a nanometer was deemed
 * a sufficent enough scale to round to!
 **/

namespace FloatSt {

    // PUBLIC:

    double RoundSt::roundNano(double number) {

        long intermediate = round(number * 10000000);
        double calc = intermediate / 10000000.00;
        return calc;
    }

} //namespace FloatSt