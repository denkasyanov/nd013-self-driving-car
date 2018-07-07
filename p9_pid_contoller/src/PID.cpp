#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

// parameters variables changed for readability
void PID::Init(double Kp_in, double Ki_in, double Kd_in) {

  Kp = Kp_in;
  Ki = Ki_in;
  Kd = Kd_in;

  p_error = .0;
  i_error = .0;
  d_error = .0;

  cte_prev = .0;
}

void PID::UpdateError(double cte_in) {
  p_error = cte_in;
  d_error = cte_in - cte_prev;
  i_error = i_error + cte_in;
}



double PID::TotalError() {

}

