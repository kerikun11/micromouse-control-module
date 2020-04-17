#include <fstream>

#include "accel_designer.h"
#include "feedback_controller.h"

std::ofstream of("main.csv");

int main(void) {
  /* Feedforward Model and Feedback Gain */
  ctrl::FeedbackController<float>::Model model = {.K1 = 1, .T1 = 0};
  ctrl::FeedbackController<float>::Gain gain = {.Kp = 0, .Ki = 0, .Kd = 0};
  /* Controller */
  ctrl::FeedbackController<float> feedback_controller(model, gain);
  /* Trajectory */
  ctrl::AccelDesigner trajectory(240, 9, 0, 1.2, 0, 0.72);
  std::cout << trajectory << std::endl;
  /* Control Period [s] */
  const float Ts = 0.001f;
  /* Control */
  feedback_controller.reset();
  for (float t = 0; t < trajectory.t_end(); t += Ts) {
    const float r = trajectory.v(t);  //< reference of output
    const float y = trajectory.v(t);  //< measurement output
    const float dr = trajectory.a(t); //< differential of reference of output
    const float dy = trajectory.a(t); //< differential of measurement output
    const float u = feedback_controller.update(r, y, dr, dy, Ts);
    /* apply control input u here */
    of << t;
    of << "," << u;
    of << std::endl;
  }

  return 0;
}
