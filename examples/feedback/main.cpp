#include <fstream>

#include <ctrl/accel_designer.h>
#include <ctrl/feedback_controller.h>

std::ofstream csv("main.csv");

int main(void) {
  /* Feedforward Model and Feedback Gain */
  ctrl::FeedbackController<float>::Model model = {.K1 = 1, .T1 = 0};
  ctrl::FeedbackController<float>::Gain gain = {.Kp = 0, .Ki = 0, .Kd = 0};
  /* Controller */
  ctrl::FeedbackController<float> feedback_controller(model, gain);
  /* Trajectory */
  ctrl::AccelDesigner trajectory(240, 9, 1.2, 0, 0, 0.72);
  std::cout << trajectory << std::endl;
  /* Control Period [s] */
  const float Ts = 1e-3f;
  /* Control */
  feedback_controller.reset();
  for (float t = 0; t < trajectory.t_end(); t += Ts) {
    const float r = trajectory.v(t);  //< reference of output
    const float y = trajectory.v(t);  //< measurement output
    const float dr = trajectory.a(t); //< differential of reference of output
    const float dy = trajectory.a(t); //< differential of measurement output
    const float u = feedback_controller.update(r, y, dr, dy, Ts);
    /* apply control input u here */
    /* csv output */
    const auto bd = feedback_controller.getBreakdown();
    csv << t;
    csv << "," << r;
    csv << "," << y;
    csv << "," << dr;
    csv << "," << dy;
    csv << "," << u;
    csv << "," << bd.ff;
    csv << "," << bd.fb;
    csv << "," << bd.fbp;
    csv << "," << bd.fbi;
    csv << "," << bd.fbd;
    csv << std::endl;
  }

  return 0;
}
