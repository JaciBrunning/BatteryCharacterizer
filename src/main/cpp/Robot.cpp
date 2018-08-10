#include <TimedRobot.h>
#include <SpeedController.h>
#include <PowerDistributionPanel.h>

#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <memory>
#include <chrono>

static double time_ms() {
  auto now = std::chrono::system_clock::now();
  double now_ms = static_cast<double>(std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count());
  return now_ms;
}

struct measurement {
  double voltage, current;
  measurement(double v, double c) : voltage(v), current(c) {}
};

class Robot : public frc::TimedRobot {
 public:
  Robot() : _pdp(0) { }

  void RobotInit() override {
    
  }

  void AutonomousInit() override {
    _state = 0;
    _transition_ms = time_ms();
  }
  
  void Sample() {
    _measures.emplace_back(_pdp.GetVoltage(), _pdp.GetTotalCurrent());
  }

  void AutonomousPeriodic() override {
    for (auto it = _motors.cbegin(); it < _motors.cend(); it++) {
      (*it)->Set(_state >= 11 ? 0 : (_state * _step));
    }

    if (time_ms() - _transition_ms > 1000) {
      if (_state < 11) {
        Sample();
        _transition_ms = time_ms();
        _state++;
      } else if (_state == 11) {
        // Report
        int n = _measures.size(), i = 0;
        Eigen::VectorXf vs(n), cs(n);

        for (auto measure = _measures.cbegin(); measure < _measures.cend(); measure++) {
          vs(i) = measure->voltage;
          cs(i) = measure->current;
          i++;
        }

        Eigen::VectorXf vavg(n), cavg(n);
        vavg.fill(vs.mean());
        cavg.fill(cs.mean());

        Eigen::VectorXf vvar = (vs - vavg), cvar = (cs - cavg);

        double numerator = (vvar*cvar).sum();
        double denominator = sqrt((vvar*vvar).sum()) * sqrt((cvar*cvar).sum());

        double correlation = numerator / denominator;
        std::cout << "Correlation: " << correlation << std::endl;

        _state++;
      }
    }
  }

 private:
  std::vector<std::shared_ptr<SpeedController> > _motors;
  std::vector<measurement> _measures;
  PowerDistributionPanel _pdp;
  int _state = 0;
  double _step = 0.1;
  double _transition_ms = 0;
};


#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif
