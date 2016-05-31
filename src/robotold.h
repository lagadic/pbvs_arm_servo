
class VispNaoqiRobot
{
public:

  VispNaoqiRobot();

  void readJoints();

  void writeJoints();

private:

  //int joint_mode;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];

};
