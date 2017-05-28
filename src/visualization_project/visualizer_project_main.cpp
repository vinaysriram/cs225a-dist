#include <model/ModelInterface.h>
#include <simulation/SimulationInterface.h>
#include <graphics/GraphicsInterface.h>
#include <graphics/ChaiGraphics.h>
#include "redis/RedisClient.h"
#include <GLFW/glfw3.h> 
#include "uiforce/UIForceWidget.h"
#include <iostream>
#include <string>
#include <cmath>

using namespace std;

static string world_file = "";
static string robot_file = "";
static string robot_name = "";

static const string CAMERA_NAME       = "camera_fixed";
static const string REDIS_KEY_PREFIX  = "cs225a::robot::";
static string JOINT_ANGLES_KEY        = "::sensors::q";
static string JOINT_VELOCITIES_KEY    = "::sensors::dq";
static string TARGET_POSITION_KEY     = "::tasks::tg_pos";
static string J5_POSITION_KEY         = "::tasks::j5_pos";
static string J6_POSITION_KEY         = "::tasks::j6_pos";
static string ROT_5_X                 = "::tasks::rot_5x";
static string ROT_5_Y                 = "::tasks::rot_5y";
static string ROT_5_Z                 = "::tasks::rot_5z";
static string ROT_6_X                 = "::tasks::rot_6x";
static string ROT_6_Y                 = "::tasks::rot_6y";
static string ROT_6_Z                 = "::tasks::rot_6z";

static void parseCommandline(int argc, char** argv);
static void glfwError(int error, const char* description);
static void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);
static void mouseClick(GLFWwindow* window, int button, int action, int mods);
static void mouseScroll(GLFWwindow* window, double xoffset, double yoffset);
static chai3d::cGenericObject *findObjectInWorld(chai3d::cWorld *world, const string &graphics_name);

static bool fTransXp = false;
static bool fTransXn = false;
static bool fTransYp = false;
static bool fTransYn = false;
static bool fRotPanTilt = false;
static bool fZoom = false;
static double zoomSpeed = 0.0;
static bool fRobotLinkSelect = false;

static const HiredisServerInfo kRedisServerInfo = {
  "127.0.0.1",  // hostname
  6379,         // port
  { 1, 500000 } // timeout = 1.5 seconds
};

int main(int argc, char** argv)
{
  parseCommandline(argc, argv);
  cout << "Loading URDF world model file: " << world_file << endl;
  auto redis_client = RedisClient();
  redis_client.serverIs(kRedisServerInfo);
  
  auto graphics_int = new Graphics::GraphicsInterface(world_file, Graphics::chai, Graphics::urdf, true);
  Graphics::ChaiGraphics* graphics = dynamic_cast<Graphics::ChaiGraphics *>(graphics_int->_graphics_internal);
  Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
  graphics->getCameraPose(CAMERA_NAME, camera_pos, camera_vertical, camera_lookat);
  
  auto robot = make_shared<Model::ModelInterface>(robot_file, Model::rbdl, Model::urdf, false);
  
  glfwSetErrorCallback(glfwError);
  glfwInit();
  
  GLFWmonitor* primary = glfwGetPrimaryMonitor();
  const GLFWvidmode* mode = glfwGetVideoMode(primary);
  
  // information about computer screen and GLUT display window
  int screenW = mode->width;
  int screenH = mode->height;
  int windowW = 0.8 * screenH;
  int windowH = 0.5 * screenH;
  int windowPosY = (screenH - windowH) / 2;
  int windowPosX = windowPosY;
  
  // create window and make it current
  glfwWindowHint(GLFW_VISIBLE, 0);
  GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS225a", NULL, NULL);
  glfwSetWindowPos(window, windowPosX, windowPosY);
  glfwShowWindow(window);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  
  // set callbacks
  glfwSetKeyCallback(window, keySelect);
  glfwSetMouseButtonCallback(window, mouseClick);
  glfwSetScrollCallback(window, mouseScroll);
  
  double last_cursorx, last_cursory;
  Eigen::Vector3d x, j5pos, j6pos, r5x, r5y, r5z, r6x, r6y, r6z;     
  auto sphere = new chai3d::cShapeSphere(0.07);
  auto segment = new chai3d::cMultiSegment();
  auto segment5x = new chai3d::cMultiSegment();
  auto segment5y = new chai3d::cMultiSegment();
  auto segment5z = new chai3d::cMultiSegment();
  auto segment6x = new chai3d::cMultiSegment();
  auto segment6y = new chai3d::cMultiSegment();
  auto segment6z = new chai3d::cMultiSegment();
  graphics->_world->addChild(segment);
  graphics->_world->addChild(segment5x);
  graphics->_world->addChild(segment5y);
  graphics->_world->addChild(segment5z);
  graphics->_world->addChild(segment6x);
  graphics->_world->addChild(segment6y);
  graphics->_world->addChild(segment6z);
  graphics->_world->addChild(sphere);
  chai3d::cColorf color_x, color_y, color_z, color;
  color.setWhite();
  color_x.setRed();
  color_y.setGreen();
  color_z.setBlue();
  
  // while window is open:
  while (!glfwWindowShouldClose(window)) {
    
    // read from Redis
    redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
    redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.getEigenMatrixDerivedString(TARGET_POSITION_KEY, x);
    redis_client.getEigenMatrixDerivedString(J5_POSITION_KEY, j5pos);
    redis_client.getEigenMatrixDerivedString(J6_POSITION_KEY, j6pos);
    redis_client.getEigenMatrixDerivedString(ROT_5_X, r5x);
    cout << r5x.transpose() << endl;
    redis_client.getEigenMatrixDerivedString(ROT_5_Y, r5y);
    redis_client.getEigenMatrixDerivedString(ROT_5_Z, r5z);
    redis_client.getEigenMatrixDerivedString(ROT_6_X, r6x);
    cout << r6x.transpose() << endl;
    redis_client.getEigenMatrixDerivedString(ROT_6_Y, r6y);
    redis_client.getEigenMatrixDerivedString(ROT_6_Z, r6z);
    
    segment->clear();
    segment->newSegment(chai3d::cVector3d(j5pos), chai3d::cVector3d(x));
    segment->setLineColor(color);

    segment5x->clear();
    segment5x->newSegment(chai3d::cVector3d(j5pos), chai3d::cVector3d(j5pos+r5x/3));
    segment5x->setLineColor(color_x);

    segment5y->clear();
    segment5y->newSegment(chai3d::cVector3d(j5pos), chai3d::cVector3d(j5pos+r5y/3));
    segment5y->setLineColor(color_y);

    segment5z->clear();
    segment5z->newSegment(chai3d::cVector3d(j5pos), chai3d::cVector3d(j5pos+r5z/3));
    segment5z->setLineColor(color_z);

    segment6x->clear();
    segment6x->newSegment(chai3d::cVector3d(j6pos), chai3d::cVector3d(j6pos+r6x/3));
    segment6x->setLineColor(color_x);
    
    segment6y->clear();
    segment6y->newSegment(chai3d::cVector3d(j6pos), chai3d::cVector3d(j6pos+r6y/3));
    segment6y->setLineColor(color_y);
    
    segment6z->clear();
    segment6z->newSegment(chai3d::cVector3d(j6pos), chai3d::cVector3d(j6pos+r6z/3));
    segment6z->setLineColor(color_z);
    
    sphere->setLocalPos(chai3d::cVector3d(x));
    
    // update transformations
    robot->updateModel();
      
    // update graphics. this automatically waits for the correct amount of time
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    graphics->updateGraphics(robot_name, robot.get());
    graphics->render(CAMERA_NAME, width, height);
    
    // swap buffers
    glfwSwapBuffers(window);
    
      // wait until all GL commands are completed
    glFinish();
    
    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    assert(err == GL_NO_ERROR);
    
    // poll for events
    glfwPollEvents();
    
    // move scene camera as required
    Eigen::Vector3d cam_up_axis;
    // cam_up_axis = camera_vertical;
      // cam_up_axis.normalize();
    cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
    Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
    cam_roll_axis.normalize();
    Eigen::Vector3d cam_lookat_axis = camera_lookat;
    cam_lookat_axis.normalize();
    if (fTransXp) {
	camera_pos = camera_pos + 0.05*cam_roll_axis;
	camera_lookat = camera_lookat + 0.05*cam_roll_axis;
    }
    if (fTransXn) {
      camera_pos = camera_pos - 0.05*cam_roll_axis;
	camera_lookat = camera_lookat - 0.05*cam_roll_axis;
    }
    if (fTransYp) {
      // camera_pos = camera_pos + 0.05*cam_lookat_axis;
      camera_pos = camera_pos + 0.05*cam_up_axis;
      camera_lookat = camera_lookat + 0.05*cam_up_axis;
    }
    if (fTransYn) {
      // camera_pos = camera_pos - 0.05*cam_lookat_axis;
      camera_pos = camera_pos - 0.05*cam_up_axis;
      camera_lookat = camera_lookat - 0.05*cam_up_axis;
    }
    if (fRotPanTilt) {
      // get current cursor position
      double cursorx, cursory;
      glfwGetCursorPos(window, &cursorx, &cursory);
      //TODO: might need to re-scale from screen units to physical units
      double compass = 0.006*(cursorx - last_cursorx);
      double azimuth = 0.006*(cursory - last_cursory);
      double radius = (camera_pos - camera_lookat).norm();
      Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
      camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
      Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
      // camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
      // TODO: the above doesn't work as intended because Chai treats the lookat
      // vector as a direction vector in the local frame, rather than as a lookat point
      camera_pos = m_pan*(camera_pos);
      camera_lookat = m_pan*(camera_lookat);
      // TODO: the above fix is a HUGE hack. Think about improving this.
    }
    if (fZoom) {
      camera_pos = camera_pos + 0.04*camera_lookat*zoomSpeed;
      fZoom = false;
    }
    graphics->setCameraPose(CAMERA_NAME, camera_pos, cam_up_axis, camera_lookat);
    glfwGetCursorPos(window, &last_cursorx, &last_cursory);
  }
  
  glfwDestroyWindow(window);
  glfwTerminate(); 
  return 0;
}

void parseCommandline(int argc, char** argv)
{
  /* Argument Parsing */
  if (argc != 4) {
    cout << "Usage: visualizer <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
    exit(0);
  }
  world_file = string(argv[1]);
  robot_file = string(argv[2]);
  robot_name = string(argv[3]);

  JOINT_ANGLES_KEY        = REDIS_KEY_PREFIX + robot_name + JOINT_ANGLES_KEY;
  JOINT_VELOCITIES_KEY    = REDIS_KEY_PREFIX + robot_name + JOINT_VELOCITIES_KEY;  
  
  TARGET_POSITION_KEY     = REDIS_KEY_PREFIX + robot_name + TARGET_POSITION_KEY;
  J5_POSITION_KEY         = REDIS_KEY_PREFIX + robot_name + J5_POSITION_KEY;
  J6_POSITION_KEY         = REDIS_KEY_PREFIX + robot_name + J6_POSITION_KEY;
  
  ROT_5_X	          = REDIS_KEY_PREFIX + robot_name + ROT_5_X;
  ROT_5_Y                 = REDIS_KEY_PREFIX + robot_name + ROT_5_Y;
  ROT_5_Z	          = REDIS_KEY_PREFIX + robot_name + ROT_5_Z;
  ROT_6_X	          = REDIS_KEY_PREFIX + robot_name + ROT_6_X;
  ROT_6_Y                 = REDIS_KEY_PREFIX + robot_name + ROT_6_Y;
  ROT_6_Z                 = REDIS_KEY_PREFIX + robot_name + ROT_6_Z; 
}

static void glfwError(int error, const char* description) {
  cerr << "GLFW Error: " << description << endl;
  exit(1);
}

static void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
  auto redis_client = RedisClient();
  redis_client.serverIs(kRedisServerInfo);
  Eigen::VectorXd x_target;
  redis_client.getEigenMatrixDerivedString(TARGET_POSITION_KEY, x_target);
  bool set = (action != GLFW_RELEASE);
  switch(key) {
  case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window,GL_TRUE); break;
  case GLFW_KEY_RIGHT: fTransXp = set; break;
  case GLFW_KEY_LEFT: fTransXn = set; break;
  case GLFW_KEY_UP: fTransYp = set; break;
  case GLFW_KEY_DOWN: fTransYn = set; break;
  case GLFW_KEY_W: x_target(2) = x_target(2) + 0.005; break;
  case GLFW_KEY_S: x_target(2) = x_target(2) - 0.005; break;
  case GLFW_KEY_A: x_target(1) = x_target(1) + 0.005; break;
  case GLFW_KEY_D: x_target(1) = x_target(1) - 0.005; break;
  case GLFW_KEY_Q: x_target(0) = x_target(0) + 0.005; break;
  case GLFW_KEY_E: x_target(0) = x_target(0) - 0.005; break;
  case GLFW_KEY_F: redis_client.set("servo_key", "1"); break;
  default: break;
  }
  redis_client.setEigenMatrixDerivedString(TARGET_POSITION_KEY, x_target);
}

static void mouseClick(GLFWwindow* window, int button, int action, int mods)
{
  bool set = (action != GLFW_RELEASE);
  switch (button) { 
  case GLFW_MOUSE_BUTTON_LEFT: fRotPanTilt = set; break;
  default: break;
  }
}

static void mouseScroll(GLFWwindow* window, double xoffset, double yoffset)
{
  fZoom = true;
  zoomSpeed = yoffset;
}
