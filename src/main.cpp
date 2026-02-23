#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

/* Initialization and Motor Configurations */
pros::MotorGroup leftMotors({-5, -4, -3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({6, 7, 9}, pros::MotorGearset::blue);
pros::Motor intake(1, pros::MotorGearset::blue);
pros::Motor hood(2, pros::MotorGearset::blue);
pros::adi::Pneumatics descore('B', false);
pros::adi::Pneumatics odomUp('C', true);
pros::adi::Pneumatics odom2Up('G', true);
pros::adi::Pneumatics scraper('A', false);
pros::adi::Pneumatics midgoal('D', false);
pros::adi::Pneumatics lowgoal('F', false);


// LEMLIB SPECIFIC CONFIGURATIONS
pros::Imu imu(10);

pros::Rotation verticalEnc(16);

// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2,
// -2);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);

/* LEMLIB INITIALIZATION */
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11.25,
                              lemlib::Omniwheel::NEW_325, 450, 2);

// Lateral Motion Controller (PID STUFF TO TUNE)
lemlib::ControllerSettings linearController(10, 0, 60, 0,
   1, 100, 3,
    500, 50);

// Angular Motion Controller (PID STUFF TO TUNE) 
lemlib::ControllerSettings angularController(5, 0, 45, 0, 1, 100,
   3, 500, 0);

// ODOMETRY SENSORS (DO NOT TOUCH)
// Only a single vertical tracking wheel is defined for this robot. If you add
// a second vertical or any horizontal tracking wheels, update the sensors
// initialization below accordingly.
lemlib::OdomSensors sensors(&vertical, nullptr, nullptr, nullptr, &imu);

// Drive Curves
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve
    throttleCurve(3,    // joystick deadband out of 127
                  10,   // minimum output where drivetrain will move out of 127
                  1.019 // expo curve gain
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steerCurve(3,    // joystick deadband out of 127
               10,   // minimum output where drivetrain will move out of 127
               1.019 // expo curve gain
    );

/* LEMLIB SETUP */
lemlib::Chassis chassis(drivetrain, linearController, angularController,
                        sensors, &throttleCurve, &steerCurve);

// Control motor with two button inputs (forward/reverse)
inline void controlMotor(pros::Motor &motor, int forwardVoltage,
                         int reverseVoltage,
                         pros::controller_digital_e_t forwardBtn,
                         pros::controller_digital_e_t reverseBtn,
                         pros::Controller &controller) {
  if (controller.get_digital(forwardBtn)) {
    motor.move_voltage(forwardVoltage);
  } else if (controller.get_digital(reverseBtn)) {
    motor.move_voltage(reverseVoltage);
  } else {
    motor.move_velocity(0);
  }
}

// Toggle pneumatic on button press
inline void toggleOnPress(pros::adi::Pneumatics &piston,
                          pros::controller_digital_e_t button,
                          pros::Controller &controller) {
  if (controller.get_digital_new_press(button)) {
    piston.toggle();
  }
}

void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

void initialize() {
  pros::lcd::initialize();
  odomUp.set_value(false);
  //skills descore

  pros::delay(1000);

  chassis.calibrate();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

  pros::Task screenTask([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // log position telemetry
      lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
      // delay to save resources
      pros::delay(50);
    }
  });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

  /*
  //Solo AWP aligned
  //push
  chassis.setPose(0, 0, 0);
  //grab first pile
  chassis.moveToPoint(0,19,700,{.forwards=true,.maxSpeed=85});
  chassis.turnToPoint(-35,19,400,{.forwards=true});
  intake.move_voltage(-11000);
  hood.move_voltage(5000);
  chassis.moveToPoint(-36,19,1000,{.forwards=true,.maxSpeed=80},true);
  pros::delay(500);
  scraper.toggle();
  pros::delay(400);
  scraper.toggle();
  //move back and to long goal
  chassis.turnToPoint(-2,43,500,{.forwards=true});
  chassis.moveToPoint(-2,43,1200,{.forwards=true,.maxSpeed=80});
  //align to long goal
  chassis.turnToPoint(-22,43,400,{.forwards=false});
  chassis.moveToPoint(-22,43,800,{.forwards=false,.maxSpeed=80},true);
  //score on long goal
  pros::delay(400);
  hood.move_voltage(-11000);
  intake.move_voltage(-11000);
  pros::delay(800);
  //move to matchloader 1
  chassis.moveToPoint(9,43,800,{.forwards=true,.maxSpeed=80},true);
  scraper.toggle();
  hood.move_voltage(5000);
  chassis.moveToPoint(27,43,600,{.forwards=true,.maxSpeed=60});   
//move to second pile
  chassis.moveToPoint(0,43,300,{.forwards=false},true);
  scraper.toggle();
  chassis.turnToPoint(-24,-25 ,400,{.forwards=true});
  chassis.moveToPoint(-24,-25,2200,{.forwards=true,.maxSpeed=80},true);
  pros::delay(1500); 
  scraper.toggle();
  //align to mid goal
  chassis.turnToHeading(135,300);
  chassis.moveToPoint(-38,-14,1500,{.forwards=false,.maxSpeed=40},true);
  pros::delay(100);
  intake.move_voltage(11000);
  pros::delay(300);
  midgoal.toggle();
  //score on mid goal
  intake.move_voltage(-9000);
  pros::delay(800);
  scraper.toggle();
  //move to second matchload
  chassis.moveToPoint(0,-53, 1300,{.forwards=true,.maxSpeed=85},true);
  scraper.toggle();
  intake.move_voltage(-11000);
  chassis.turnToPoint(5,-53,300,{.forwards=true});
  midgoal.set_value(false);
  chassis.moveToPoint(24,-53,1200,{.forwards=true,.maxSpeed=85});
  pros::delay(400);
  //move to second long goal
  chassis.moveToPoint(-20,-53,2500,{.forwards=false});
  pros::delay(300);
  hood.move_voltage(-11000);
  */

  /*
  //4+3
  chassis.setPose(0, 0, 0);
  intake.move_velocity(-11000);
  hood.move_velocity(5000);
  chassis.turnToPoint(-12, 32, 200);
  chassis.moveToPoint(-12, 32, 2000, {.maxSpeed = 45});

  chassis.turnToPoint(-36,4,500,{.forwards=false});
  chassis.moveToPoint(-36, 4, 2000, {.forwards=false, .maxSpeed = 80});

  chassis.turnToPoint(-36,23,500,{.forwards=false});
  chassis.moveToPoint(-36,23,700,{.forwards=false, .maxSpeed=70});
  pros::delay(500);
  hood.move_voltage(-11000);
  pros::delay(1000);
  hood.move_velocity(0);
  chassis.setPose(-36, 23, 180);

  scraper.toggle();
  chassis.moveToPoint(-35,-6,500,{.forwards=true,.maxSpeed=80});
  chassis.moveToPoint(-35,-19,1100,{.forwards=true,.maxSpeed=40});

  chassis.turnToPoint(3,45,500,{.forwards=false});
  chassis.moveToPoint(3,45,2500,{.forwards=false,.maxSpeed=80});
  pros::delay(800);
  intake.move_velocity(11000);
  pros::delay(300);
  midgoal.set_value(true);
  pros::delay(100);
  intake.move_velocity(-9000);
  hood.move_voltage(5000);
  pros::delay(2000);

  chassis.turnToPoint(-26,17,500,{.forwards=true});
  chassis.moveToPoint(-26,17,1200,{.forwards=true,.maxSpeed=80});
  chassis.turnToPoint(-26,50,400,{.forwards=false});
  pros::delay(1000);
  chassis.moveToPoint(-26,40,1000,{.forwards=false});

  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
*/

  //grab first pile
  odomUp.set_value(false);
  chassis.setPose(0, 0, 0);
  pros::delay(500);
  chassis.moveToPoint(0,13,5000,{.forwards=true,.maxSpeed=80},false);
  chassis.turnToPoint(-12,25,400,{.forwards=true});
  intake.move_voltage(-11000);
  chassis.moveToPoint(-12,25,1000,{.forwards=true,.maxSpeed=60},false);
  //move to midgoal
  chassis.turnToPoint(6,41,1000,{.forwards=false});
  chassis.moveToPoint(6,41,5000,{.forwards=false,.maxSpeed=40}, true);
  pros::delay(500);
  intake.move_voltage(11000);
  pros::delay(200);
  midgoal.toggle();
  intake.move_voltage(0);
  pros::delay(300);
  intake.move_voltage(-9000);
  pros::delay(1000);
  //move to matchload 
  chassis.moveToPoint(-37,2,3000,{.forwards=true,.maxSpeed=80},false);
  intake.move_voltage(-11000);
  hood.move_voltage(-11000);
  chassis.turnToPoint(-37,-10,1000,{.forwards=true});
  scraper.toggle();
  hood.move_voltage(3000);
  //grab matchload
  pros::delay(500); 
  midgoal.set_value(false);
  chassis.moveToPoint(-37,-19,2700,{.forwards=true,.maxSpeed=50},false);
  scraper.toggle();
  //move to side of long goal
  chassis.moveToPoint(-37,1,1000,{.forwards=false,.maxSpeed=85},false);
  chassis.turnToPoint(-49,1,400,{.forwards=false});
  chassis.moveToPoint(-49,1,1000,{.forwards=false,.maxSpeed=85},false);
  chassis.turnToPoint(-49,100,400,{.forwards=false});
  chassis.moveToPoint(-49,100,10000,{.forwards=false,.maxSpeed=80},false);
  //align to goal
  chassis.turnToPoint(-35,100,1000,{.forwards=false});
  chassis.moveToPoint(-35,100,1000,{.forwards=false,.maxSpeed=80},false);
  chassis.turnToPoint(-35,10,1000,{.forwards=false});
  chassis.moveToPoint(-35,50,1500,{.forwards=false,.maxSpeed=60},false);
  pros::delay(400);
  hood.move(-11000);
  pros::delay(1700);  
  //push with hood
  chassis.setPose(-35, 70, 0);
  scraper.toggle();
  hood.move_voltage(3000);
  chassis.moveToPoint(-35,100,1000,{.forwards=true,.maxSpeed=80},false);
  chassis.moveToPoint(-35,120,2700,{.forwards=true,.maxSpeed=80},false);
  //score second time
  chassis.moveToPoint(-35,60,1800,{.forwards=false,.maxSpeed=60});
  pros::delay(1000);
  hood.move(-11000);
  pros::delay(2000);  
  chassis.setPose(-35, 70, 0);
  chassis.moveToPoint(-35,75,1300,{.forwards=false,.maxSpeed=70},false);
  chassis.moveToPoint(-35,70,1300,{.forwards=false,.maxSpeed=70},false);
  pros::delay(500);
  chassis.setPose(-35, 70, 0);
  pros::delay(500);
  scraper.toggle();
  chassis.moveToPoint(-35,96,1800,{.forwards=true,.maxSpeed=80},false);
 //move to park zone
  chassis.turnToPoint(-2,108,1000,{.forwards=true});
  hood.move_voltage(3000);
  chassis.moveToPoint(-2,108,2700,{.forwards=true,.maxSpeed=80},false);
  //align to park
  chassis.turnToPoint(30,108,1000,{.forwards=true});
  odomUp.set_value(true);
  pros::delay(1000);
  //cross park
  chassis.moveToPoint(30,110,1000,{.forwards=true},false);
  chassis.moveToPoint(30,110,1500,{.forwards=true,.maxSpeed=60},false);
  pros::delay(1000);
  //reset on park
  odomUp.set_value(false);
  chassis.setPose(-10,107,90);
  pros::delay(1000);
  chassis.turnToPoint(-60,115,2000,{.forwards=false,.maxSpeed=80},false);
  chassis.moveToPoint(-60,115,2000,{.forwards=false,.maxSpeed=55},false);
  chassis.setPose(-10,107,90);
  //move to midgoal
  chassis.moveToPoint(-5,107,2000,{.forwards=false,.maxSpeed=80},false);
  chassis.turnToPoint(-5,68,1000,{.forwards=true},false);
  chassis.moveToPoint(-5,68,2000,{.forwards=true,.maxSpeed=80},true);
  scraper.toggle();
  //align to mid goal
  chassis.turnToPoint(-15,52,1000,{.forwards=false});
  chassis.moveToPoint(-15,52,1000,{.forwards=false,.maxSpeed=40}, true);
  pros::delay(500);
  intake.move_voltage(11000);
  pros::delay(200);
  midgoal.toggle();
  intake.move_voltage(0);
  pros::delay(300);
  intake.move_voltage(-9000);
  pros::delay(1000);
  //move to third matchload
  chassis.turnToPoint(30,75,1000,{.forwards=true});
  chassis.moveToPoint(30,75,1000,{.forwards=true,.maxSpeed=80},false);

  /*
  intake.move_voltage(11000);
  pros::delay(200);
  intake.move_voltage(0);
  pros::delay(1000);
  intake.move_voltage(-10000);
  pros::delay(100);
  midgoal.toggle();
  pros::delay(1700);
  //move to match load
  chassis.moveToPoint(-42, 1, 1800, {.forwards=true,.maxSpeed=85}, true);
  pros::delay(1500);
  midgoal.toggle();
  
  //matchload
  chassis.turnToPoint(-42, -20, 500, {.forwards=true});
  scraper.toggle();
  chassis.moveToPoint(-42, -20, 2700, {.forwards=true,.maxSpeed=40});
  
  //move to side of long goal
  chassis.turnToPoint(-50, 10, 400, {.forwards=false});
  scraper.toggle();
  chassis.moveToPoint(-50, 10, 1200, {.forwards=false,.maxSpeed=85});
  chassis.turnToPoint(-50, 15, 1000, {.forwards=false});
  chassis.moveToPoint(-50, 120, 1000, {.forwards=false,.maxSpeed=85});
  */

  /*
  // Skills safe
  chassis.setPose(0, 0, 90);
  descore.toggle();
  chassis.moveToPoint(28, 0,1000,{.forwards=true,.maxSpeed=80});
  chassis.turnToPoint(28, -16, 1000, {.maxSpeed=80});
  scraper.toggle();
  pros::delay(500);
  intake.move_voltage(-11000);
  hood.get_voltage(5000);

  //scrape and clear matchload
  chassis.moveToPoint(28, -24,2500,{.forwards=true,.maxSpeed=40}, false);
  chassis.moveToPoint(28, 0, 1000,{.forwards=false,.maxSpeed=60}, false);
  scraper.retract();
  chassis.turnToPoint(44, 0, 1000,{.forwards=false,.maxSpeed=80});
  chassis.moveToPoint(44, 0, 1500,{.forwards=false,.maxSpeed=80});
  //go to other side of goal
  chassis.turnToPoint(39, 96, 1000,{.forwards=true,.maxSpeed=80});
  intake.move_velocity(0);
  chassis.moveToPoint(39, 96, 3000,{.forwards=true,.maxSpeed=80});
  //move to goal
  chassis.turnToPoint(29, 94, 1500,{.forwards=false,.maxSpeed=80});
  intake.move_voltage(-11000);
  chassis.moveToPoint(29, 94, 2000,{.forwards=false,.maxSpeed=80});
  //score on goal
  chassis.turnToPoint(29,79, 1000,{.forwards=false,.maxSpeed=80});
  chassis.moveToPoint(29,79, 750,{.forwards=false,.maxSpeed=70}, false);
  hood.move_voltage(-11000);
  pros::delay(2500);
  chassis.setPose(29, 85,0);

  scraper.toggle();
  hood.move_voltage(0);
  //matchload
  chassis.moveToPoint(28, 100, 500,{.forwards=true,.maxSpeed=800}, false);
  chassis.moveToPoint(29, 134, 3000,{.forwards=true,.maxSpeed=30}, false);
  //score second time on goal
  chassis.moveToPoint(29, 84, 1000,{.forwards=false,.maxSpeed=80}, false);
  hood.move_voltage(-11000);
  pros::delay(2000);
  scraper.retract();
  //move to third matchload
  chassis.moveToPoint(29, 96, 1000,{.forwards=true,.maxSpeed=80}, false);
  chassis.turnToPoint(-73, 96, 500,{.forwards=false,.maxSpeed=80});
  hood.move_voltage(0);
  chassis.moveToPoint(-73, 96, 4000,{.forwards=false,.maxSpeed=80});
  chassis.turnToPoint(-73, 134, 1000,{.forwards=true,.maxSpeed=80});
  scraper.toggle();
  //second matchload
  chassis.moveToPoint(-73, 110, 1000,{.forwards=true,.maxSpeed=80});
  chassis.moveToPoint(-73, 148, 1000,{.forwards=true,.maxSpeed=40});
  chassis.moveToPoint(-73, 148, 2000,{.forwards=true,.maxSpeed=80});
  chassis.moveToPoint(-73, 100, 1000,{.forwards=false,.maxSpeed=80});
  scraper.retract();
  intake.move_voltage(0);
  chassis.turnToPoint(-87, 100, 500,{.forwards=false,.maxSpeed=80});
  //move to score second on second goal
  chassis.moveToPoint(-87, 100, 3000,{.forwards=false,.maxSpeed=80});
  chassis.turnToPoint(-87, 11, 1000,{.forwards=false,.maxSpeed=80});
  chassis.moveToPoint(-87, 10, 2000,{.forwards=false,.maxSpeed=80});
  //align to second goal
  chassis.turnToPoint(-75, 10, 1500,{.forwards=false,.maxSpeed=80});
  chassis.moveToPoint(-75, 10, 1000,{.forwards=false,.maxSpeed=80});
  //score on second goal
  chassis.turnToPoint(-75, 36, 1000,{.forwards=false,.maxSpeed=80});
  chassis.moveToPoint(-75, 36, 1000,{.forwards=false,.maxSpeed=80});
  pros::delay(500);
  hood.move_voltage(-11000);
  intake.move_voltage(-11000);
  pros::delay(1500);
  //third matchload
  hood.move_voltage(0);
  scraper.toggle();
  chassis.moveToPoint(-76, -29, 500,{.forwards=true,.maxSpeed=80}, false);
  chassis.moveToPoint(-76, -29, 3000,{.forwards=true,.maxSpeed=60}, false);
  //score on third goal
  chassis.moveToPoint(-76, 33, 1000,{.forwards=false,.maxSpeed=80}, false);
  pros::delay(500);
  hood.move_voltage(-11000);
  scraper.toggle();
  pros::delay(1500);
  //move to park
  intake.move_voltage(11000);
  chassis.moveToPoint(-76,7,1000,{.forwards=true,.maxSpeed=80}, false);
  chassis.turnToPoint(-40,-25,750,{.forwards=true,.maxSpeed=80});
  chassis.moveToPoint(-40,-25,1000,{.forwards=true,.maxSpeed=60}, false);
  pros::delay(1000);
  chassis.turnToPoint(-15,-25,750,{.forwards=true,.maxSpeed=80});
  odomUp.set_value(true);
  chassis.moveToPoint(2,-25,1000,{.forwards=true},true);
  pros::delay(100);
  scraper.toggle();
  pros::delay(250);
  odomUp.set_value(true);
  pros::delay(1000);
  scraper.toggle();
  */

  /*
  //Left Elim
   chassis.setPose(0, 0, 0);
  descore.set_value(true);
  intake.move_velocity(-11000);
  chassis.turnToPoint(-11, 32, 250);
  chassis.moveToPoint(-11, 32, 7000, {.maxSpeed = 85},true);
  pros::delay(500);
  scraper.toggle();
  pros::delay(300);

  scraper.toggle();
  chassis.turnToPoint(-31,4,500,{.forwards=false});
  chassis.moveToPoint(-31, 4, 1000, {.forwards=false, .maxSpeed = 80});

  chassis.turnToPoint(-33,23,750,{.forwards=false});
  chassis.moveToPoint(-33,23,750,{.forwards=false, .maxSpeed=85});
  pros::delay(500);
  hood.move_voltage(-11000);
  pros::delay(900);
  hood.move_velocity(0);

  chassis.turnToPoint(-26,10,250,{.forwards=true,.maxSpeed=85});
  chassis.moveToPoint(-26,10,250,{.forwards=true,.maxSpeed=85});
  descore.toggle();
  chassis.turnToPoint(-26,23,500,{.forwards=false,.maxSpeed=85});
  descore.toggle();
  pros::delay(150);
  chassis.moveToPoint(-26,35,1000,{.forwards=false,.maxSpeed=50});
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  */

  /*
 //Right Elim
  chassis.setPose(0, 0, 0);
  descore.set_value(true);
  intake.move_velocity(-11000);
  chassis.turnToPoint(11, 32, 250);
  chassis.moveToPoint(11, 32, 1000, {.maxSpeed = 85},true);
  pros::delay(500);
  scraper.toggle();
  pros::delay(200);


  chassis.turnToPoint(30,4,500,{.forwards=false});
  scraper.toggle();
  chassis.moveToPoint(30, 4, 1000, {.forwards=false, .maxSpeed = 80});


  chassis.turnToPoint(32,23,750,{.forwards=false});
  chassis.moveToPoint(32,23,750,{.forwards=false, .maxSpeed=85});
  pros::delay(500);
  hood.move_voltage(-11000);
  pros::delay(1100);
  hood.move_velocity(0);


  chassis.turnToPoint(41,10,250,{.forwards=true,.maxSpeed=85});
  chassis.moveToPoint(41,10,500,{.forwards=true,.maxSpeed=85});
  descore.toggle();
  chassis.turnToPoint(41,23,500,{.forwards=false,.maxSpeed=85});
  descore.toggle();
  pros::delay(150);
  chassis.moveToPoint(41,35,1000,{.forwards=false,.maxSpeed=50});
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  */

  /*
   // Right Side 7
  //grab first pile
  odom2Up.set_value(false);
  odomUp.set_value(false);
  chassis.setPose(0, 0, 0); 
  pros::delay(300);
  descore.set_value(true);
  intake.move_velocity(-11000);
  chassis.turnToPoint(11, 32, 250, {.maxSpeed = 85});
  chassis.moveToPoint(11, 32, 1300, {.maxSpeed = 85}, true);
  pros::delay(390);
  scraper.toggle();
  pros::delay(300);
  scraper.toggle();
  //move to goal
  chassis.turnToPoint(33, 2, 500, {.forwards = false, .maxSpeed = 85});
  chassis.moveToPoint(33, 2, 1300, {.forwards = false, .maxSpeed = 85});

  chassis.turnToPoint(33, 23, 500, {.forwards = false, .maxSpeed = 85});
  chassis.moveToPoint(33, 23, 900, {.forwards = false, .maxSpeed = 85}, true);
  pros::delay(500);
  hood.move_voltage(-11000);
  pros::delay(1300);
  hood.move_velocity(1000);
  chassis.setPose(33, 19, 180);

  scraper.toggle();
  chassis.moveToPoint(33, 0, 500, {.forwards = true, .maxSpeed = 85}, false);
  chassis.moveToPoint(33, -24, 1100, {.forwards = true, .maxSpeed = 50}, false);

  chassis.moveToPoint(33, 23, 900, {.forwards = false, .maxSpeed = 70}, true);

  pros::delay(500);
  hood.move_voltage(-11000);
  scraper.toggle();
  pros::delay(1500);
  chassis.turnToPoint(41, 9, 300, {.forwards = true, .maxSpeed = 85});
  chassis.moveToPoint(41, 9, 400, {.forwards = true, .maxSpeed = 85});
  chassis.turnToPoint(41, 40, 300, {.forwards = false, .maxSpeed = 85});
  descore.set_value(false);
  pros::delay(150);
  chassis.moveToPoint(41, 28, 1000, {.forwards = false, .maxSpeed = 50});
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  */

  /*
  //Left Side 9
   //grab first pile
  chassis.setPose(0, 0, 0);
  descore.set_value(true);
  intake.move_velocity(-11000);
  chassis.turnToPoint(-11, 36, 250, {.maxSpeed = 85});
  chassis.moveToPoint(-11, 36, 1500, {.maxSpeed = 85}, true);
  pros::delay(390);
  scraper.toggle();
  pros::delay(200);
  //move to second pile
  scraper.toggle();
  chassis.turnToPoint(-28, 40, 750, {.maxSpeed = 85});
  chassis.moveToPoint(-28, 40, 1500, {.maxSpeed = 70}, true);
  hood.move_voltage(1000);
  //move back 
  chassis.moveToPoint(-11, 32, 2000, {.forwards = false, .maxSpeed = 85});
  //move to goal
  chassis.turnToPoint(-35, 4, 750, {.forwards = false, .maxSpeed = 85});
  chassis.moveToPoint(-35, 4, 1500, {.forwards = false, .maxSpeed = 85});

  chassis.turnToPoint(-35, 23, 1000, {.forwards = false, .maxSpeed = 85});
  chassis.moveToPoint(-35, 23, 900, {.forwards = false, .maxSpeed = 85}, false);
  pros::delay(100);
  hood.move_voltage(-11000);
  pros::delay(1500);
  hood.move_velocity(1000);
  chassis.setPose(-38, 19, 180);

  scraper.toggle();
  chassis.moveToPoint(-37, -5, 500, {.forwards = true, .maxSpeed = 85}, false);
  chassis.moveToPoint(-37, -24, 1100, {.forwards = true, .maxSpeed = 50}, false);

  chassis.moveToPoint(-38, 23, 1000, {.forwards = false, .maxSpeed = 70}, false);

  hood.move_voltage(-11000);
  scraper.toggle();
  pros::delay(750);
  chassis.turnToPoint(-30, 9, 500, {.forwards = true, .maxSpeed = 85});
  chassis.moveToPoint(-30, 9, 500, {.forwards = true, .maxSpeed = 85});
  chassis.turnToPoint(-30, 40, 500, {.forwards = false, .maxSpeed = 85});
  descore.set_value(false);
  pros::delay(150);
  chassis.moveToPoint(-30, 28, 1000, {.forwards = false, .maxSpeed = 50});
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  */
}

void opcontrol() {
  odomUp.set_value(true);
  odom2Up.set_value(true);
  pros::Controller controller(pros::E_CONTROLLER_MASTER);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  constexpr pros::controller_analog_e_t THROTTLE_AXIS =
      pros::E_CONTROLLER_ANALOG_LEFT_Y;
  constexpr bool THROTTLE_INVERT = false; // set to true to invert forward/back
  constexpr pros::controller_analog_e_t TURN_AXIS =
      pros::E_CONTROLLER_ANALOG_RIGHT_X;
  constexpr bool TURN_INVERT = false; // set to true to invert left/right

  while (true) {
    if ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) &&
        (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))) {
      hood.move_voltage(-11000);
      intake.move_voltage(-11000);
    } else if (!(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) &&
               (controller.get_digital_new_press(
                   pros::E_CONTROLLER_DIGITAL_L1))) {
    
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      midgoal.set_value(true);
      intake.move_voltage(-9000);
      hood.move_voltage(11000);
  //skills macro
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      scraper.set_value(true);
      pros::delay(200);
      intake.move_voltage(11000);
      hood.move_voltage(11000);
      pros::delay(100);

     } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake.move_voltage(-11000);
      hood.move_voltage(5000);
      lowgoal.set_value(false);
      midgoal.set_value(false);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.move_voltage(11000);
      hood.move_voltage(5000);
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    midgoal.toggle();
    /*
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {      
      scraper.set_value(true);
      pros::delay(100);
      intake.move_voltage(10000);
      hood.move_voltage(11000);
      pros::delay(150);
      */
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      lowgoal.toggle();
    } else {
      hood.move_velocity(0);
      intake.move_velocity(0);
    }
    // read joystick positions using the configurable mapping above
    int throttle =
        controller.get_analog(THROTTLE_AXIS) * (THROTTLE_INVERT ? -1 : 1);
    int turn = controller.get_analog(TURN_AXIS) * (TURN_INVERT ? -1 : 1);
    // move the chassis with arcade drive (throttle, turn)
    chassis.arcade(throttle, turn);

    // Toggle scraper on X press
    toggleOnPress(scraper, pros::E_CONTROLLER_DIGITAL_X, controller);  
  }
}
