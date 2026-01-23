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
pros::MotorGroup rightMotors({6, 7, 8}, pros::MotorGearset::blue);
pros::Motor intake(1, pros::MotorGearset::blue);
pros::Motor hood(2, pros::MotorGearset::blue);
pros::adi::Pneumatics descore('B', false);
pros::adi::Pneumatics odomUp('C', true);
pros::adi::Pneumatics scraper('A', false);

// LEMLIB SPECIFIC CONFIGURATIONS
pros::Imu imu(10);

pros::Rotation verticalEnc(-9);

// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2,
// -2);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);

/* LEMLIB INITIALIZATION */
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11.25,
                              lemlib::Omniwheel::NEW_325, 450, 2);

// Lateral Motion Controller (PID STUFF TO TUNE)
lemlib::ControllerSettings linearController(14, 0, 60, 0,
   1, 100, 3,
    500,
    0);

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

/* HELPER FUNCTIONS */
inline void odom(bool raise) { odomUp.set_value(raise); }

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

  odom(false);
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
   //Solo AWP
  chassis.setPose(0, 0, 0);
  intake.move_velocity(-11000);
  chassis.turnToPoint(-12, 32, 500, {.maxSpeed = 75},true);
  chassis.moveToPoint(-12, 32, 1200, {.maxSpeed = 80},true);
  pros::delay(400);
  scraper.toggle();


  chassis.turnToPoint(-37,4,500,{.forwards=false,.maxSpeed=85});
  chassis.moveToPoint(-37, 4, 1000, {.forwards=false, .maxSpeed = 80});
  scraper.toggle();


  chassis.turnToPoint(-37,24,500,{.forwards=false, .maxSpeed=85});
  chassis.moveToPoint(-37,24,1000,{.forwards=false, .maxSpeed=70});
  pros::delay(700);
  hood.move_voltage(-11000);
  pros::delay(800);
  hood.move_velocity(0);


  chassis.moveToPoint(-36,-19,1600,{.forwards=true,.maxSpeed=80});
  scraper.toggle();


  chassis.turnToPoint(-3,41,500,{.forwards=false,.maxSpeed=85});
  chassis.moveToPoint(-3,41,2300,{.forwards=false,.maxSpeed=80});
  pros::delay(1150);
  intake.move_velocity(-8000);
  hood.move_voltage(-7000);
  pros::delay(450);


  intake.move_velocity(0);
  hood.move_voltage(0);


  chassis.moveToPoint(-5,35,350,{.forwards=true,.maxSpeed=80});
  chassis.turnToPoint(38,30,500,{.forwards=true,.maxSpeed=80});
  scraper.toggle();
  intake.move_velocity(-11000);


  chassis.moveToPoint(38,30,1000,{.forwards=true,.maxSpeed=90},true);
  pros::delay(750);
  scraper.toggle();


  chassis.turnToPoint(60,0,500,{.forwards=true,.maxSpeed=85});
  chassis.moveToPoint(60,0,1000,{.forwards=true,.maxSpeed=80});


  chassis.turnToPoint(60,-10,500,{.forwards=true,.maxSpeed=85});
  chassis.moveToPoint(60,-19,1000,{.forwards=true,.maxSpeed=80});


  chassis.moveToPoint(60, 32,900,{.forwards=false});
  pros::delay(600);
  hood.move_voltage(-11000);
  */

  /*
  //4+3
  chassis.setPose(0, 0, 0);
  intake.move_velocity(-11000);
  chassis.turnToPoint(-12, 32, 750, {.maxSpeed = 45});
  chassis.moveToPoint(-12, 32, 2000, {.maxSpeed = 45});


  chassis.turnToPoint(-37,4,1000,{.forwards=false,.maxSpeed=80});
  chassis.moveToPoint(-37, 4, 2000, {.forwards=false, .maxSpeed = 80});


  chassis.turnToPoint(-37,23,1000,{.forwards=false, .maxSpeed=80});
  chassis.moveToPoint(-37,23,1000,{.forwards=false, .maxSpeed=70});
  pros::delay(500);
  hood.move_voltage(-11000);
  pros::delay(1000);
  hood.move_velocity(0);
  chassis.setPose(-37, 23, 180);


  scraper.toggle();
  chassis.moveToPoint(-35,-19,1850,{.forwards=true,.maxSpeed=50});


  chassis.turnToPoint(-2,42,750,{.forwards=false,.maxSpeed=80});
  chassis.moveToPoint(-2,42,2500,{.forwards=false,.maxSpeed=80});
  pros::delay(1000);
  intake.move_velocity(-9000);
  hood.move_voltage(-5000);
  pros::delay(600);


  intake.move_velocity(0);
  hood.move_voltage(11000);
  chassis.moveToPoint(0,44,1000,{.forwards=false,.maxSpeed=80});
  chassis.moveToPoint(-2,42,1000,{.forwards=false,.maxSpeed=80});
  intake.move_velocity(-9000);
  hood.move_voltage(-5000);
  scraper.toggle();


  pros::delay(500);
  chassis.moveToPoint(-3,38,1000,{.forwards=true,.maxSpeed=80});
  pros::delay(500);


  chassis.turnToPoint(-29,17,750,{.forwards=true,.maxSpeed=80});
  chassis.moveToPoint(-29,17,1500,{.forwards=true,.maxSpeed=80});
  chassis.turnToPoint(-29,50,1000,{.forwards=false,.maxSpeed=80});
  pros::delay(1000);
  chassis.moveToPoint(-29,38,1000,{.forwards=false});


  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  */

  /*
  // Skills
  chassis.setPose(0, 0, 90);
  descore.toggle();
  chassis.moveToPoint(32, 0,1000,{.forwards=true,.maxSpeed=80});
  chassis.turnToPoint(31, -16, 1000, {.maxSpeed=80});
  scraper.toggle();
  pros::delay(500);
  intake.move_voltage(-11000);


  //scrape and clear matchload
  chassis.moveToPoint(31, -24,2500,{.forwards=true,.maxSpeed=50}, false);
  chassis.moveToPoint(31, 0, 1000,{.forwards=false,.maxSpeed=60}, false);
  scraper.retract();
  chassis.turnToPoint(44, 0, 1000,{.forwards=false,.maxSpeed=80});
  chassis.moveToPoint(44, 0, 1500,{.forwards=false,.maxSpeed=80});
  //go to other side of goal
  chassis.turnToPoint(39, 96, 1000,{.forwards=true,.maxSpeed=80});
  intake.move_velocity(0);
  chassis.moveToPoint(39, 96, 3000,{.forwards=true,.maxSpeed=80});
  //move to goal
  chassis.turnToPoint(27, 94, 1500,{.forwards=false,.maxSpeed=80});
  intake.move_voltage(-11000);
  chassis.moveToPoint(27, 94, 2000,{.forwards=false,.maxSpeed=80});
  //score on goal
  chassis.turnToPoint(27,79, 1000,{.forwards=false,.maxSpeed=80});
  chassis.moveToPoint(27,79, 750,{.forwards=false,.maxSpeed=70}, false);
  hood.move_voltage(-11000);
  pros::delay(2500);
  chassis.setPose(27, 85,0);


  scraper.toggle();
  hood.move_voltage(0);
  //matchload
  chassis.moveToPoint(27, 100, 500,{.forwards=true,.maxSpeed=30}, false);
  chassis.moveToPoint(27, 134, 3000,{.forwards=true,.maxSpeed=60}, false);
  //score second time on goal
  chassis.moveToPoint(27, 84, 1000,{.forwards=false,.maxSpeed=80}, false);
  hood.move_voltage(-11000);
  pros::delay(2000);
  scraper.retract();
  //move to third matchload
  chassis.moveToPoint(27, 96, 1000,{.forwards=true,.maxSpeed=80}, false);
  chassis.turnToPoint(-71, 96, 500,{.forwards=false,.maxSpeed=80});
  hood.move_voltage(0);
  chassis.moveToPoint(-72, 96, 4000,{.forwards=false,.maxSpeed=80});
  chassis.turnToPoint(-72, 134, 1000,{.forwards=true,.maxSpeed=80});
  scraper.toggle();
  //second matchload
  chassis.moveToPoint(-72, 148, 1000,{.forwards=true,.maxSpeed=60});
  chassis.moveToPoint(-72, 148, 2000,{.forwards=true,.maxSpeed=80});
  chassis.moveToPoint(-72, 100, 1000,{.forwards=false,.maxSpeed=80});
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
  scraper.toggle();
  chassis.moveToPoint(2,-25,1000,{.forwards=true},true);
  pros::delay(250);
  odomUp.set_value(true);
  pros::delay(1000);
  scraper.toggle();








  /*
  //Left Elim
   chassis.setPose(0, 0, 0);
   descore.toggle();
    intake.move_velocity(-11000);
   chassis.turnToPoint(-11, 32, 750, {.maxSpeed = 65});
   chassis.moveToPoint(-11, 32, 2000, {.maxSpeed = 80},true);
   pros::delay(400);
   scraper.toggle();
   pros::delay(600);
   chassis.turnToPoint(-37,4,500,{.forwards=false,.maxSpeed=80});
   scraper.toggle();
   chassis.moveToPoint(-37, 4, 2000, {.forwards=false, .maxSpeed = 80});


   chassis.turnToPoint(-37,23,500,{.forwards=false, .maxSpeed=80});
   chassis.moveToPoint(-37,23,1000,{.forwards=false, .maxSpeed=70});
   pros::delay(500);
   hood.move_voltage(-11000);


   pros::delay(750);
   chassis.turnToPoint(-29,10,500,{.forwards=true,.maxSpeed=80});
   chassis.moveToPoint(-29,10,750,{.forwards=true,.maxSpeed=80});
   chassis.turnToPoint(-29,23,500,{.forwards=false,.maxSpeed=80});
   descore.toggle();
   pros::delay(150);
   chassis.moveToPoint(-29,38,1500,{.forwards=false,.maxSpeed=50});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
   */

  /*
 //Right Elim
  chassis.setPose(0, 0, 0);
  descore.set_value(true);
  intake.move_velocity(-11000);
  chassis.turnToPoint(11, 32, 750, {.maxSpeed = 65});
  chassis.moveToPoint(11, 32, 2000, {.maxSpeed = 80},true);
  pros::delay(250);
  scraper.toggle();
  pros::delay(500);


  chassis.turnToPoint(35,4,1000,{.forwards=false,.maxSpeed=80});
  scraper.toggle();
  chassis.moveToPoint(35, 4, 2500, {.forwards=false, .maxSpeed = 80});


  chassis.turnToPoint(35,23,1000,{.forwards=false, .maxSpeed=80});
  chassis.moveToPoint(35,23,1000,{.forwards=false, .maxSpeed=70});
  pros::delay(500);
  hood.move_voltage(-11000);
  pros::delay(1500);
  hood.move_velocity(0);


  chassis.turnToPoint(43,10,500,{.forwards=true,.maxSpeed=80});
  chassis.moveToPoint(43,10,500,{.forwards=true,.maxSpeed=80});
  descore.toggle();
  chassis.turnToPoint(43,23,500,{.forwards=false,.maxSpeed=80});
  descore.toggle();
  pros::delay(150);
  chassis.moveToPoint(43,39,1000,{.forwards=false,.maxSpeed=50});
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  */

  // Right Side 9
  chassis.setPose(0, 0, 0);
  descore.set_value(true);
  intake.move_velocity(-11000);
  chassis.turnToPoint(11, 32, 500, {.maxSpeed = 65});
  chassis.moveToPoint(11, 32, 2000, {.maxSpeed = 80}, true);
  pros::delay(390);
  scraper.toggle();
  pros::delay(500);

  scraper.toggle();
  chassis.turnToPoint(31, 43, 750, {.maxSpeed = 80});
  chassis.moveToPoint(31, 43, 1000, {.maxSpeed = 70}, true);

  chassis.moveToPoint(11, 32, 2000, {.forwards = false, .maxSpeed = 80});

  chassis.turnToPoint(35, 4, 1000, {.forwards = false, .maxSpeed = 80});
  chassis.moveToPoint(35, 4, 2000, {.forwards = false, .maxSpeed = 80});

  chassis.turnToPoint(35, 23, 1000, {.forwards = false, .maxSpeed = 80});
  chassis.moveToPoint(35, 23, 900, {.forwards = false, .maxSpeed = 80}, false);
  pros::delay(100);
  hood.move_voltage(-11000);
  pros::delay(1400);
  hood.move_velocity(0);
  chassis.setPose(35, 19, 180);

  scraper.toggle();
  chassis.moveToPoint(34, -24, 1700, {.forwards = true, .maxSpeed = 60}, false);
  chassis.moveToPoint(35, 23, 1000, {.forwards = false, .maxSpeed = 80}, false);

  hood.move_voltage(-11000);
  scraper.toggle();
  pros::delay(750);
  chassis.turnToPoint(42, 9, 500, {.forwards = true, .maxSpeed = 80});
  chassis.moveToPoint(42, 9, 500, {.forwards = true, .maxSpeed = 80});
  chassis.turnToPoint(42, 40, 500, {.forwards = false, .maxSpeed = 80});
  descore.set_value(false);
  pros::delay(150);
  chassis.moveToPoint(43, 38, 1000, {.forwards = false, .maxSpeed = 50});
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

  /*
  //Left Side 9
   chassis.setPose(0, 0, 0);
   descore.set_value(true);
   intake.move_velocity(-11000);
   chassis.turnToPoint(-11, 32, 750, {.maxSpeed = 65});
   chassis.moveToPoint(-11, 32, 2000, {.maxSpeed = 80},true);
   pros::delay(400);
   scraper.toggle();
   pros::delay(600);


   scraper.toggle();
   chassis.turnToPoint(-31,43,750,{.maxSpeed=80});
   chassis.moveToPoint(-31,43,2500,{.maxSpeed=70},true);
    chassis.moveToPoint(-11, 32, 2000, {.forwards=false, .maxSpeed = 45});


   chassis.turnToPoint(-37,4,1000,{.forwards=false,.maxSpeed=80});
   chassis.moveToPoint(-37, 4, 2000, {.forwards=false, .maxSpeed = 80});


   chassis.turnToPoint(-37,23,1000,{.forwards=false, .maxSpeed=80});
   chassis.moveToPoint(-37,23,900,{.forwards=false, .maxSpeed=70}, false);
   pros::delay(200);
   hood.move_voltage(-11000);
   pros::delay(1250);
   hood.move_velocity(0);
   chassis.setPose(-37,19,180);


   scraper.toggle();
   chassis.moveToPoint(-37,-24,1700,{.forwards=true,.maxSpeed=60}, false);
   chassis.moveToPoint(-37,23,1000,{.forwards=false,.maxSpeed=80}, false);


   hood.move_voltage(-11000);
   pros::delay(750);
   chassis.turnToPoint(-30,10,500,{.forwards=true,.maxSpeed=80});
   chassis.moveToPoint(-30,10,500,{.forwards=true,.maxSpeed=80});
   chassis.turnToPoint(-30,23,500,{.forwards=false,.maxSpeed=80});
   descore.set_value(false);
   pros::delay(150);
   chassis.moveToPoint(-30,43,1000,{.forwards=false,.maxSpeed=50});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
   */
}

void opcontrol() {
  odomUp.set_value(true);
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
      descore.toggle();
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.move_voltage(11000);
      hood.move_voltage(0);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake.move_voltage(-11000);
      hood.move_voltage(0);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      hood.move_voltage(-5000);
      intake.move_voltage(-8000);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      hood.move_voltage(11000);
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

    // Toggle descore on L2 press
    toggleOnPress(descore, pros::E_CONTROLLER_DIGITAL_L2, controller);
    pros::delay(10);
  }
}
