enum class AcctuarDirPin : uint8_t {
  UPPER = 27, LOWER = 25
};

enum class AcctuarPin : uint8_t {
  UPPER = 23, LOWER = 10
};

enum class AcctuarController {
  UPPER = 0, LOWER = 1, BOTH = 2
};

enum class AcctuarControllerDir {
  NONE = 0, FORWARD = 1, BACKWARD = 2
};

enum class AcctuarDir {
  FORWARD = 2,
  BACKWARDS = 1,
  NONE = 0,
};
enum class Acctuar {
  UPPER = 0,
  LOWER = 1
};
enum class DrivingDir {
  FORWARD = 2,
  BACKWARDS = 1,
  NONE = 0,
};
enum class DriveDir{
  FORWARD = 0,
  BACKWARD = 1,
  LEFT = 2,
  RIGHT = 3,
  NONE,
};

enum class MotorsState {
  NONE = 0, DRIVING = 1, ROTATING = 2, DRIVING_BACKWARD = 3,
};
